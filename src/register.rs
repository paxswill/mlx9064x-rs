use num_enum::{IntoPrimitive, TryFromPrimitive};

// TODO: Investigate using bitvec to make the bit twiddling easier.

/// Trait for common register functionality.
pub(crate) trait Register: From<u16> + Into<u16> {
    /// A bit mask of which bits can be modified by the controller.
    ///
    /// When changing register values on the device, the current value should be read, then
    /// bitwise-ANDed with the complement of this mask, then bitwise-ORd with the new value. This
    /// preserves the values of any reserved bits in the registers.
    fn write_mask() -> u16;
}

/// Represents the possible states of the status register (0x8000).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct StatusRegister {
    /// The subpage which was last updated by the camera. Read-only.
    pub(crate) last_updated_subpage: Subpage,

    /// Set when there is new data available in RAM. Read-write.
    ///
    /// This flag is set to true by the camera, and can only be reset by the controller.
    pub(crate) new_data: bool,

    /// Whether data in RAM can be overwritten.
    pub(crate) overwrite_enabled: bool,

    /// Start a measurement in step mode.
    ///
    /// This value must be enabled by the controller, and will then be reset by the camera once the
    /// measurement is complete. Each measurement covers one subpage, so if you want to retrieve
    /// both subpages you will need to trigger two measurements.
    /// This value is only applicable in step mode, and the documentation has been removed from the
    /// more recent datasheets. See `ControlRegister::step_mode` for more details.
    pub(crate) start_measurement: bool,
}

impl Register for StatusRegister {
    fn write_mask() -> u16 {
        // Note that the three least significant bits are read-only.
        0x0038
    }
}

impl From<u16> for StatusRegister {
    /// Create a `StatusRegister` from the raw `u16` read from the camera.
    fn from(raw: u16) -> Self {
        // Only the first bit is used, the other two bits for this value are "Melxis" reserved
        let subpage = 0x0001 & raw;
        let new_data = is_bit_set(raw, 3);
        let overwrite_enabled = is_bit_set(raw, 4);
        let start_measurement = is_bit_set(raw, 5);
        Self {
            // Safe to unwrap, as there are only two possible values, 0 or 1, as above we're
            // masking off all but the first bit.
            last_updated_subpage: Subpage::try_from_primitive(subpage as usize).unwrap(),
            new_data,
            overwrite_enabled,
            start_measurement,
        }
    }
}

impl From<StatusRegister> for u16 {
    fn from(status: StatusRegister) -> Self {
        let mut register = 0u16;
        let subpage_int: usize = status.last_updated_subpage.into();
        register |= subpage_int as u16;
        register |= (status.new_data as u16) << 3;
        register |= (status.overwrite_enabled as u16) << 4;
        register
    }
}

/// Represents the possible states of the control register (0x800D).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
// skip formatting in here as rustfmt will remove the extra blank lines around the "extra" bit
// comments.
#[rustfmt::skip]
pub(crate) struct ControlRegister {
    // The fields in this struct are laid out in least to most significant bits they occupy in the
    // control register.

    /// Whether or not to use subpages.
    ///
    /// If subpages are disabled, only one page will be updated. The default is enabled
    pub(crate) use_subpages: bool,

    /// Enable "step mode".
    ///
    /// This mode is documented in older versions of the datasheet, but was later removed. In "step
    /// mode", the camera is idle until signalled, then a single measurement is taken.
    ///
    /// The default is continuous mode (ie "step mode" disabled).
    pub(crate) step_mode: bool,

    /// Enabled data hold.
    ///
    /// The default is disabled.
    // TODO: puzzle out what the datasheet means by this.
    pub(crate) data_hold: bool,

    /// Whether or not to automatically alternate between subpages.
    ///
    /// This value only has an effect when `use_subpages` is enabled. The default is disabled.
    pub(crate) subpage_repeat: bool,

    /// Which subpage to use.
    ///
    /// This value only has an effect if *both* `use_subpages` and `subpage_repeat` are enabled.
    /// The default is [`0`][Subpage::Zero].
    pub(crate) subpage: Subpage,

    // `subpage` takes up three bits.

    /// The frame rate the camera should run at.
    ///
    /// See the note on `FrameRate` for I²C bus clock rate requirements. Teh default is
    /// [2Hz][FrameRate::Two].
    pub(crate) frame_rate: FrameRate,

    // `frame_rate` takes up three bits

    /// The resolution to run the internal ADC at.
    ///
    /// The default is [18 bits][Resolution::Eighteen].
    pub(crate) resolution: Resolution,

    // `resolution` takes up two bits.

    /// Which access pattern to use.
    ///
    /// The default for the MLX90640 is the [chess pattern][AccessPattern::Chess] mode, while the
    /// default for the MLX90641 is [interleaved][AccessPattern::Interleave].
    pub(crate) access_pattern: AccessPattern,

    // The rest of the bits are reserved.
}

impl ControlRegister {
    /// The default settings (as documented in the datasheet) for the MLX90640.
    pub(crate) fn default_mlx90640() -> Self {
        Self {
            use_subpages: true,
            step_mode: false,
            data_hold: false,
            subpage_repeat: false,
            subpage: Subpage::Zero,
            frame_rate: FrameRate::default(),
            resolution: Resolution::default(),
            // This is the only value specific to the '640
            access_pattern: AccessPattern::Chess,
        }
    }

    /// The default settings (as documented in the datasheet) for the MLX90641.
    pub(crate) fn default_mlx90641() -> Self {
        Self {
            use_subpages: true,
            step_mode: false,
            data_hold: false,
            subpage_repeat: false,
            subpage: Subpage::Zero,
            frame_rate: FrameRate::default(),
            resolution: Resolution::default(),
            // This is the only value specific to the '641
            access_pattern: AccessPattern::Interleave,
        }
    }
}

impl Register for ControlRegister {
    fn write_mask() -> u16 {
        // *Technically* it's 0x1FFD, but the second bit is documented to always be 0
        0x1FFF
    }
}

impl From<u16> for ControlRegister {
    fn from(raw: u16) -> Self {
        let use_subpages = is_bit_set(raw, 0);
        let step_mode = is_bit_set(raw, 1);
        let data_hold = is_bit_set(raw, 2);
        let subpage_repeat = is_bit_set(raw, 3);
        let subpage = if is_bit_set(raw, 4) {
            Subpage::One
        } else {
            Subpage::Zero
        };
        // Unwrapping is safe here as the only values valid for FrameRate are 0-7, and we're
        // masking off all but 3 bits.
        let frame_rate = FrameRate::try_from_primitive(((raw & 0x0380) >> 7) as u8).unwrap();
        // Unwrap: Same as deal as frame_rate, except now it's two bits and there's only four
        // values.
        let resolution = Resolution::try_from_primitive(((raw & 0x0C00) >> 10) as u8).unwrap();
        let access_pattern = if is_bit_set(raw, 12) {
            AccessPattern::Chess
        } else {
            AccessPattern::Interleave
        };
        Self {
            use_subpages,
            step_mode,
            data_hold,
            subpage_repeat,
            subpage,
            frame_rate,
            resolution,
            access_pattern,
        }
    }
}

impl From<ControlRegister> for u16 {
    fn from(register: ControlRegister) -> Self {
        let mut raw = 0u16;
        raw |= register.use_subpages as u16;
        raw |= (register.step_mode as u16) << 1;
        raw |= (register.data_hold as u16) << 2;
        raw |= (register.subpage_repeat as u16) << 3;
        let subpage_int: usize = register.subpage.into();
        raw |= (subpage_int as u16) << 4;
        let frame_rate_int: u8 = register.frame_rate.into();
        raw |= (frame_rate_int as u16) << 7;
        let resolution_int: u8 = register.resolution.into();
        raw |= (resolution_int as u16) << 10;
        if register.access_pattern == AccessPattern::Chess {
            raw |= 1u16 << 12;
        }
        raw
    }
}

/// Represents the possible states of the I²C configuration register (0x800F).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct I2cRegister {
    /// Is Fast Mode+ (FM+) enabled?
    ///
    /// Defaults to enabled.
    pub(crate) fast_mode_plus: bool,

    /// Halve the I²C threshold level?
    ///
    /// Defaults to disabled.
    // I don't know what this actually means.
    pub(crate) i2c_threshold_halved: bool,

    /// Whether or not to enable the SDA current limit.
    ///
    /// The default is for it to be enabled
    pub(crate) sda_current_limiter: bool,
}

impl Default for I2cRegister {
    fn default() -> Self {
        Self {
            fast_mode_plus: true,
            i2c_threshold_halved: false,
            sda_current_limiter: true,
        }
    }
}

impl Register for I2cRegister {
    fn write_mask() -> u16 {
        // the fourth bit is documented, but it is "reserved" at 0. It *isn't* documented to always
        // be 0 though, so I'm not including it in the mask.
        0x0007
    }
}

impl From<u16> for I2cRegister {
    fn from(raw: u16) -> Self {
        // I'm inverting the fast mode plus flag for usage in this library. 0 should always be
        // "disabled", not "not disabled".
        let fast_mode_plus = !is_bit_set(raw, 0);
        let i2c_threshold_halved = is_bit_set(raw, 1);
        // Like fast_mode_plus, I'm rephrasing this flag to be the opposite
        let sda_current_limiter = !is_bit_set(raw, 2);
        Self {
            fast_mode_plus,
            i2c_threshold_halved,
            sda_current_limiter,
        }
    }
}

impl From<I2cRegister> for u16 {
    fn from(register: I2cRegister) -> Self {
        let mut raw = 0u16;
        // And here we translate back to "not disabled" land.
        if !register.fast_mode_plus {
            raw |= 0x0001;
        }
        if register.i2c_threshold_halved {
            raw |= 0x0002;
        }
        // And the other "not disabled" bit.
        if !register.sda_current_limiter {
            raw |= 0x0004;
        }
        raw
    }
}

/// Identify which subpage to access.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(usize)]
pub enum Subpage {
    Zero = 0,
    One = 1,
}

/// The possible refresh rates supported by the cameras. Before using the higher refresh rates,
/// ensure your I²C bus is fast enough. A quick rundown of the the maximum frame rate some common
/// I²C bus speeds can support:
///
/// * 100kHz: [4Hz][RefreshRate::Four]
/// * 400kHz: [16Hz][RefreshRate::Sixteen]
/// * 1MHz: [64Hz][RefreshRate::Four] (barely, [32Hz][RefreshRate::ThirtyTwo] is safer)
///
/// On top of this requirement, your hardware has to be fast enough to be able to process each
/// frame of data before the next frame is ready.
///
/// NOTE: The discriminant values are the raw values used in the control register. The discriminant
/// values do *not* match the frame rate values (ex: [`SixtyFour`][FrameRate::SixtyFour] has a
/// discriminant of 7). This also means that `IntoPrimitive` and `TryFromPrimitive` will *not* use
/// the values you would assume.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum FrameRate {
    /// 0.5 Hz, one frame every two seconds.
    Half = 0,

    /// 1Hz.
    One = 1,

    /// 2Hz, which is also the default for the MLX90640 and MLX90641.
    Two = 2,

    // 4Hz.
    Four = 3,

    // 8Hz.
    Eight = 4,

    // 16 Hz.
    Sixteen = 5,

    // 32Hz.
    ThirtyTwo = 6,

    // 64Hz.
    SixtyFour = 7,
}

impl Default for FrameRate {
    fn default() -> Self {
        Self::Two
    }
}

/// The resolution of the internal [ADC][adc].
///
/// [adc]: https://en.wikipedia.org/wiki/Analog-to-digital_converter
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum Resolution {
    /// 16-bit.
    Sixteen = 0,

    /// 17-bit.
    Seventeen = 1,

    /// 18-bit, which is also the default for both the MLX90640 and MLX90641.
    Eighteen = 2,

    /// 19-bit.
    Nineteen = 3,
}

impl Default for Resolution {
    fn default() -> Self {
        Self::Eighteen
    }
}

/// The pixel access pattern used by a camera.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum AccessPattern {
    /// Pixels alternate between subpages, resulting in a chess or checker board pattern.
    ///
    /// This is the default (and strongly recommended value) for the MLX909640. While the MLX909641
    /// datasheet mentions this mode in the register section, no further mention is made in the
    /// datasheet, so I'm not sure it's actually supported.
    Chess,

    /// Each row of pixels is in the same subpage, with the rows alternating between subpages.
    ///
    /// This is the default mode for the MLX90641.
    Interleave,
}

/// Check if the n-th bit is set.
///
/// Bits are 0-indexed, from the LSB.
fn is_bit_set(value: u16, index: u16) -> bool {
    (value & (1 << index)) > 0
}

#[cfg(test)]
mod test {
    use super::*;

    macro_rules! assert_register_field {
        ($register:ty, $value:literal, $field:ident, $expected:expr) => {
            let packed: $register = From::from($value);
            assert_eq!(packed.$field, $expected);
            let unpacked: u16 = packed.into();
            assert_eq!(unpacked, $value);
        };
    }

    #[test]
    fn status_register_masked() {
        // The write mask is not quite a mask for all relevant bits for the status register: the
        // last updated subpage is a read-only field, so the three least significant bits aren't in
        // the write mask.
        let all_on = StatusRegister::from(0xFFF8);
        let masked_on = StatusRegister::from(StatusRegister::write_mask());
        assert_eq!(all_on, masked_on);
    }

    #[test]
    fn status_register_last_updated_subpage() {
        assert_register_field!(StatusRegister, 0x0001, last_updated_subpage, Subpage::One);
        assert_register_field!(StatusRegister, 0x0000, last_updated_subpage, Subpage::Zero);
    }

    #[test]
    fn status_register_new_data() {
        assert_register_field!(StatusRegister, 0x0008, new_data, true);
        assert_register_field!(StatusRegister, 0x0000, new_data, false);
    }

    #[test]
    fn status_register_overwrite() {
        assert_register_field!(StatusRegister, 0x0010, overwrite_enabled, true);
        assert_register_field!(StatusRegister, 0x0000, overwrite_enabled, false);
    }

    #[test]
    fn control_register_masked() {
        let all_on = ControlRegister::from(0xFFFF);
        // The write mask is also a mask for which bits are relevant to the structure.
        let masked_on = ControlRegister::from(ControlRegister::write_mask());
        assert_eq!(all_on, masked_on);
    }

    #[test]
    fn control_register_use_subpages() {
        assert_register_field!(ControlRegister, 0x0001, use_subpages, true);
        assert_register_field!(ControlRegister, 0x0000, use_subpages, false);
    }

    #[test]
    fn control_register_step_mode() {
        assert_register_field!(ControlRegister, 0x0002, step_mode, true);
        assert_register_field!(ControlRegister, 0x0000, step_mode, false);
    }

    #[test]
    fn control_register_data_hold() {
        assert_register_field!(ControlRegister, 0x0004, data_hold, true);
        assert_register_field!(ControlRegister, 0x0000, data_hold, false);
    }

    #[test]
    fn control_register_subpage_repeat() {
        assert_register_field!(ControlRegister, 0x0008, subpage_repeat, true);
        assert_register_field!(ControlRegister, 0x0000, subpage_repeat, false);
    }

    #[test]
    fn control_register_subpage() {
        assert_register_field!(ControlRegister, 0x0000, subpage, Subpage::Zero);
        assert_register_field!(ControlRegister, 0x0010, subpage, Subpage::One);
    }

    #[test]
    fn control_register_frame_rate() {
        assert_register_field!(ControlRegister, 0x0000, frame_rate, FrameRate::Half);
        assert_register_field!(ControlRegister, 0x0080, frame_rate, FrameRate::One);
        assert_register_field!(ControlRegister, 0x0100, frame_rate, FrameRate::Two);
        assert_register_field!(ControlRegister, 0x0180, frame_rate, FrameRate::Four);
        assert_register_field!(ControlRegister, 0x0200, frame_rate, FrameRate::Eight);
        assert_register_field!(ControlRegister, 0x0280, frame_rate, FrameRate::Sixteen);
        assert_register_field!(ControlRegister, 0x0300, frame_rate, FrameRate::ThirtyTwo);
        assert_register_field!(ControlRegister, 0x0380, frame_rate, FrameRate::SixtyFour);
    }

    #[test]
    fn control_register_resolution() {
        assert_register_field!(ControlRegister, 0x0000, resolution, Resolution::Sixteen);
        assert_register_field!(ControlRegister, 0x0400, resolution, Resolution::Seventeen);
        assert_register_field!(ControlRegister, 0x0800, resolution, Resolution::Eighteen);
        assert_register_field!(ControlRegister, 0x0C00, resolution, Resolution::Nineteen);
    }

    #[test]
    fn control_register_access_mode() {
        assert_register_field!(
            ControlRegister,
            0x0000,
            access_pattern,
            AccessPattern::Interleave
        );
        assert_register_field!(
            ControlRegister,
            0x1000,
            access_pattern,
            AccessPattern::Chess
        );
    }

    #[test]
    fn i2c_register_fast_mode_plus() {
        // "not disabled"
        assert_register_field!(I2cRegister, 0x0000, fast_mode_plus, true);
        assert_register_field!(I2cRegister, 0x0001, fast_mode_plus, false);
    }

    #[test]
    fn i2c_register_threshold() {
        assert_register_field!(I2cRegister, 0x0000, i2c_threshold_halved, false);
        assert_register_field!(I2cRegister, 0x0002, i2c_threshold_halved, true);
    }

    #[test]
    fn i2c_register_current_limiter() {
        // the other "not disabled" flag
        assert_register_field!(I2cRegister, 0x0000, sda_current_limiter, true);
        assert_register_field!(I2cRegister, 0x0004, sda_current_limiter, false);
    }

    #[test]
    fn default_frame_rate() {
        assert_eq!(FrameRate::default(), FrameRate::Two)
    }

    #[test]
    fn default_resolution() {
        assert_eq!(Resolution::default(), Resolution::Eighteen);
    }

    #[test]
    fn is_bit_set() {
        for n in 0..16 {
            let value: u16 = 1 << n;
            assert!(
                super::is_bit_set(value, n),
                "is_bit_set was incorrect for bit {}",
                n
            );
        }
    }
}
