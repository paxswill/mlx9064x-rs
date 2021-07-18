use core::convert::{TryFrom, TryInto};

use num_enum::{IntoPrimitive, TryFromPrimitive};

use crate::common::Address;
use crate::error::LibraryError;
use crate::util::is_bit_set;

/// Trait for common register functionality.
pub trait Register: Into<[u8; 2]> + for<'a> From<&'a [u8]> {
    /// A bit mask of which bits can be modified by the controller.
    ///
    /// When changing register values on the camera, the current value should be read, then
    /// bitwise-ANDed with the complement of this mask, then bitwise-ORd with the new value. This
    /// preserves the values of any reserved bits in the registers.
    fn write_mask() -> [u8; 2];

    /// The address of this register in the camera's memory map.
    fn address() -> Address;
}

/// Represents the possible states of the status register (0x8000).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct StatusRegister {
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
    fn write_mask() -> [u8; 2] {
        // Note that the three least significant bits are read-only.
        [0x00, 0x38]
    }

    fn address() -> Address {
        0x8000.into()
    }
}

impl<'a> From<&'a [u8]> for StatusRegister {
    /// Create a `StatusRegister` from the raw `u16` read from the camera.
    ///
    /// This method will `panic` if there aren't enough bytes in the slice.
    fn from(buf: &'a [u8]) -> Self {
        let (int_bytes, _) = buf.split_at(core::mem::size_of::<u16>());
        let int_bytes: [u8; 2] = int_bytes
            .try_into()
            .expect("Not enough bytes in status register buffer");
        let raw = u16::from_be_bytes(int_bytes);
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

impl From<StatusRegister> for [u8; 2] {
    fn from(status: StatusRegister) -> Self {
        let mut register = 0u16;
        let subpage_int: usize = status.last_updated_subpage.into();
        register |= subpage_int as u16;
        register |= (status.new_data as u16) << 3;
        register |= (status.overwrite_enabled as u16) << 4;
        register.to_be_bytes()
    }
}

/// Represents the possible states of the control register (0x800D).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
// skip formatting in here as rustfmt will remove the extra blank lines around the "extra" bit
// comments.
#[rustfmt::skip]
pub struct ControlRegister {
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
    /// By default data is transferred into RAM for each frame, but if this flag is enabled, data
    /// will only be written into RAM when the `StatusRegister::overwrite_enabled` flag is set.
    /// The default is disabled, meaning data will be written into RAM for every measurement.
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
    pub fn default_mlx90640() -> Self {
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
    pub fn default_mlx90641() -> Self {
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
    fn write_mask() -> [u8; 2] {
        // *Technically* it's 0x1FFD, but the second bit is documented to always be 0
        [0x1F, 0xFF]
    }

    fn address() -> Address {
        0x800D.into()
    }
}

impl<'a> From<&'a [u8]> for ControlRegister {
    fn from(buf: &'a [u8]) -> Self {
        let (int_bytes, _) = buf.split_at(core::mem::size_of::<u16>());
        let int_bytes: [u8; 2] = int_bytes
            .try_into()
            .expect("Not enough bytes in control register buffer");
        let raw = u16::from_be_bytes(int_bytes);
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
        let frame_rate = FrameRate::from_raw((raw & 0x0380) >> 7).unwrap();
        // Unwrap: Same as deal as frame_rate, except now it's two bits and there's only four
        // values.
        let resolution = Resolution::from_raw((raw & 0x0C00) >> 10).unwrap();
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

impl From<ControlRegister> for [u8; 2] {
    fn from(register: ControlRegister) -> Self {
        let mut raw = 0u16;
        raw |= register.use_subpages as u16;
        raw |= (register.step_mode as u16) << 1;
        raw |= (register.data_hold as u16) << 2;
        raw |= (register.subpage_repeat as u16) << 3;
        let subpage_int: usize = register.subpage.into();
        raw |= (subpage_int as u16) << 4;
        let frame_rate_int: u8 = register.frame_rate.as_raw() as u8;
        raw |= (frame_rate_int as u16) << 7;
        let resolution_int: u8 = register.resolution.as_raw() as u8;
        raw |= (resolution_int as u16) << 10;
        if register.access_pattern == AccessPattern::Chess {
            raw |= 1u16 << 12;
        }
        raw.to_be_bytes()
    }
}

/// Represents the possible states of the I²C configuration register (0x800F).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct I2cRegister {
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
    fn write_mask() -> [u8; 2] {
        // the fourth bit is documented, but it is "reserved" at 0. It *isn't* documented to always
        // be 0 though, so I'm not including it in the mask.
        [0x00, 0x07]
    }

    fn address() -> Address {
        0x800F.into()
    }
}

impl<'a> From<&'a [u8]> for I2cRegister {
    fn from(buf: &'a [u8]) -> Self {
        let (int_bytes, _) = buf.split_at(core::mem::size_of::<u16>());
        let int_bytes: [u8; 2] = int_bytes
            .try_into()
            .expect("Not enough bytes in status register buffer");
        let raw = u16::from_be_bytes(int_bytes);
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

impl From<I2cRegister> for [u8; 2] {
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
        raw.to_be_bytes()
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
/// * 100kHz: [4Hz][FrameRate::Four]
/// * 400kHz: [16Hz][FrameRate::Sixteen]
/// * 1MHz: [64Hz][FrameRate::Four] (barely, [32Hz][FrameRate::ThirtyTwo] is safer)
///
/// On top of this requirement, your hardware has to be fast enough to be able to process each
/// frame of data before the next frame is ready.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub enum FrameRate {
    /// 0.5 Hz, one frame every two seconds.
    Half,

    /// 1Hz.
    One,

    /// 2Hz, which is also the default for the MLX90640 and MLX90641.
    Two,

    // 4Hz.
    Four,

    // 8Hz.
    Eight,

    // 16 Hz.
    Sixteen,

    // 32Hz.
    ThirtyTwo,

    // 64Hz.
    SixtyFour,
}

impl FrameRate {
    /// Attempt to create a `FrameRate` from a raw value from the camera.
    pub(crate) fn from_raw(raw_value: u16) -> Result<Self, LibraryError> {
        match raw_value {
            0 => Ok(Self::Half),
            1 => Ok(Self::One),
            2 => Ok(Self::Two),
            3 => Ok(Self::Four),
            4 => Ok(Self::Eight),
            5 => Ok(Self::Sixteen),
            6 => Ok(Self::ThirtyTwo),
            7 => Ok(Self::SixtyFour),
            _ => Err(LibraryError::InvalidData("Invalid frame rate given")),
        }
    }

    /// Map a frame rate variant into the representation used by the camera.
    pub(crate) fn as_raw(&self) -> u16 {
        match self {
            Self::Half => 0,
            Self::One => 1,
            Self::Two => 2,
            Self::Four => 3,
            Self::Eight => 4,
            Self::Sixteen => 5,
            Self::ThirtyTwo => 6,
            Self::SixtyFour => 7,
        }
    }
}

impl Default for FrameRate {
    fn default() -> Self {
        Self::Two
    }
}

impl TryFrom<f32> for FrameRate {
    type Error = LibraryError;

    /// Attempt to create a `FrameRate` from a number.
    ///
    /// This will only work if the source number *exactly* matches one of the values named as a
    /// variant.
    /// ```
    /// # use core::convert::TryFrom;
    /// # use mlx9064x::FrameRate;
    /// assert_eq!(FrameRate::try_from(0.5), Ok(FrameRate::Half));
    /// let almost_half = 0.50001;
    /// assert!(FrameRate::try_from(almost_half).is_err());
    /// ```
    #[allow(clippy::float_cmp)]
    fn try_from(value: f32) -> Result<Self, Self::Error> {
        if value == 0.5 {
            Ok(Self::Half)
        } else if value == 1.0 {
            Ok(Self::One)
        } else if value == 2.0 {
            Ok(Self::Two)
        } else if value == 4.0 {
            Ok(Self::Four)
        } else if value == 8.0 {
            Ok(Self::Eight)
        } else if value == 16.0 {
            Ok(Self::Sixteen)
        } else if value == 32.0 {
            Ok(Self::ThirtyTwo)
        } else if value == 64.0 {
            Ok(Self::SixtyFour)
        } else {
            Err(LibraryError::InvalidData(
                "The given number does not match a valid frame rate",
            ))
        }
    }
}

impl TryFrom<u8> for FrameRate {
    type Error = LibraryError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        // No way to say 0.5, so skipping it
        match value {
            1 => Ok(Self::One),
            2 => Ok(Self::Two),
            4 => Ok(Self::Four),
            8 => Ok(Self::Eight),
            16 => Ok(Self::Sixteen),
            32 => Ok(Self::ThirtyTwo),
            64 => Ok(Self::SixtyFour),
            _ => Err(LibraryError::InvalidData(
                "The given number does not match a valid frame rate",
            )),
        }
    }
}

impl From<FrameRate> for f32 {
    fn from(frame_rate: FrameRate) -> Self {
        match frame_rate {
            FrameRate::Half => 0.5,
            FrameRate::One => 1f32,
            FrameRate::Two => 2f32,
            FrameRate::Four => 4f32,
            FrameRate::Eight => 8f32,
            FrameRate::Sixteen => 16f32,
            FrameRate::ThirtyTwo => 32f32,
            FrameRate::SixtyFour => 64f32,
        }
    }
}

/// The resolution of the internal [ADC][adc].
///
/// [adc]: https://en.wikipedia.org/wiki/Analog-to-digital_converter
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Resolution {
    /// 16-bit.
    Sixteen,

    /// 17-bit.
    Seventeen,

    /// 18-bit, which is also the default for both the MLX90640 and MLX90641.
    Eighteen,

    /// 19-bit.
    Nineteen,
}

impl Resolution {
    /// Attempt to create a `FrameRate` from a raw value from the camera.
    pub(crate) fn from_raw(raw_value: u16) -> Result<Self, LibraryError> {
        match raw_value {
            0 => Ok(Self::Sixteen),
            1 => Ok(Self::Seventeen),
            2 => Ok(Self::Eighteen),
            3 => Ok(Self::Nineteen),
            _ => Err(LibraryError::InvalidData(
                "Invalid raw resolution value given",
            )),
        }
    }

    /// Map a frame rate variant into the representation used by the camera.
    pub(crate) fn as_raw(&self) -> u16 {
        match self {
            Self::Sixteen => 0,
            Self::Seventeen => 1,
            Self::Eighteen => 2,
            Self::Nineteen => 3,
        }
    }
}

impl TryFrom<u8> for Resolution {
    type Error = LibraryError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            16 => Ok(Self::Sixteen),
            17 => Ok(Self::Seventeen),
            18 => Ok(Self::Eighteen),
            19 => Ok(Self::Nineteen),
            _ => Err(LibraryError::InvalidData(
                "The given value did not match a valid ADC resolution",
            )),
        }
    }
}

impl From<Resolution> for u8 {
    fn from(resolution: Resolution) -> Self {
        match resolution {
            Resolution::Sixteen => 16,
            Resolution::Seventeen => 17,
            Resolution::Eighteen => 18,
            Resolution::Nineteen => 19,
        }
    }
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
    Chess = 1,

    /// Each row of pixels is in the same subpage, with the rows alternating between subpages.
    ///
    /// This is the default mode for the MLX90641.
    Interleave = 0,
}

#[cfg(test)]
mod test {
    use super::*;

    macro_rules! assert_register_field {
        ($register:ty, $value:literal, $field:ident, $expected:expr) => {
            // backdoor type annotation for the macro
            let value: u16 = $value;
            let bytes = value.to_be_bytes();
            let packed: $register = From::from(&bytes[..]);
            assert_eq!(packed.$field, $expected);
            let unpacked: [u8; 2] = packed.into();
            assert_eq!(unpacked, bytes);
        };
    }

    #[test]
    fn status_register_masked() {
        // The write mask is not quite a mask for all relevant bits for the status register: the
        // last updated subpage is a read-only field, so the three least significant bits aren't in
        // the write mask.
        let bytes = [0xFF, 0xF8];
        let all_on = StatusRegister::from(&bytes[..]);
        let mask_bytes = StatusRegister::write_mask();
        let masked_on: StatusRegister = StatusRegister::from(&mask_bytes[..]);
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
        let bytes: [u8; 2] = [0xFF, 0xFF];
        let all_on = ControlRegister::from(&bytes[..]);
        // The write mask is also a mask for which bits are relevant to the structure.
        let mask_bytes = ControlRegister::write_mask();
        let masked_on = ControlRegister::from(&mask_bytes[..]);
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
    fn frame_rate_from_raw() {
        assert_eq!(FrameRate::from_raw(0).unwrap(), FrameRate::Half);
        assert_eq!(FrameRate::from_raw(1).unwrap(), FrameRate::One);
        assert_eq!(FrameRate::from_raw(2).unwrap(), FrameRate::Two);
        assert_eq!(FrameRate::from_raw(3).unwrap(), FrameRate::Four);
        assert_eq!(FrameRate::from_raw(4).unwrap(), FrameRate::Eight);
        assert_eq!(FrameRate::from_raw(5).unwrap(), FrameRate::Sixteen);
        assert_eq!(FrameRate::from_raw(6).unwrap(), FrameRate::ThirtyTwo);
        assert_eq!(FrameRate::from_raw(7).unwrap(), FrameRate::SixtyFour);
        assert!(FrameRate::from_raw(8).is_err());
    }

    #[test]
    fn frame_rate_as_raw() {
        assert_eq!(FrameRate::Half.as_raw(), 0);
        assert_eq!(FrameRate::One.as_raw(), 1);
        assert_eq!(FrameRate::Two.as_raw(), 2);
        assert_eq!(FrameRate::Four.as_raw(), 3);
        assert_eq!(FrameRate::Eight.as_raw(), 4);
        assert_eq!(FrameRate::Sixteen.as_raw(), 5);
        assert_eq!(FrameRate::ThirtyTwo.as_raw(), 6);
        assert_eq!(FrameRate::SixtyFour.as_raw(), 7);
    }

    #[test]
    fn frame_rate_from_f32() {
        assert_eq!(FrameRate::try_from(0.5f32).unwrap(), FrameRate::Half);
        assert_eq!(FrameRate::try_from(1f32).unwrap(), FrameRate::One);
        assert_eq!(FrameRate::try_from(2f32).unwrap(), FrameRate::Two);
        assert_eq!(FrameRate::try_from(4f32).unwrap(), FrameRate::Four);
        assert_eq!(FrameRate::try_from(8f32).unwrap(), FrameRate::Eight);
        assert_eq!(FrameRate::try_from(16f32).unwrap(), FrameRate::Sixteen);
        assert_eq!(FrameRate::try_from(32f32).unwrap(), FrameRate::ThirtyTwo);
        assert_eq!(FrameRate::try_from(64f32).unwrap(), FrameRate::SixtyFour);
        // Don't try to add more zeros into the next test; too many more and it gets truncated.
        assert!(FrameRate::try_from(0.5000001f32).is_err());
    }

    #[test]
    fn frame_rate_from_u8() {
        // No fractions in integer-land
        assert_eq!(FrameRate::try_from(1u8).unwrap(), FrameRate::One);
        assert_eq!(FrameRate::try_from(2u8).unwrap(), FrameRate::Two);
        assert_eq!(FrameRate::try_from(4u8).unwrap(), FrameRate::Four);
        assert_eq!(FrameRate::try_from(8u8).unwrap(), FrameRate::Eight);
        assert_eq!(FrameRate::try_from(16u8).unwrap(), FrameRate::Sixteen);
        assert_eq!(FrameRate::try_from(32u8).unwrap(), FrameRate::ThirtyTwo);
        assert_eq!(FrameRate::try_from(64u8).unwrap(), FrameRate::SixtyFour);
        assert!(FrameRate::try_from(u8::MAX).is_err());
    }

    #[test]
    fn frame_rate_to_f32() {
        assert_eq!(f32::from(FrameRate::Half), 0.5);
        assert_eq!(f32::from(FrameRate::One), 1f32);
        assert_eq!(f32::from(FrameRate::Two), 2f32);
        assert_eq!(f32::from(FrameRate::Four), 4f32);
        assert_eq!(f32::from(FrameRate::Eight), 8f32);
        assert_eq!(f32::from(FrameRate::Sixteen), 16f32);
        assert_eq!(f32::from(FrameRate::ThirtyTwo), 32f32);
        assert_eq!(f32::from(FrameRate::SixtyFour), 64f32);
    }

    #[test]
    fn default_frame_rate() {
        assert_eq!(FrameRate::default(), FrameRate::Two)
    }
    #[test]
    fn resolution_from_raw() {
        assert_eq!(Resolution::from_raw(0).unwrap(), Resolution::Sixteen);
        assert_eq!(Resolution::from_raw(1).unwrap(), Resolution::Seventeen);
        assert_eq!(Resolution::from_raw(2).unwrap(), Resolution::Eighteen);
        assert_eq!(Resolution::from_raw(3).unwrap(), Resolution::Nineteen);
        assert!(Resolution::from_raw(4).is_err());
    }

    #[test]
    fn resolution_as_raw() {
        assert_eq!(Resolution::Sixteen.as_raw(), 0);
        assert_eq!(Resolution::Seventeen.as_raw(), 1);
        assert_eq!(Resolution::Eighteen.as_raw(), 2);
        assert_eq!(Resolution::Nineteen.as_raw(), 3);
    }

    #[test]
    fn resolution_from_u8() {
        assert_eq!(Resolution::try_from(16u8).unwrap(), Resolution::Sixteen);
        assert_eq!(Resolution::try_from(17u8).unwrap(), Resolution::Seventeen);
        assert_eq!(Resolution::try_from(18u8).unwrap(), Resolution::Eighteen);
        assert_eq!(Resolution::try_from(19u8).unwrap(), Resolution::Nineteen);
        assert!(Resolution::try_from(1u8).is_err());
    }

    #[test]
    fn resolution_to_u8() {
        assert_eq!(u8::from(Resolution::Sixteen), 16);
        assert_eq!(u8::from(Resolution::Seventeen), 17);
        assert_eq!(u8::from(Resolution::Eighteen), 18);
        assert_eq!(u8::from(Resolution::Nineteen), 19);
    }

    #[test]
    fn default_resolution() {
        assert_eq!(Resolution::default(), Resolution::Eighteen);
    }
}
