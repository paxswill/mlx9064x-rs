use core::convert::{TryFrom, TryInto};

use num_enum::{IntoPrimitive, TryFromPrimitive};

use crate::common::Address;
use crate::error::LibraryError;
use crate::util::is_bit_set;

/// Trait for common register functionality.
pub(crate) trait Register: Into<[u8; 2]> + for<'a> From<&'a [u8]> {
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
#[derive(Clone, Copy, Debug)]
pub struct StatusRegister(u16);

impl StatusRegister {
    // Technically it's 0x0007, but only the first bit is used.
    const SUBPAGE_MASK: u16 = 0x0001;

    const NEW_DATA_MASK: u16 = 0x0008;

    const OVERWRITE_ENABLED_MASK: u16 = 0x0010;

    const STEP_MODE_MASK: u16 = 0x0020;

    /// The subpage which was last updated by the camera. Read-only.
    pub fn last_updated_subpage(&self) -> Subpage {
        // The three LSB are for the subpage, but only the LSB is used and the other bits are
        // reserved. Safe to unwrap as the only allowed subpage values are 0 and 1, and we're
        // masking off everything except a single bit.
        Subpage::try_from_primitive((self.0 & Self::SUBPAGE_MASK) as usize).unwrap()
    }

    /// Set when there is new data available in RAM. Read-write.
    ///
    /// This flag is set to by the camera, and can only be reset by the controller.
    pub fn new_data(&self) -> bool {
        self.0 & Self::NEW_DATA_MASK > 0
    }

    /// Reset the data available flag.
    pub fn reset_new_data(&mut self) {
        self.0 &= !Self::NEW_DATA_MASK
    }

    /// Whether data in RAM can be overwritten.
    pub fn overwrite_enabled(&self) -> bool {
        self.0 & Self::OVERWRITE_ENABLED_MASK > 0
    }

    /// Set whether or not data in RAM can be overwritten by the camera.
    pub fn set_overwrite_enabled(&mut self, overwrite_enabled: bool) {
        if overwrite_enabled {
            self.0 |= Self::OVERWRITE_ENABLED_MASK
        } else {
            self.0 &= !Self::OVERWRITE_ENABLED_MASK
        }
    }

    /// Start a measurement in step mode.
    ///
    /// This value must be enabled by the controller, and will then be reset by the camera once the
    /// measurement is complete. Each measurement covers one subpage, so if you want to retrieve
    /// both subpages you will need to trigger two measurements.
    /// This value is only applicable in step mode, and the documentation has been removed from
    /// more recent datasheets. See `ControlRegister::step_mode` for more details.
    pub fn start_measurement(&self) -> bool {
        self.0 & Self::STEP_MODE_MASK > 0
    }

    /// Signal to the camera that a step mode measurement is to be started.
    pub fn set_start_measurement(&mut self) {
        self.0 |= Self::STEP_MODE_MASK
    }
}

impl PartialEq for StatusRegister {
    fn eq(&self, other: &Self) -> bool {
        let mask = Self::SUBPAGE_MASK
            | Self::NEW_DATA_MASK
            | Self::OVERWRITE_ENABLED_MASK
            | Self::STEP_MODE_MASK;
        (self.0 & mask) == (other.0 & mask)
    }
}

impl Register for StatusRegister {
    fn write_mask() -> [u8; 2] {
        // The subpage bits are read-only, so they are not included in the write mask
        (Self::NEW_DATA_MASK | Self::OVERWRITE_ENABLED_MASK | Self::STEP_MODE_MASK).to_be_bytes()
    }

    fn address() -> Address {
        0x8000.into()
    }
}

impl<'a> From<&'a [u8]> for StatusRegister {
    /// Create a `StatusRegister` from the raw bytes read from the camera.
    ///
    /// This method will `panic` if there aren't enough bytes in the slice.
    fn from(buf: &'a [u8]) -> Self {
        let (int_bytes, _) = buf.split_at(core::mem::size_of::<u16>());
        let int_bytes: [u8; 2] = int_bytes
            .try_into()
            .expect("Not enough bytes in status register buffer");
        let raw = u16::from_be_bytes(int_bytes);
        Self(raw)
    }
}

impl From<StatusRegister> for [u8; 2] {
    fn from(status: StatusRegister) -> Self {
        status.0.to_be_bytes()
    }
}

/// Represents the possible states of the control register (0x800D).
#[derive(Clone, Copy, Debug)]
pub struct ControlRegister(u16);

impl ControlRegister {
    const USE_SUBPAGES_MASK: u16 = 0x0001;

    const STEP_MODE_MASK: u16 = 0x0002;

    const DATA_HOLD_MASK: u16 = 0x0004;

    const SUBPAGE_REPEAT_MASK: u16 = 0x0008;

    // Technically the mask is 0x0070, but the other bits are reserved
    const SUBPAGE_MASK: u16 = 0x0010;

    const FRAME_RATE_MASK: u16 = 0x0380;

    const RESOLUTION_MASK: u16 = 0x0C00;

    const ACCESS_PATTERN_MASK: u16 = 0x1000;

    const MLX90640_DEFAULT: u16 = 0x1901;

    const MLX90641_DEFAULT: u16 = 0x0901;

    /// The default settings (as documented in the datasheet) for the MLX90640.
    pub fn default_mlx90640() -> Self {
        Self(Self::MLX90640_DEFAULT)
    }

    /// The default settings (as documented in the datasheet) for the MLX90641.
    pub fn default_mlx90641() -> Self {
        Self(Self::MLX90641_DEFAULT)
    }

    /// Check if subpages are enabled.
    ///
    /// If subpages are disabled, only one page will be updated.
    ///
    /// The default is to use subpages.
    pub fn use_subpages(&self) -> bool {
        self.0 & Self::USE_SUBPAGES_MASK > 0
    }

    /// Enable or disable the use of subpages.
    pub fn set_use_subpages(&mut self, use_subpages: bool) {
        if use_subpages {
            self.0 |= Self::USE_SUBPAGES_MASK
        } else {
            self.0 &= !Self::USE_SUBPAGES_MASK
        }
    }

    /// Check if step mode is enabled.
    ///
    /// In step mode the camera is idle until signalled with [`StatusRegister::start_measurement`],
    /// which then starts a single measurement. The camera is not calibrated for step mode, and the
    /// values are inaccurate. Melexis no longer documents step mode because of this.
    ///
    /// The default is continuous mode (i.e. step mode disabled).
    pub fn step_mode(&self) -> bool {
        self.0 & Self::STEP_MODE_MASK > 0
    }

    /// Enable or disable step mode.
    ///
    /// It is not recommended to enable step mode, see [`step_mode`] for more details.
    pub fn set_step_mode(&mut self, step_mode: bool) {
        if step_mode {
            self.0 |= Self::STEP_MODE_MASK
        } else {
            self.0 &= !Self::STEP_MODE_MASK
        }
    }

    /// Check if data holding is enabled.
    ///
    /// By default data is transferred into RAM for each frame, but if this flag is enabled data
    /// will only be written into RAM when the `StatusRegister::overwrite_enabled` flag is set.
    pub fn data_hold(&self) -> bool {
        self.0 & Self::DATA_HOLD_MASK > 0
    }

    /// Enable or disable data holding.
    pub fn set_data_hold(&mut self, data_hold: bool) {
        if data_hold {
            self.0 |= Self::DATA_HOLD_MASK
        } else {
            self.0 &= !Self::DATA_HOLD_MASK
        }
    }

    /// Check to see if the camera automatically alternates between subpages.
    ///
    /// This value only has an effect when `use_subpages` is enabled. The default is disabled,
    /// meaning the camera automatically alternates between subpages.
    pub fn subpage_repeat(&self) -> bool {
        self.0 & Self::SUBPAGE_REPEAT_MASK > 0
    }

    /// Enable or disable subpage repetition.
    pub fn set_subpage_repeat(&mut self, subpage_repeat: bool) {
        if subpage_repeat {
            self.0 |= Self::SUBPAGE_REPEAT_MASK
        } else {
            self.0 &= !Self::SUBPAGE_REPEAT_MASK
        }
    }

    /// Check which subpage will be written to.
    ///
    /// This value only has an effect if *both* [`use_subpages`] and [`subpage_repeat`] are
    /// enabled. The default is [`0`][Subpage::Zero].
    pub fn subpage(&self) -> Subpage {
        let subpage_num = (self.0 & Self::SUBPAGE_MASK) >> (Self::SUBPAGE_MASK.trailing_zeros());
        // Safe to unwrap as only 0 and 1 are allowable values, and all but one bit are masked off
        Subpage::try_from_primitive(subpage_num as usize).unwrap()
    }

    /// Set the subpage to update.
    pub fn set_subpage(&mut self, subpage: Subpage) {
        let subpage_num = subpage as u16;
        self.0 = self.0 & !Self::SUBPAGE_MASK | (subpage_num << Self::SUBPAGE_MASK.trailing_zeros())
    }

    /// The frame rate the camera runs at.
    ///
    /// See the note on `FrameRate` for I²C bus clock rate requirements. The default is
    /// [2Hz][FrameRate::Two].
    pub fn frame_rate(&self) -> FrameRate {
        let raw = (self.0 & Self::FRAME_RATE_MASK) >> Self::FRAME_RATE_MASK.trailing_zeros();
        // Safe to unwrap as there are only eight values, and only three bits are unmasked
        FrameRate::from_raw(raw).unwrap()
    }

    /// Set the camera's frame rate.
    pub fn set_frame_rate(&mut self, frame_rate: FrameRate) {
        let raw = frame_rate.as_raw();
        self.0 = self.0 & !Self::FRAME_RATE_MASK | (raw << Self::FRAME_RATE_MASK.trailing_zeros())
    }

    /// The resolution to run the internal ADC at.
    ///
    /// The default is [18 bits][Resolution::Eighteen].
    pub fn resolution(&self) -> Resolution {
        let raw = (self.0 & Self::RESOLUTION_MASK) >> (Self::RESOLUTION_MASK.trailing_zeros());
        // Safe to unwrap as there are only four values, and only two bits are unmasked
        Resolution::from_raw(raw).unwrap()
    }

    /// Set the camera's resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        let raw = resolution.as_raw();
        self.0 = self.0 & !Self::RESOLUTION_MASK | (raw << Self::RESOLUTION_MASK.trailing_zeros())
    }

    /// The [access pattern][AccessPattern] used by the camera.
    ///
    /// The default for the MLX90640 is the [chess pattern][AccessPattern::Chess] mode, while the
    /// default for the MLX90641 is [interleaved][AccessPattern::Interleave].
    pub fn access_pattern(&self) -> AccessPattern {
        let raw =
            (self.0 & Self::ACCESS_PATTERN_MASK) >> (Self::ACCESS_PATTERN_MASK.trailing_zeros());
        // Safe to unwrap as only 0 and 1 are allowable values, and all but one bit are masked off
        AccessPattern::try_from_primitive(raw as u8).unwrap()
    }

    /// Set the access pattern to use.
    pub fn set_access_pattern(&mut self, access_pattern: AccessPattern) {
        let raw = access_pattern as u8;
        self.0 = self.0 & !Self::ACCESS_PATTERN_MASK
            | ((raw as u16) << Self::ACCESS_PATTERN_MASK.trailing_zeros())
    }
}

impl PartialEq for ControlRegister {
    fn eq(&self, other: &Self) -> bool {
        let mask = Self::USE_SUBPAGES_MASK
            | Self::STEP_MODE_MASK
            | Self::DATA_HOLD_MASK
            | Self::SUBPAGE_REPEAT_MASK
            | Self::SUBPAGE_MASK
            | Self::FRAME_RATE_MASK
            | Self::RESOLUTION_MASK
            | Self::ACCESS_PATTERN_MASK;
        (self.0 & mask) == (other.0 & mask)
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
        Self(raw)
    }
}

impl From<ControlRegister> for [u8; 2] {
    fn from(register: ControlRegister) -> Self {
        register.0.to_be_bytes()
    }
}

/// Represents the possible states of the I²C configuration register (0x800F).
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct I2cRegister {
    fast_mode_plus: bool,

    i2c_threshold_halved: bool,

    sda_current_limiter: bool,
}

impl I2cRegister {
    /// Check if I²C Fast Mode+ enabled.
    ///
    /// Defaults to enabled.
    pub fn fast_mode_plus(&self) -> bool {
        self.fast_mode_plus
    }

    /// Enable or disable I²C Fast Mode+.
    pub fn set_fast_mode_plus(&mut self, fast_mode_plus: bool) {
        self.fast_mode_plus = fast_mode_plus;
    }

    /// Check if the I²C threshold level is halved.
    ///
    /// The default is disabled.
    pub fn i2c_threshold_halved(&self) -> bool {
        self.i2c_threshold_halved
    }

    /// Enable or disable halving the I²C threshold level.
    pub fn set_i2c_threshold_halved(&mut self, i2c_threshold_halved: bool) {
        self.i2c_threshold_halved = i2c_threshold_halved;
    }

    /// Check if the I²C current limit is enabled.
    ///
    /// The defualt is enabled.
    pub fn sda_current_limiter(&self) -> bool {
        self.sda_current_limiter
    }

    /// Enable or disable the I²C current limiter.
    pub fn set_sda_current_limiter(&mut self, sda_current_limiter: bool) {
        self.sda_current_limiter = sda_current_limiter;
    }
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
    /// This is the default (and strongly recommended value) for the MLX90640. The MLX90641 updates
    /// the entire frame at once with only the read location being alternated. This means that
    /// chess mode has no benefit for the MLX90641, and this library doesn't support it.
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
            assert_eq!(packed.$field(), $expected);
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
        // Testing each subpage individually to ensure the current subpage is preserved when
        // resetting the data ready flag.
        // Subpage 0
        let mut subpage0_ready: StatusRegister = (&[0x00, 0x08][..]).into();
        assert_eq!(subpage0_ready.last_updated_subpage(), Subpage::Zero);
        assert!(subpage0_ready.new_data());
        subpage0_ready.reset_new_data();
        assert!(!subpage0_ready.new_data());
        let subpage0_bytes: [u8; 2] = subpage0_ready.into();
        assert_eq!(subpage0_bytes, [0x00, 0x00]);
        // Subpage 1
        let mut subpage1_ready: StatusRegister = (&[0x00, 0x09][..]).into();
        assert_eq!(subpage1_ready.last_updated_subpage(), Subpage::One);
        assert!(subpage1_ready.new_data());
        subpage1_ready.reset_new_data();
        assert!(!subpage1_ready.new_data());
        let subpage1_bytes: [u8; 2] = subpage1_ready.into();
        assert_eq!(subpage1_bytes, [0x00, 0x01]);
    }

    #[test]
    fn status_register_overwrite() {
        assert_register_field!(StatusRegister, 0x0010, overwrite_enabled, true);
        assert_register_field!(StatusRegister, 0x0000, overwrite_enabled, false);
        let mut status_register: StatusRegister = (&[0x00, 0x00][..]).into();
        assert!(!status_register.overwrite_enabled());
        status_register.set_overwrite_enabled(true);
        assert!(status_register.overwrite_enabled());
        status_register.set_overwrite_enabled(false);
        assert!(!status_register.overwrite_enabled());
    }

    #[test]
    fn status_register_start_measurement() {
        assert_register_field!(StatusRegister, 0x0020, start_measurement, true);
        assert_register_field!(StatusRegister, 0x0000, start_measurement, false);
        let mut status_register: StatusRegister = (&[0x00, 0x00][..]).into();
        assert!(!status_register.start_measurement());
        status_register.set_start_measurement();
        assert!(status_register.start_measurement());
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
        let mut control_register = ControlRegister::default_mlx90640();
        assert!(control_register.use_subpages());
        control_register.set_use_subpages(false);
        assert!(!control_register.use_subpages());
        control_register.set_use_subpages(true);
        assert!(control_register.use_subpages());
    }

    #[test]
    fn control_register_step_mode() {
        assert_register_field!(ControlRegister, 0x0002, step_mode, true);
        assert_register_field!(ControlRegister, 0x0000, step_mode, false);
        let mut control_register = ControlRegister::default_mlx90640();
        assert!(!control_register.step_mode());
        control_register.set_step_mode(true);
        assert!(control_register.step_mode());
        control_register.set_step_mode(false);
        assert!(!control_register.step_mode());
    }

    #[test]
    fn control_register_data_hold() {
        assert_register_field!(ControlRegister, 0x0004, data_hold, true);
        assert_register_field!(ControlRegister, 0x0000, data_hold, false);
        let mut control_register = ControlRegister::default_mlx90640();
        assert!(!control_register.data_hold());
        control_register.set_data_hold(true);
        assert!(control_register.data_hold());
        control_register.set_data_hold(false);
        assert!(!control_register.data_hold());
    }

    #[test]
    fn control_register_subpage_repeat() {
        assert_register_field!(ControlRegister, 0x0008, subpage_repeat, true);
        assert_register_field!(ControlRegister, 0x0000, subpage_repeat, false);
        let mut control_register = ControlRegister::default_mlx90640();
        assert!(!control_register.subpage_repeat());
        control_register.set_subpage_repeat(true);
        assert!(control_register.subpage_repeat());
        control_register.set_subpage_repeat(false);
        assert!(!control_register.subpage_repeat());
    }

    #[test]
    fn control_register_subpage() {
        assert_register_field!(ControlRegister, 0x0000, subpage, Subpage::Zero);
        assert_register_field!(ControlRegister, 0x0010, subpage, Subpage::One);
        let mut control_register = ControlRegister::default_mlx90640();
        assert_eq!(control_register.subpage(), Subpage::Zero);
        control_register.set_subpage(Subpage::One);
        assert_eq!(control_register.subpage(), Subpage::One);
        control_register.set_subpage(Subpage::Zero);
        assert_eq!(control_register.subpage(), Subpage::Zero);
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
        let mut control_register = ControlRegister::default_mlx90640();
        assert_eq!(control_register.frame_rate(), FrameRate::Two);
        control_register.set_frame_rate(FrameRate::Half);
        assert_eq!(control_register.frame_rate(), FrameRate::Half);
        control_register.set_frame_rate(FrameRate::One);
        assert_eq!(control_register.frame_rate(), FrameRate::One);
        control_register.set_frame_rate(FrameRate::Two);
        assert_eq!(control_register.frame_rate(), FrameRate::Two);
        control_register.set_frame_rate(FrameRate::Four);
        assert_eq!(control_register.frame_rate(), FrameRate::Four);
        control_register.set_frame_rate(FrameRate::Eight);
        assert_eq!(control_register.frame_rate(), FrameRate::Eight);
        control_register.set_frame_rate(FrameRate::Sixteen);
        assert_eq!(control_register.frame_rate(), FrameRate::Sixteen);
        control_register.set_frame_rate(FrameRate::ThirtyTwo);
        assert_eq!(control_register.frame_rate(), FrameRate::ThirtyTwo);
        control_register.set_frame_rate(FrameRate::SixtyFour);
        assert_eq!(control_register.frame_rate(), FrameRate::SixtyFour);
    }

    #[test]
    fn control_register_resolution() {
        assert_register_field!(ControlRegister, 0x0000, resolution, Resolution::Sixteen);
        assert_register_field!(ControlRegister, 0x0400, resolution, Resolution::Seventeen);
        assert_register_field!(ControlRegister, 0x0800, resolution, Resolution::Eighteen);
        assert_register_field!(ControlRegister, 0x0C00, resolution, Resolution::Nineteen);
        let mut control_register = ControlRegister::default_mlx90640();
        assert_eq!(control_register.resolution(), Resolution::Eighteen);
        control_register.set_resolution(Resolution::Sixteen);
        assert_eq!(control_register.resolution(), Resolution::Sixteen);
        control_register.set_resolution(Resolution::Seventeen);
        assert_eq!(control_register.resolution(), Resolution::Seventeen);
        control_register.set_resolution(Resolution::Eighteen);
        assert_eq!(control_register.resolution(), Resolution::Eighteen);
        control_register.set_resolution(Resolution::Nineteen);
        assert_eq!(control_register.resolution(), Resolution::Nineteen);
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
        let mut control_register = ControlRegister::default_mlx90640();
        assert_eq!(control_register.access_pattern(), AccessPattern::Chess);
        control_register.set_access_pattern(AccessPattern::Interleave);
        assert_eq!(control_register.access_pattern(), AccessPattern::Interleave);
        // Also check the MLX90641 default
        assert_eq!(
            ControlRegister::default_mlx90641().access_pattern(),
            AccessPattern::Interleave
        );
    }

    #[test]
    fn i2c_register_fast_mode_plus() {
        // "not disabled"
        assert_register_field!(I2cRegister, 0x0000, fast_mode_plus, true);
        assert_register_field!(I2cRegister, 0x0001, fast_mode_plus, false);
        let mut i2c_register = I2cRegister::default();
        assert!(i2c_register.fast_mode_plus());
        i2c_register.set_fast_mode_plus(false);
        assert!(!i2c_register.fast_mode_plus());
        i2c_register.set_fast_mode_plus(true);
        assert!(i2c_register.fast_mode_plus());
    }

    #[test]
    fn i2c_register_threshold() {
        assert_register_field!(I2cRegister, 0x0000, i2c_threshold_halved, false);
        assert_register_field!(I2cRegister, 0x0002, i2c_threshold_halved, true);
        let mut i2c_register = I2cRegister::default();
        assert!(!i2c_register.i2c_threshold_halved());
        i2c_register.set_i2c_threshold_halved(true);
        assert!(i2c_register.i2c_threshold_halved());
        i2c_register.set_i2c_threshold_halved(false);
        assert!(!i2c_register.i2c_threshold_halved());
    }

    #[test]
    fn i2c_register_current_limiter() {
        // the other "not disabled" flag
        assert_register_field!(I2cRegister, 0x0000, sda_current_limiter, true);
        assert_register_field!(I2cRegister, 0x0004, sda_current_limiter, false);
        let mut i2c_register = I2cRegister::default();
        assert!(i2c_register.sda_current_limiter());
        i2c_register.set_sda_current_limiter(false);
        assert!(!i2c_register.sda_current_limiter());
        i2c_register.set_sda_current_limiter(true);
        assert!(i2c_register.sda_current_limiter());
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
