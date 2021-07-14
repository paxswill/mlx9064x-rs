use core::convert::TryFrom;

use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

use crate::address_enum_ops;
use crate::camera::Address;

/// EEPROM addresses for th MLX90640

// NOTE: To make it easier to compare against the datasheet, discriminant values should *always* be
// explicitly written out.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
pub(super) enum EepromAddress {
    /// The start of the EEPROM address range. For calibration purposes, the 16 words (word size is
    /// 16 bits) can be ignored.
    Base = 0x2400,

    /// Covers α PTAT and offset compensation scaling factors (row, column and remainder).
    OffsetCompensation = 0x2410,

    PixelOffsetAverage = 0x2411,

    OffsetCompensationRowStart = 0x2412,

    OffsetCompensationColumnStart = 0x2418,

    /// Covers α scaling factor, and pixel α compensation scaling factors.
    SensitivityScale = 0x2420,

    PixelSensitivityAverage = 0x2421,

    PixelSensitivityCompensationRowStart = 0x2422,

    PixelSensitivityCompensationColumnStart = 0x2428,

    Gain = 0x2430,

    Ptat25 = 0x2431,

    /// Covers both K<sub>V<sub>PTAT</sub></sub> and K<sub>t<sub>PTAT</sub></sub>
    PtatConstants = 0x2432,

    /// Covers both Kv<sub>V<sub>DD</sub></sub> and V<sub>DD<sub>25</sub></sub>
    VddConstants = 0x2433,

    AverageVoltageConstants = 0x2434,

    InterlacedModeCompensation = 0x2435,

    /// Covers both even and odd row indexes for K<sub>T<sub>a</sub>average</sub> for even column
    /// indicies.
    ///
    /// NOTE: this library uses 0-indexing, while the datasheet uses 1-indexing. Because of this,
    /// what this library considers even, the datasheet considers odd and vice-versa. This address
    /// is for the columns the *library* considers even (and what the datasheet considers odd).
    AmbientTemperatureAverageConstantsEvenColumns = 0x2436,

    /// Covers both even and odd row indexes for K<sub>T<sub>a</sub>average</sub> for odd column
    /// indices.
    ///
    /// NOTE: this library uses 0-indexing, while the datasheet uses 1-indexing. Because of this,
    /// what this library considers even, the datasheet considers odd and vice-versa. This address
    /// is for the columns the *library* considers even (and what the datasheet considers odd).
    AmbientTemperatureAverageConstantsOddColumns = 0x2437,

    /// Contains scaling factors for K<sub>V</sub> and V<sub>T<sub>a</sub></sub>, as well as the
    /// resolution control calibration.
    VAndTaScale = 0x2438,

    CompensationPixelSensitivity = 0x2439,

    CompensationPixelOffset = 0x243A,

    /// K<sub>v</sub> and K<sub>T<sub>a</sub></sub> for the compensation pixels.
    CompensationPixelConstants = 0x243B,

    /// Covers both K<sub>S<sub>T<sub>0</sub></sub></sub> and the temperature gradient coefficient.
    AmbientTemperatureSensitivityConstant = 0x243C,

    /// Covers object temperature sensitivity ranges 0 and 1.
    ObjectTemperatureSensitivityRangeConstants1 = 0x243D,

    /// Covers object temperature sensitivity ranges 2 and 3,
    ObjectTemperatureSensitivityRangeConstants2 = 0x243E,

    /// Covers corner temperatures 2 and 3, the corner temperature step coefficient and the
    /// object temperature sensitivity constant (K<sub>S<sub>T<sub>0</sub></sub></sub>)
    CornerTemperatures = 0x243F,

    /// The start of the individual pixel calibration data. Each word (words are 16 bits) has the
    /// pixel offset, sensitivity and ambient temperature constant, along with a flag for outlier
    /// pixels. The data is laid out in row-major order.
    PixelCalibrationStart = 0x2440,

    /// The last valid address in the MLX90640 EEPROM.
    End = 0x273F,
}

address_enum_ops!(EepromAddress);

impl TryFrom<Address> for EepromAddress {
    type Error = TryFromPrimitiveError<EepromAddress>;

    fn try_from(value: Address) -> Result<Self, Self::Error> {
        Self::try_from(value.0)
    }
}

impl From<EepromAddress> for Address {
    fn from(eeprom_address: EepromAddress) -> Self {
        let raw_address: u16 = eeprom_address.into();
        Address(raw_address)
    }
}
