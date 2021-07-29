// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::convert::TryFrom;

use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

use crate::address_enum_ops;
use crate::common::Address;

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub(super) enum EepromAddress {
    /// The start of the EEPROM address range
    ///
    /// For calibration purposes, the first 16 words (word size is 16 bits) can be ignored.
    Base = 0x2400,

    /// The bits 5 through 10 are the offset compensation scaling factor.
    OffsetCompensationScale = 0x2410,

    /// The pixel offset average is split across two words.
    OffsetCompensationAverageStart = 0x2411,

    K_TaAverage = 0x2415,

    K_TaScale = 0x2416,

    K_VAverage = 0x2417,

    K_VScale = 0x2418,

    PixelSensitivityScaleStart = 0x2419,

    PixelSensitivityMaxStart = 0x241C,

    KsTa = 0x2422,

    Emissivity = 0x2423,

    GainStart = 0x2424,

    Vdd25 = 0x2426,

    K_Vdd = 0x2427,

    PtatStart = 0x2428,

    KtPtat = 0x242A,

    KvPtat = 0x242B,

    AlphaPtat = 0x242C,

    CompensationPixelSensitivity = 0x242D,

    CompensationPixelSensitivityScale = 0x242E,

    CompensationPixelOffsetStart = 0x242F,

    CompensationPixelKt = 0x2431,

    CompensationPixelKv = 0x243,

    ThermalGradientCompensation = 0x2433,

    KsToScale = 0x2434,

    KsTo0 = 0x2435,

    KsTo1 = 0x2436,

    KsTo2 = 0x2437,

    KsTo3 = 0x2438,

    KsTo4 = 0x2439,

    CornerTemperature5 = 0x243A,

    KsTo5 = 0x243B,

    CornerTemperature6 = 0x243C,

    KsTo6 = 0x243D,

    CornerTemperature7 = 0x243E,

    KsTo7 = 0x243F,

    PixelOffsetSubpage0Start = 0x2440,

    PixelSensitivityStart = 0x2500,

    PixelConstantsStart = 0x25C0,

    PixelOffsetSubpage1Start = 0x2680,

    /// The last valid address in the MLX90641 EEPROM.
    End = 0x273F,
}

address_enum_ops!(EepromAddress);

impl TryFrom<Address> for EepromAddress {
    type Error = TryFromPrimitiveError<EepromAddress>;

    fn try_from(value: Address) -> Result<Self, Self::Error> {
        let raw_address: u16 = value.into();
        Self::try_from(raw_address)
    }
}

impl From<EepromAddress> for Address {
    fn from(eeprom_address: EepromAddress) -> Self {
        let raw_address: u16 = eeprom_address.into();
        raw_address.into()
    }
}

/// Significant RAM addresses for the MLX909641.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
pub enum RamAddress {
    /// The start of RAM in the MLX90641 address space. This is also the start of the pixel
    /// addresses, which are laid out in row-major order.
    Base = 0x0400,

    /// Labelled V<sub>BE</sub> and Ta<sub>V<sub>BE</sub></sub> in the datasheet.
    AmbientTemperatureVoltageBe = 0x0580,

    /// The compensation pixel for subpage 0.
    CompensationPixelZero = 0x0588,

    /// The current (in the temporal sense, not electrical) gain.
    Gain = 0x058A,

    /// Ambient temperature voltage, labelled T<sub>a<sub>PTAT</sub></sub> in the datasheet.
    AmbientTemperatureVoltage = 0x05A0,

    /// The compensation pixel for subpage 1.
    CompensationPixelOne = 0x05A8,

    /// The pixel supply voltage, labelled V<sub>DD<sub>pix</sub></sub> in the datasheet.
    PixelSupplyVoltage = 0x05AA,

    /// The last valid RAM address for the MLX90641.
    End = 0x05BF,
}

address_enum_ops!(RamAddress);

impl TryFrom<Address> for RamAddress {
    type Error = TryFromPrimitiveError<RamAddress>;

    fn try_from(value: Address) -> Result<Self, Self::Error> {
        let raw_address: u16 = value.into();
        Self::try_from(raw_address)
    }
}

impl From<RamAddress> for Address {
    fn from(ram_address: RamAddress) -> Self {
        let raw_address: u16 = ram_address.into();
        raw_address.into()
    }
}
