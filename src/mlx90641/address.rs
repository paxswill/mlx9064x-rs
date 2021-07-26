// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::convert::TryFrom;

use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

use crate::address_enum_ops;
use crate::common::Address;

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
