use crate::common::*;
use crate::register::{AccessPattern, Subpage};

use super::address::RamAddress;

const RAM_BASE_ADDRESS: u16 = 0x0400;

const WIDTH: u16 = super::WIDTH as u16;
const HEIGHT: u16 = super::HEIGHT as u16;

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct Mlx90640Ram(AccessPattern);

impl MelexisRamAddress for Mlx90640Ram {
    fn pixel(&self, row: u8, column: u8, subpage: Subpage) -> Option<Address> {
        let row = row as u16;
        let column = column as u16;
        if row >= HEIGHT || column >= WIDTH {
            return None;
        }
        // The pixel addresses don't change depending on the subpage for the '640 (just their
        // vailidty). It's just a simple row-major indexing.
        let index = row as u16 * WIDTH + column as u16;
        match (self.0, row % 2, column % 2, subpage) {
            (AccessPattern::Chess, 0, 0, Subpage::Zero) => Some(Address(RAM_BASE_ADDRESS + index)),
            (AccessPattern::Chess, 1, 1, Subpage::One) => Some(Address(RAM_BASE_ADDRESS + index)),
            (AccessPattern::Interleave, 0, _, Subpage::Zero) => {
                Some(Address(RAM_BASE_ADDRESS + index))
            }
            (AccessPattern::Interleave, 1, _, Subpage::One) => {
                Some(Address(RAM_BASE_ADDRESS + index))
            }
            _ => None,
        }
    }

    fn t_a_v_be(&self) -> Address {
        RamAddress::AmbientTemperatureVoltageBe.into()
    }

    fn t_a_ptat(&self) -> Address {
        RamAddress::AmbientTemperatureVoltage.into()
    }

    fn compensation_pixel(&self, subpage: Subpage) -> Address {
        match subpage {
            Subpage::Zero => RamAddress::CompensationPixelZero.into(),
            Subpage::One => RamAddress::CompensationPixelOne.into(),
        }
    }

    fn gain(&self) -> Address {
        RamAddress::Gain.into()
    }

    fn v_dd_pixel(&self) -> Address {
        RamAddress::PixelSupplyVoltage.into()
    }
}
