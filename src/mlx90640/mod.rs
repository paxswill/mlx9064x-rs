mod address;
mod eeprom;

use crate::common::{Address, CalibrationData, MelexisCamera};
use crate::error::Error;
use crate::register::{AccessPattern, ControlRegister, Subpage};

use address::RamAddress;
use embedded_hal::blocking::i2c;

/// The height of the image captured by sensor in pixels.
pub(crate) const HEIGHT: usize = 24;

/// The width of the image captured by the sensor in pixels.
pub(crate) const WIDTH: usize = 32;

/// The total number of pixels an MLX90640 has.
pub(crate) const NUM_PIXELS: usize = HEIGHT * WIDTH;

/// The first RAM address for teh MLX90640.
const RAM_BASE_ADDRESS: u16 = 0x0400;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640 {
    calibration_data: eeprom::Mlx90640Calibration,
    config: ControlRegister,
}

impl MelexisCamera for Mlx90640 {
    fn new<I2C>(register: ControlRegister, eeprom: &[u8]) -> Result<Self, Error<I2C>>
    where
        I2C: i2c::WriteRead,
    {
        let calibration_data =
            eeprom::Mlx90640Calibration::from_data(eeprom).map_err(Error::Other)?;
        Ok(Self {
            calibration_data,
            config: register,
        })
    }

    fn pixel(&self, row: u8, column: u8, subpage: Subpage) -> Option<Address> {
        let row = row as u16;
        let column = column as u16;
        if row >= (HEIGHT as u16) || column >= (WIDTH as u16) {
            return None;
        }
        // The pixel addresses don't change depending on the subpage for the '640 (just their
        // vailidty). It's just a simple row-major indexing.
        let index = row as u16 * (WIDTH as u16) + column as u16;
        match (self.config.access_pattern, row % 2, column % 2, subpage) {
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

    fn calibration(&self) -> &dyn CalibrationData {
        &self.calibration_data
    }

    fn update_control_register(&mut self, register: ControlRegister) {
        self.config = register;
    }
}
