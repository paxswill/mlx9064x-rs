mod address;
mod eeprom;

// When testing, open up access to the EEPROM data test fixture
#[cfg(test)]
pub(crate) use eeprom::test::eeprom_data;

use crate::common::{Address, CalibrationData, MelexisCamera, PixelAddressRange};
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

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640 {
    calibration_data: eeprom::Mlx90640Calibration,
    config: ControlRegister,
}

impl MelexisCamera for Mlx90640 {
    type PixelRangeIterator = core::array::IntoIter<PixelAddressRange, 1>;
    type PixelsInSubpageIterator = Mlx90640PixelSubpage;

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

    fn pixel_ranges(&self, _subpage: Subpage) -> Self::PixelRangeIterator {
        // For the MLX90640, the best strategy (no matter the access mode) is to just load
        // everything.
        core::array::IntoIter::new([PixelAddressRange {
            start_address: RamAddress::Base.into(),
            length: NUM_PIXELS,
        }])
    }

    fn pixels_in_subpage(&self, subpage: Subpage) -> Self::PixelsInSubpageIterator {
        Mlx90640PixelSubpage::new(self.config.access_pattern, subpage)
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mlx90640PixelSubpage {
    /// The count of the current pixel.
    index: usize,

    /// The access pattern being used.
    access_pattern: AccessPattern,

    /// The subpage (as a number) this sequence is being generated for.
    subpage_num: usize,
}

impl Mlx90640PixelSubpage {
    fn new(access_pattern: AccessPattern, subpage: Subpage) -> Self {
        Self {
            index: 0,
            access_pattern,
            subpage_num: subpage as usize,
        }
    }
}

impl Iterator for Mlx90640PixelSubpage {
    type Item = bool;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < NUM_PIXELS {
            let pixel_subpage = match self.access_pattern {
                AccessPattern::Chess => self.index % 2,
                AccessPattern::Interleave => (self.index / WIDTH) % 2,
            };
            self.index += 1;
            Some(pixel_subpage == self.subpage_num)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use core::iter::repeat;

    use crate::{AccessPattern, Subpage};

    use super::{Mlx90640PixelSubpage, WIDTH};

    #[test]
    fn pixel_subpage_interleaved() {
        let seq0 = Mlx90640PixelSubpage::new(AccessPattern::Interleave, Subpage::Zero);
        let pattern0 = repeat(true)
            .take(WIDTH)
            .chain(repeat(false).take(WIDTH))
            .cycle();
        seq0.zip(pattern0)
            .enumerate()
            .for_each(|(index, (seq, expected))| {
                assert_eq!(seq, expected, "{} is incorrect (pixel {})", seq, index)
            });
        let seq1 = Mlx90640PixelSubpage::new(AccessPattern::Interleave, Subpage::One);
        let pattern1 = repeat(false)
            .take(WIDTH)
            .chain(repeat(true).take(WIDTH))
            .cycle();
        seq1.zip(pattern1)
            .enumerate()
            .for_each(|(index, (seq, expected))| {
                assert_eq!(seq, expected, "{} is incorrect (pixel {})", seq, index)
            });
    }

    #[test]
    fn pixel_subpage_chess() {
        let seq0 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::Zero);
        let pattern0 = [true, false];
        seq0.zip(pattern0.iter().cycle())
            .enumerate()
            .for_each(|(index, (seq, expected))| {
                assert_eq!(seq, *expected, "{} is incorrect (pixel {})", seq, index)
            });
        let seq1 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::One);
        let pattern1 = [false, true];
        seq1.zip(pattern1.iter().cycle())
            .enumerate()
            .for_each(|(index, (seq, expected))| {
                assert_eq!(seq, *expected, "{} is incorrect (pixel {})", seq, index)
            });
    }
}
