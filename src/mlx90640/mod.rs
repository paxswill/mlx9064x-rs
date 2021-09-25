// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod address;
mod eeprom;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use crate::common::{Address, MelexisCamera, PixelAddressRange};
use crate::register::{AccessPattern, Subpage};

pub use address::RamAddress;
pub use eeprom::Mlx90640Calibration;

/// The height of the image captured by sensor in pixels.
pub const HEIGHT: usize = 24;

/// The width of the image captured by the sensor in pixels.
pub const WIDTH: usize = 32;

/// The total number of pixels an MLX90640 has.
pub const NUM_PIXELS: usize = HEIGHT * WIDTH;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640();

impl MelexisCamera for Mlx90640 {
    type PixelRangeIterator = core::array::IntoIter<PixelAddressRange, 1>;
    type PixelsInSubpageIterator = Mlx90640PixelSubpage;

    fn pixel_ranges(_subpage: Subpage, _access_pattern: AccessPattern) -> Self::PixelRangeIterator {
        // For the MLX90640, the best strategy (no matter the access mode) is to just load
        // everything.
        core::array::IntoIter::new([PixelAddressRange {
            start_address: RamAddress::Base.into(),
            buffer_offset: 0,
            // Each pixel is two bytes
            length: NUM_PIXELS * 2,
        }])
    }

    fn pixels_in_subpage(
        subpage: Subpage,
        access_pattern: AccessPattern,
    ) -> Self::PixelsInSubpageIterator {
        Mlx90640PixelSubpage::new(access_pattern, subpage)
    }

    //const T_A_V_BE: Address = RamAddress::AmbientTemperatureVoltageBe.into();
    const T_A_V_BE: Address = Address::new(RamAddress::AmbientTemperatureVoltageBe as u16);

    const T_A_PTAT: Address = Address::new(RamAddress::AmbientTemperatureVoltage as u16);

    fn compensation_pixel(subpage: Subpage) -> Address {
        match subpage {
            Subpage::Zero => RamAddress::CompensationPixelZero.into(),
            Subpage::One => RamAddress::CompensationPixelOne.into(),
        }
    }

    const GAIN: Address = Address::new(RamAddress::Gain as u16);

    const V_DD_PIXEL: Address = Address::new(RamAddress::PixelSupplyVoltage as u16);

    fn resolution_correction(calibrated_resolution: u8, current_resolution: u8) -> f32 {
        // These values are safe to convert to i8, as they were originally 4-bit unsigned ints.
        let resolution_exp: i8 = calibrated_resolution as i8 - current_resolution as i8;
        // Have to use an f32 here as resolution_exp may be negative.
        f32::from(resolution_exp).exp2()
    }

    // It's defined as 1 in the datasheet(well, 2, but 1-indexed, so 1 when 0-indexed).
    const BASIC_TEMPERATURE_RANGE: usize = 1;
}

/// An iterator for determining which pixels are part of the current subpage.
///
/// This type is an implementation detail, and should not be relied upon by consumers of this
/// crate.
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
            let row = self.index / WIDTH;
            let column = self.index % WIDTH;
            self.index += 1;
            Some(match self.access_pattern {
                AccessPattern::Chess => {
                    if self.subpage_num == 0 {
                        row % 2 == column % 2
                    } else {
                        row % 2 != column % 2
                    }
                }
                AccessPattern::Interleave => row % 2 == self.subpage_num,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use core::iter::repeat;

    use crate::{AccessPattern, Subpage};

    use super::{Mlx90640PixelSubpage, NUM_PIXELS, WIDTH};

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
        let subpages = [Subpage::Zero, Subpage::One];
        let zero_first = subpages.iter().copied().cycle().take(WIDTH);
        let one_first = subpages.iter().copied().cycle().skip(1).take(WIDTH);
        let chessboard = zero_first.chain(one_first).cycle().take(NUM_PIXELS);

        let seq0 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::Zero);
        let seq1 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::One);

        let all = chessboard.zip(seq0.zip(seq1));

        for (subpage, (seq0, seq1)) in all {
            assert_eq!(subpage == Subpage::Zero, seq0);
            assert_eq!(subpage == Subpage::One, seq1);
        }
    }
}
