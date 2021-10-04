// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
//! MLX90640 specific details.
mod address;
mod eeprom;

use core::iter;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use crate::common::{Address, MelexisCamera, PixelAddressRange};
use crate::register::{AccessPattern, Subpage};
use crate::util::Sealed;

pub use address::RamAddress;
pub use eeprom::Mlx90640Calibration;

/// MLX90640-specific constants and supporting functions.
///
/// The functionality of this type covers any MLX90640 camera module. The individual
/// camera-specific processing is performed by [`Mlx90640Calibration`].
#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640();

impl Sealed for Mlx90640 {}

impl MelexisCamera for Mlx90640 {
    type PixelRangeIterator = Mlx90640Pixels;
    type PixelsInSubpageIterator = Mlx90640PixelSubpage;

    fn pixel_ranges(subpage: Subpage, access_pattern: AccessPattern) -> Self::PixelRangeIterator {
        Mlx90640Pixels::new(subpage, access_pattern)
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

    // Implicitly documented in section 11.2.2.9 of the datasheet.
    const SELF_HEATING: f32 = 8.0;

    const HEIGHT: usize = 24;

    const WIDTH: usize = 32;

    const NUM_PIXELS: usize = Self::HEIGHT * Self::WIDTH;
}

/// An iterator of memory ranges to read from the camera.
///
/// Each I²C read transaction has an overhead of 4 bytes (1 byte for the write start message, 2
/// bytes for the address to read, 1 byte for the read start message). In chess board mode almost
/// all pixels are non-contiguous, so the most efficient method is to load all of the pixels at
/// once. In interleaved mode though it is more efficient to load each row at a time.
pub enum Mlx90640Pixels {
    #[doc(hidden)]
    Chess(iter::Once<PixelAddressRange>),
    #[doc(hidden)]
    Interleave(u16),
}

impl Mlx90640Pixels {
    fn new(subpage: Subpage, access_pattern: AccessPattern) -> Self {
        match access_pattern {
            AccessPattern::Chess => {
                let once = iter::once(PixelAddressRange {
                    start_address: RamAddress::Base.into(),
                    buffer_offset: 0,
                    // each pixel is two bytes
                    length: Mlx90640::NUM_PIXELS * 2,
                });
                Self::Chess(once)
            }
            AccessPattern::Interleave => {
                let starting_address = match subpage {
                    Subpage::Zero => 0,
                    Subpage::One => Mlx90640::WIDTH as u16,
                };
                Self::Interleave(starting_address)
            }
        }
    }
}

impl iter::Iterator for Mlx90640Pixels {
    type Item = PixelAddressRange;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Chess(inner) => inner.next(),
            Mlx90640Pixels::Interleave(current_offset) => {
                // current_offset is the offset *in camera addresses*
                if *current_offset < Mlx90640::NUM_PIXELS as u16 {
                    let next_value = PixelAddressRange {
                        start_address: (RamAddress::Base as u16 + *current_offset).into(),
                        // To convert from address offsets to buffer offsets, multiply by two
                        // (because each address refers to two bytes).
                        buffer_offset: (*current_offset * 2) as usize,
                        length: Mlx90640::WIDTH * 2,
                    };
                    *current_offset += Mlx90640::WIDTH as u16 * 2;
                    Some(next_value)
                } else {
                    None
                }
            }
        }
    }
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
        if self.index < Mlx90640::NUM_PIXELS {
            let row = self.index / Mlx90640::WIDTH;
            let column = self.index % Mlx90640::WIDTH;
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

    use crate::{AccessPattern, MelexisCamera, Resolution, Subpage};
    use crate::common::PixelAddressRange;

    use super::{Mlx90640, Mlx90640PixelSubpage, RamAddress};

    #[test]
    fn resolution_correction() {
        let resolutions = [
            (Resolution::Sixteen, 4.0),
            (Resolution::Seventeen, 2.0),
            (Resolution::Eighteen, 1.0),
            (Resolution::Nineteen, 0.5),
        ];
        for (register_resolution, expected) in resolutions {
            assert_eq!(
                Mlx90640::resolution_correction(
                    // Using 18 as the calibration value as that's the default calibration value.
                    Resolution::Eighteen as u8,
                    register_resolution as u8
                ),
                expected,
            )
        }
    }

    #[test]
    fn pixel_access_chess() {
        // In the chess access pattern, the optimal access pattern is to load all the pixels at
        // once.
        let mut pixel_iter0 = Mlx90640::pixel_ranges(Subpage::Zero, AccessPattern::Chess);
        let range0 = pixel_iter0.next();
        assert!(range0.is_some());
        assert!(
            pixel_iter0.next().is_none(),
            "Mlx90640::pixel_ranges() should only yield one range for chess mode"
        );
        let mut pixel_iter1 = Mlx90640::pixel_ranges(Subpage::One, AccessPattern::Chess);
        let range1 = pixel_iter1.next();
        assert!(range1.is_some());
        assert!(
            pixel_iter1.next().is_none(),
            "Mlx90640::pixel_ranges() should only yield one range for chess mode"
        );
        assert_eq!(
            range0, range1,
            "MLX90640 pixel ranges don't vary on subpage in chess mode"
        );
        let range0 = range0.unwrap();
        assert_eq!(range0.start_address, RamAddress::Base.into());
        assert_eq!(range0.buffer_offset, 0);
        assert_eq!(range0.length, Mlx90640::NUM_PIXELS * 2);
    }

    fn access_interleave_test(
        mut ranges: <Mlx90640 as MelexisCamera>::PixelRangeIterator,
        start_address: u16,
    ) {
        // Only checking the first four ranges, as from there the pattern should be set.
        const RANGE_LENGTH: usize = Mlx90640::WIDTH * 2;
        let buffer_start_offset = (start_address - RamAddress::Base as u16) as usize * 2;
        let first = ranges.next().unwrap();
        assert_eq!(
            first,
            PixelAddressRange {
                start_address: start_address.into(),
                buffer_offset: buffer_start_offset,
                length: RANGE_LENGTH
            }
        );
        let second = ranges.next().unwrap();
        assert_eq!(
            second,
            PixelAddressRange {
                start_address: (start_address + Mlx90640::WIDTH as u16 * 2).into(),
                buffer_offset: buffer_start_offset + RANGE_LENGTH * 2,
                length: RANGE_LENGTH
            }
        );
        let third = ranges.next().unwrap();
        assert_eq!(
            third,
            PixelAddressRange {
                start_address: (start_address + Mlx90640::WIDTH as u16 * 4).into(),
                buffer_offset: buffer_start_offset + RANGE_LENGTH * 4,
                length: RANGE_LENGTH
            }
        );
        let fourth = ranges.next().unwrap();
        assert_eq!(
            fourth,
            PixelAddressRange {
                start_address: (start_address + Mlx90640::WIDTH as u16 * 6).into(),
                buffer_offset: buffer_start_offset + RANGE_LENGTH * 6,
                length: RANGE_LENGTH
            }
        );
        // There can only be eight more rows
        assert_eq!(ranges.count(), 8);
    }

    #[test]
    fn pixel_access_interleave_subpage0() {
        access_interleave_test(
            Mlx90640::pixel_ranges(Subpage::Zero, AccessPattern::Interleave),
            RamAddress::Base.into()
        );
    }

    #[test]
    fn pixel_access_interleave_subpage1() {
        // For the first subpage, we're based on the second row
        let second_row = RamAddress::Base as u16 + Mlx90640::WIDTH as u16;
        access_interleave_test(
            Mlx90640::pixel_ranges(Subpage::One, AccessPattern::Interleave),
            second_row,
        );
    }

    #[test]
    fn pixel_subpage_interleaved() {
        let seq0 = Mlx90640PixelSubpage::new(AccessPattern::Interleave, Subpage::Zero);
        let pattern0 = repeat(true)
            .take(Mlx90640::WIDTH)
            .chain(repeat(false).take(Mlx90640::WIDTH))
            .cycle();
        seq0.zip(pattern0)
            .enumerate()
            .for_each(|(index, (seq, expected))| {
                assert_eq!(seq, expected, "{} is incorrect (pixel {})", seq, index)
            });
        let seq1 = Mlx90640PixelSubpage::new(AccessPattern::Interleave, Subpage::One);
        let pattern1 = repeat(false)
            .take(Mlx90640::WIDTH)
            .chain(repeat(true).take(Mlx90640::WIDTH))
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
        let zero_first = subpages.iter().copied().cycle().take(Mlx90640::WIDTH);
        let one_first = subpages
            .iter()
            .copied()
            .cycle()
            .skip(1)
            .take(Mlx90640::WIDTH);
        let chessboard = zero_first
            .chain(one_first)
            .cycle()
            .take(Mlx90640::NUM_PIXELS);

        let seq0 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::Zero);
        let seq1 = Mlx90640PixelSubpage::new(AccessPattern::Chess, Subpage::One);

        let all = chessboard.zip(seq0.zip(seq1));

        for (subpage, (seq0, seq1)) in all {
            assert_eq!(subpage == Subpage::Zero, seq0);
            assert_eq!(subpage == Subpage::One, seq1);
        }
    }
}
