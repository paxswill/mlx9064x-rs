// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
//! MLX90641 specific details.
mod address;
mod eeprom;
pub mod hamming;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use core::cmp::Ordering;
use core::iter;

use crate::common::{AccessPatternFilter, Address, MelexisCamera, PixelAddressRange};
use crate::register::{AccessPattern, Subpage};
use crate::util::Sealed;

pub use address::RamAddress;
pub use eeprom::Mlx90641Calibration;

/// MLX90641-specific constants and supporting functions.
///
/// The functionality of this type covers any MLX90641 camera module. The individual
/// camera-specific processing is performed by [`Mlx90641Calibration`].
#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90641();

impl Sealed for Mlx90641 {}

impl MelexisCamera for Mlx90641 {
    type PixelRangeIterator = SubpageInterleave;

    fn pixel_ranges(subpage: Subpage, _access_pattern: AccessPattern) -> Self::PixelRangeIterator {
        // The 90641 updates an entire frame at a time and only vary the data location on subpage
        SubpageInterleave::new(subpage)
    }

    fn filter_by_subpage<I: IntoIterator>(
        iter: I,
        _subpage: Subpage,
        _access_pattern: AccessPattern,
    ) -> AccessPatternFilter<I::IntoIter> {
        AccessPatternFilter::unfiltered(iter)
    }

    const T_A_V_BE: Address = Address::new(RamAddress::AmbientTemperatureVoltageBe as u16);

    const T_A_PTAT: Address = Address::new(RamAddress::AmbientTemperatureVoltage as u16);

    fn compensation_pixel(subpage: Subpage) -> Address {
        match subpage {
            Subpage::Zero => RamAddress::CompensationPixelZero,
            Subpage::One => RamAddress::CompensationPixelOne,
        }
        .into()
    }

    const GAIN: Address = Address::new(RamAddress::Gain as u16);

    const V_DD_PIXEL: Address = Address::new(RamAddress::PixelSupplyVoltage as u16);

    fn resolution_correction(calibrated_resolution: u8, current_resolution: u8) -> f32 {
        // These values are safe to convert to i8, as they were originally 4-bit unsigned ints.
        let resolution_exp: i8 = calibrated_resolution as i8 - current_resolution as i8;
        // Have to use an f32 here as resolution_exp may be negative.
        f32::from(resolution_exp).exp2()
    }

    // It's defined as 2 in the datasheet(well, 3, but 1-indexed, so 2 when 0-indexed).
    const BASIC_TEMPERATURE_RANGE: usize = 2;

    // Implicitly documented in section 11.2.2.9 of the datasheet.
    const SELF_HEATING: f32 = 5.0;

    const HEIGHT: usize = 12;

    const WIDTH: usize = 16;

    const NUM_PIXELS: usize = Self::HEIGHT * Self::WIDTH;
}

#[derive(Clone, Copy, Debug)]
pub struct SubpageInterleave {
    stride_count: u16,
    base_address: u16,
}

impl SubpageInterleave {
    /// Length of each section of pixels with a common subpage
    ///
    /// The MLX90641 updates the entire frame at a time, but interleaves the data in memory. This
    /// means the access pattern doesn't really matter, the subpage only changes where to read
    /// from, but every pixel is written to.
    /// The datasheet documents the interleaved access mode, and in that mode pixels alternate
    /// subpages every 32 pixels, but each pixel is present in both subpages. In other words,
    /// starting at 0x0400, there are pixels 0 through 31 for subpage 0. Then there are pixels 0
    /// through 31 for subpage 1. Then pixels 32-63 for subpage 0, and so on.
    // Multiply by two to get the number of bytes.
    const STRIDE_LENGTH: u16 = 32 * 2;

    /// The beginning of the range of valid pixels.
    const PIXEL_START_ADDRESS: u16 = RamAddress::Base as u16;

    /// The number of strides in each frame.
    const NUM_STRIDES: u16 = (Mlx90641::HEIGHT / 2) as u16;

    fn new(subpage: Subpage) -> Self {
        let starting_address: u16 = match subpage {
            Subpage::Zero => Self::PIXEL_START_ADDRESS,
            // We need to divide by two to get the *address* offset
            Subpage::One => Self::PIXEL_START_ADDRESS + (Self::STRIDE_LENGTH / 2),
        };
        Self {
            stride_count: 0,
            base_address: starting_address,
        }
    }
}

impl iter::Iterator for SubpageInterleave {
    type Item = PixelAddressRange;

    fn next(&mut self) -> Option<Self::Item> {
        // There are two frame rows per stride
        match self.stride_count.cmp(&Self::NUM_STRIDES) {
            Ordering::Less => {
                let next_value = PixelAddressRange {
                    start_address: (self.base_address + self.stride_count * Self::STRIDE_LENGTH)
                        .into(),
                    buffer_offset: (self.stride_count * Self::STRIDE_LENGTH) as usize,
                    length: Self::STRIDE_LENGTH as usize,
                };
                self.stride_count += 1;
                Some(next_value)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{AccessPattern, MelexisCamera, Resolution, Subpage};

    use super::Mlx90641;

    #[test]
    fn filter_by_subpage() {
        let mut count = 0;
        // Only testing interleave as chess mode isn't used with the MLX90641
        let sub0 = Mlx90641::filter_by_subpage(
            0..Mlx90641::NUM_PIXELS,
            Subpage::Zero,
            AccessPattern::Interleave,
        );
        let sub1 = Mlx90641::filter_by_subpage(
            0..Mlx90641::NUM_PIXELS,
            Subpage::One,
            AccessPattern::Interleave,
        );
        for (expected, actual) in sub0.zip(sub1).enumerate() {
            assert_eq!(
                actual.0, actual.1,
                "MLX90641 doesn't vary pixels on subpages"
            );
            assert_eq!(expected, actual.0, "Every pixel is valid for MLX90641");
            count += 1;
        }
        assert_eq!(
            count,
            Mlx90641::NUM_PIXELS,
            "Ever pixels needs a value for pixels_in_subpage()"
        );
    }

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
                Mlx90641::resolution_correction(
                    // Using 18 as the calibration value as that's the default calibration value.
                    Resolution::Eighteen as u8,
                    register_resolution as u8
                ),
                expected,
            )
        }
    }
}
