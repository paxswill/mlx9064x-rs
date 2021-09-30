// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod address;
mod eeprom;
pub mod hamming;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use core::cmp::Ordering;
use core::iter;

use crate::common::{Address, MelexisCamera};
use crate::register::{AccessPattern, Subpage};
use crate::util::{self, Sealed};

pub use address::RamAddress;
pub use eeprom::Mlx90641Calibration;

/// Iterator of valid pixel ranges for the MLX90641.
///
/// The MLX90641 updates the entire frame at a time, but interleaves the data in memory. This
/// means the access pattern doesn't really matter, the subpage only changes where to read
/// from, and every pixel is written to.
/// The datasheet documents the interleaved access mode, and in that mode pixels alternate
/// subpages every 32 pixels, but each pixel is present in both subpages. In other words,
/// starting at 0x0400, there are pixels 0 through 31 for subpage 0. Then there are pixels 0
/// through 31 for subpage 1. Then pixels 32-63 for subpage 0, and so on.
type SubpageInterleave = util::SubpageInterleave<
    // There are two rows of the image per stride
    { (<Mlx90641 as MelexisCamera>::WIDTH * 2) as u16 },
    { (<Mlx90641 as MelexisCamera>::HEIGHT / 2) as u16 },
    { RamAddress::Base as u16 },
>;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90641();

impl Sealed for Mlx90641 {}

impl MelexisCamera for Mlx90641 {
    type PixelRangeIterator = SubpageInterleave;
    type PixelsInSubpageIterator = AllPixels<{ Self::NUM_PIXELS }>;

    fn pixel_ranges(subpage: Subpage, _access_pattern: AccessPattern) -> Self::PixelRangeIterator {
        // The 90641 updates an entire frame at a time and only vary the data location on subpage
        SubpageInterleave::new(subpage)
    }

    fn pixels_in_subpage(
        _subpage: Subpage,
        _access_pattern: AccessPattern,
    ) -> Self::PixelsInSubpageIterator {
        AllPixels::default()
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

#[derive(Clone, Copy, Debug, Default)]
pub struct AllPixels<const COUNT: usize> {
    current: usize,
}

impl<const COUNT: usize> iter::Iterator for AllPixels<COUNT> {
    type Item = bool;

    fn next(&mut self) -> Option<Self::Item> {
        match self.current.cmp(&COUNT) {
            Ordering::Less => {
                self.current += 1;
                Some(true)
            }
            _ => None,
        }
    }
}
