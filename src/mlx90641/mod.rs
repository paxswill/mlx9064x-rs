// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod address;
mod eeprom;
mod hamming;

use core::cmp::Ordering;
use core::iter;

use crate::common::{Address, MelexisCamera, PixelAddressRange};
use crate::register::{AccessPattern, Subpage};

pub use address::RamAddress;
pub use eeprom::Mlx90641Calibration;
pub use hamming::{add_checksum, validate_checksum};

/// The height of the image captured by sensor in pixels.
pub(crate) const HEIGHT: usize = 12;

/// The width of the image captured by the sensor in pixels.
pub(crate) const WIDTH: usize = 16;

/// The total number of pixels an MLX90640 has.
pub(crate) const NUM_PIXELS: usize = HEIGHT * WIDTH;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90641();

impl MelexisCamera for Mlx90641 {
    type PixelRangeIterator = SubpageInterleave;
    type PixelsInSubpageIterator = AllPixels<NUM_PIXELS>;

    fn pixel_ranges(subpage: Subpage, access_pattern: AccessPattern) -> Self::PixelRangeIterator {
        match access_pattern {
            AccessPattern::Chess => panic!("The chess pattern is not documented for the MLX90641"),
            AccessPattern::Interleave => SubpageInterleave::new(subpage),
        }
    }

    fn pixels_in_subpage(
        _subpage: Subpage,
        _access_pattern: AccessPattern,
    ) -> Self::PixelsInSubpageIterator {
        AllPixels::default()
    }

    fn t_a_v_be() -> Address {
        RamAddress::AmbientTemperatureVoltageBe.into()
    }

    fn t_a_ptat() -> Address {
        RamAddress::AmbientTemperatureVoltage.into()
    }

    fn compensation_pixel(subpage: Subpage) -> Address {
        match subpage {
            Subpage::Zero => RamAddress::CompensationPixelZero,
            Subpage::One => RamAddress::CompensationPixelOne,
        }
        .into()
    }

    fn gain() -> Address {
        RamAddress::Gain.into()
    }

    fn v_dd_pixel() -> Address {
        RamAddress::PixelSupplyVoltage.into()
    }

    fn resolution_correction(calibrated_resolution: u8, current_resolution: u8) -> f32 {
        // These values are safe to convert to i8, as they were originally 4-bit unsigned ints.
        let resolution_exp: i8 = calibrated_resolution as i8 - current_resolution as i8;
        // Have to use an f32 here as resolution_exp may be negative.
        f32::from(resolution_exp).exp2()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SubpageInterleave {
    current_address: u16,
}

impl SubpageInterleave {
    /// Length of each section of pixels with a common subpage
    ///
    /// The datasheet only documents interleaved access mode, and in that mode pixels alternate
    /// subpages every 32 pixels, but each pixel is present in both subpages. In other words,
    /// starting at 0x0400, there are pixels 0 through 31 for subpage 0. Then there are pixels 0
    /// through 31 for subpage 1. Then pixels 32-63 for subpage 0, and so on.
    // Multiply by two to get the number of bytes.
    const STRIDE_LENGTH: u16 = 32 * 2;

    /// The end of the range of valid pixels.
    const PIXEL_END_ADDRESS: u16 = 0x0580;

    fn new(subpage: Subpage) -> Self {
        let starting_address: u16 = match subpage {
            Subpage::Zero => RamAddress::Base as u16,
            Subpage::One => RamAddress::Base as u16 + Self::STRIDE_LENGTH,
        };
        Self {
            current_address: starting_address,
        }
    }
}

impl iter::Iterator for SubpageInterleave {
    type Item = PixelAddressRange;

    fn next(&mut self) -> Option<Self::Item> {
        match self.current_address.cmp(&Self::PIXEL_END_ADDRESS) {
            Ordering::Less => {
                let next_value = PixelAddressRange {
                    start_address: self.current_address.into(),
                    length: Self::STRIDE_LENGTH as usize,
                };
                // Skip forward *two* Strides, as the stride immediately after this one is for the
                // next subpage.
                self.current_address += Self::STRIDE_LENGTH * 2;
                Some(next_value)
            }
            _ => None,
        }
    }
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
