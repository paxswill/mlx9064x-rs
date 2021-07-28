// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod address;
mod hamming;

pub use address::RamAddress;
pub use hamming::{add_checksum, validate_checksum};

/// The height of the image captured by sensor in pixels.
pub(crate) const HEIGHT: usize = 12;

/// The width of the image captured by the sensor in pixels.
pub(crate) const WIDTH: usize = 16;

/// The total number of pixels an MLX90640 has.
pub(crate) const NUM_PIXELS: usize = HEIGHT * WIDTH;

