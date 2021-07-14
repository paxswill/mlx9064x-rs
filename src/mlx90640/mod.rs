mod address;
mod eeprom;

/// The height of the image captured by sensor in pixels.
pub(crate) const HEIGHT: usize = 24;

/// The width of the image captured by the sensor in pixels.
pub(crate) const WIDTH: usize = 32;

/// The total number of pixels an MLX90640 has.
pub(crate) const NUM_PIXELS: usize = HEIGHT * WIDTH;
