//! A pure-Rust library for accessing the MLX90640 and MLX90641 (eventually!) thermal cameras over
//! I²C.
//!
//! These cameras are not the simplest to use, with a significant amount of slow floating point
//! calculations that need to be performed for each frame. The advantages these cameras have over
//! others in this price range are a much higher resolution, faster refresh rate, and (depending on
//! the specific camera) wider view angles.
//!
//! # Usage
//! ```no_run
//! use std::thread::sleep;
//! use std::time::Duration;
//! use mlx9064x::Mlx90640Camera;
//! use linux_embedded_hal::I2cdev;
//!
//! let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
//! // Default address for these cameras is 0x33
//! let mut camera = Mlx90640Camera::new(i2c_bus, 0x33)?;
//! // A buffer for storing the temperature "image"
//! let mut temperatures = vec![0f32; camera.height() * camera.width()];
//! camera.generate_image_if_ready(&mut temperatures)?;
//! // Default frame rate is 2Hz
//! sleep(Duration::from_millis(500));
//! camera.generate_image_if_ready(&mut temperatures)?;
//! # Ok::<(), mlx9064x::Error<I2cdev>>(())
//! ```
//! This library uses the [`embedded-hal`] I²C traits, meaning you should be able to use this
//! library on other platforms, as long as there's an `embedded-hal` I²C implementation available.
//! This library is also `no_std` compatible (there is a large memory requirement though).

#![no_std]

mod camera;
mod common;
mod error;
pub mod mlx90640;
pub mod mlx90641;
mod register;
mod util;

pub use camera::Camera;
pub use common::{Address, CalibrationData, MelexisCamera};
pub use error::Error;
pub use register::*;

pub type Mlx90640Camera<I2C> = Camera<
    mlx90640::Mlx90640,
    I2C,
    { mlx90640::HEIGHT },
    { mlx90640::WIDTH },
    { mlx90640::NUM_PIXELS * 2 },
>;
