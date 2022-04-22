//! A pure-Rust library for accessing the MLX90640 and MLX90641 (eventually!) thermal cameras over
//! I²C.
//!
//! These cameras have a large amount of calibration data that must be pre-processed before use,
//! and the output data also requires a somewhat complex process to turn it into temperature data.
//! This crate has two levels of API, a high-level API that handles the calibration data and raw
//! data processing for you, and a low-level API if you need to go beyond what the high-level API
//! can do for you.
//!
//! This library uses the [`embedded-hal`][embedded-hal] I²C traits, meaning you should be able to
//! use this library on other platforms, as long as there's an `embedded-hal` I²C implementation
//! available. This library is also `no_std` compatible (there is a large memory requirement
//! though).
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/*/embedded_hal/blocking/i2c/index.html
//!
//! # High-Level API
//! ```no_run
//! use std::thread::sleep;
//! use std::time::Duration;
//! use mlx9064x::Mlx90640Driver;
//! use linux_embedded_hal::I2cdev;
//!
//! let i2c_bus = I2cdev::new("/dev/i2c-1").expect("/dev/i2c-1 needs to be an I2C controller");
//! // Default address for these cameras is 0x33
//! let mut camera = Mlx90640Driver::new(i2c_bus, 0x33)?;
//! // A buffer for storing the temperature "image"
//! let mut temperatures = vec![0f32; camera.height() * camera.width()];
//! camera.generate_image_if_ready(&mut temperatures)?;
//! // Default frame rate is 2Hz
//! sleep(Duration::from_millis(500));
//! camera.generate_image_if_ready(&mut temperatures)?;
//! # Ok::<(), mlx9064x::Error<I2cdev>>(())
//! ```
//! This snippet gives a quick example of using the high-level API with an MLX90640 on Linux. The
//! camera is using the I²C bus #1 (`/dev/i2c-1`) and the default I²C address (`0x33`). The
//! calibration data is loaded from the camera over I²C and saved into an
//! [`Mlx90640Calibration`][mlx90640::Mlx90640Calibration] within `camera`. A destination buffer is
//! created to store the temperature data from the camera using a `Vec`, and then the temperature
//! data is retrieved twice to cover both [subpages](#subpages-and-access-patterns), with a delay
//! between the accesses to allow the next frame of data to become available.
//!
//! The high-level API is exposed through [`CameraDriver`], and also makes it easy to configure the
//! camera settings like frame rate or access mode. If you need to tailor the functionality beyond
//! what `CameraDriver` provides for you, the low-level API is probably a better choice for you.
//!
//! # Low-Level API
//! The low-level API is the foundation for the high-level API, exposed for those cases where a
//! more customized approach is needed. A common example is customizing how the calibration data is
//! loaded. To reduce startup time and memory usage, you might want to pre-process the calibration
//! data for a specific camera and store it in a microcontroller's flash memory. This can be done
//! by implementing [`CalibrationData`][common::CalibrationData]. Because `CameraDriver` is generic
//! over `CalibrationData`, you can use your custom `CalibrationData` with the rest of the
//! high-level API with almost no changes.
//!
//! Most users of the low-level API will probably find the [`common`], [`register`], and
//! [`calculations`] modules most relevant to their needs, with camera-model specific constants and
//! types available in the [`mlx90640`] and [`mlx90641`] modules.
//!
//! # Subpages and Access Patterns
//! A significant difference between these Melexis cameras and other common thermal cameras is how
//! the Melexis cameras update their image data. Each frame, one [subpage][Subpage] of data is
//! updated. For the MLX90640 each subpage covers half of the pixels, and the [access
//! pattern][AccessPattern] determines how the pixels are divided between the subpages. The
//! MLX90641's subpages cover *all* of the pixels (and the access pattern should *not* be changed
//! from [interleave][AccessPattern::Interleave]).

#![no_std]
#![allow(clippy::float_cmp)]

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("Either the 'std' or 'libm' feature must be enabled.");

pub mod calculations;
pub mod common;
#[doc(hidden)]
pub mod driver;
#[doc(hidden)]
pub mod error;
pub mod mlx90640;
pub mod mlx90641;
pub mod register;
mod util;

pub use common::{Address, CalibrationData, MelexisCamera};
#[doc(inline)]
pub use driver::CameraDriver;
#[doc(inline)]
pub use error::{Error, LibraryError};
pub use register::*;

/// High-level MLX90640 driver.
pub type Mlx90640Driver<I2C> = CameraDriver<
    mlx90640::Mlx90640,
    mlx90640::Mlx90640Calibration,
    I2C,
    { mlx90640::Mlx90640::HEIGHT },
    { mlx90640::Mlx90640::WIDTH },
    { mlx90640::Mlx90640::NUM_PIXELS * 2 },
>;

/// High-level MLX90641 driver.
pub type Mlx90641Driver<I2C> = CameraDriver<
    mlx90641::Mlx90641,
    mlx90641::Mlx90641Calibration,
    I2C,
    { mlx90641::Mlx90641::HEIGHT },
    { mlx90641::Mlx90641::WIDTH },
    { mlx90641::Mlx90641::NUM_PIXELS * 2 },
>;
