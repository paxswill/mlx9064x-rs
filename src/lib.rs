// This is a no-good-terrible hack that adds some HTML to the generated docs, but also doesn't
// break using `cargo doc` as it only applies to this crate.
// This technique attributed to Mara Bos, as shown at
// https://github.com/m-ou-se/rust-horrible-katex-hack
#![doc(html_favicon_url = r#"">
<!-- Use KaTeX for rendering mathematical formulas -->
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.13.13/dist/katex.min.css" integrity="sha384-RZU/ijkSsFbcmivfdRBQDtwuwVqK7GMOw6IMvKyeWL2K5UAlyp6WonmB8m7Jd0Hn" crossorigin="anonymous">
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.13.13/dist/katex.min.js" integrity="sha384-pK1WpvzWVBQiP0/GjnvRxV4mOb0oxFuyRxJlk6vVw146n3egcN5C925NCP7a7BY8" crossorigin="anonymous"></script>
<!-- automatically render KaTeX -->
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.13.13/dist/contrib/auto-render.min.js" integrity="sha384-vZTG03m+2yp6N6BNi5iM4rW4oIwk5DfcNdFfxkk9ZWpDriOkXX8voJBFrAO7MpVl" crossorigin="anonymous"></script>
<script>
    document.addEventListener("DOMContentLoaded", function() {
        renderMathInElement(document.body, {
            delimiters: [
                { left: "$$", right: "$$", display: true },
                { left: "$", right: "$", display: false },
                { left: "\\begin{align*}", right: "\\end{align*}", display: true },
                { left: "\\begin{alignat*}", right: "\\end{alignat*}", display: true },
            ],
            macros: {
                "\\eeprom": "\\text{EE}[\\text{#1}]",
            },
        });
    });
</script>"#)]

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
//! One of the key differences between these cameras and other common thermal cameras is that not
//! all of the image is updated at once. The imaging area is divided into two [subpages][Subpage],
//! each being updated in turn. The pixels are split into subpages depending on the current [access
//! pattern][AccessPattern]. In chess board mode, the pixels alternate subpages in both the X and
//! Y axes, resulting in a chess or checker board-like pattern:
//! ```text
//! 0 1 0 1 0 1 0 1
//! 1 0 1 0 1 0 1 0
//! 0 1 0 1 0 1 0 1
//! 1 0 1 0 1 0 1 0
//! ```
//! The other access mode interleaves each row, so pixels will alternate subpages only on the Y
//! axis. This is also referred to as "TV" mode in the manufacturer's datasheet.
//! ```text
//! 0 0 0 0 0 0 0 0
//! 1 1 1 1 1 1 1 1
//! 0 0 0 0 0 0 0 0
//! 1 1 1 1 1 1 1 1
//! ```
//! The default mode is different between these two cameras, and the datasheet either strongly
//! advises against changing the access mode (90640), or doesn't mention the impact of changing the
//! access mode at all (90641).
//!

#![no_std]
#![feature(generic_associated_types)]

pub mod calculations;
pub mod common;
pub mod driver;
pub mod error;
pub mod mlx90640;
pub mod mlx90641;
pub mod register;
#[cfg(test)]
mod test;
mod util;

pub use common::{Address, CalibrationData};
pub use driver::CameraDriver;
pub use error::Error;
pub use register::*;

pub type Mlx90640Driver<I2C> = CameraDriver<
    mlx90640::Mlx90640,
    mlx90640::Mlx90640Calibration,
    I2C,
    { mlx90640::HEIGHT },
    { mlx90640::WIDTH },
    { mlx90640::NUM_PIXELS * 2 },
>;

pub type Mlx90641Driver<I2C> = CameraDriver<
    mlx90641::Mlx90641,
    mlx90641::Mlx90641Calibration,
    I2C,
    { mlx90641::HEIGHT },
    { mlx90641::WIDTH },
    { mlx90641::NUM_PIXELS * 2 },
>;
