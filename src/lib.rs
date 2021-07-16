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

pub type Mlx90640<I2C> = Camera<mlx90640::Mlx90640, I2C, { mlx90640::HEIGHT }, { mlx90640::WIDTH }>;
