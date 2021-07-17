#[cfg(feature = "std")]
extern crate std;

use core::fmt;

use embedded_hal::blocking::i2c;

#[derive(Clone, PartialEq)]
pub enum Error<I2C>
where
    I2C: i2c::WriteRead,
{
    /// Errors originating from the I²C implementation.
    I2cError(I2C::Error),

    /// When a value from the camera is malformed in some way.
    InvalidData(&'static str),

    Other(&'static str),
}

impl<I2C> fmt::Debug for Error<I2C>
where
    I2C: i2c::WriteRead,
    <I2C as i2c::WriteRead>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2cError(i2c_error) => {
                f.debug_tuple("Error::I2cError").field(i2c_error).finish()
            }
            Error::InvalidData(error_msg) => f
                .debug_tuple("Error::InvalidData")
                .field(error_msg)
                .finish(),
            Error::Other(error_msg) => f.debug_tuple("Error::Other").field(error_msg).finish(),
        }
    }
}

impl<I2C> fmt::Display for Error<I2C>
where
    I2C: i2c::WriteRead,
    <I2C as i2c::WriteRead>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2cError(i2c_error) => {
                write!(f, "I2C Error: {:?}", i2c_error)
            }
            Error::InvalidData(error_msg) => write!(f, "Invalid data from camera: {}", error_msg),
            Error::Other(error_msg) => write!(f, "{}", error_msg),
        }
    }
}

#[cfg(feature = "std")]
impl<I2C> std::error::Error for Error<I2C>
where
    I2C: i2c::WriteRead + fmt::Debug,
    <I2C as i2c::WriteRead>::Error: std::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2cError(i2c_error) => Some(i2c_error),
            _ => None,
        }
    }
}
