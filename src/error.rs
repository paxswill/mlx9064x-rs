// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
#[cfg(feature = "std")]
extern crate std;

use core::fmt;

use embedded_hal::blocking::i2c;

/// Errors that don't involve I²C.
#[derive(Clone, Debug, PartialEq)]
pub enum LibraryError {
    /// When a value from the camera is malformed in some way.
    InvalidData(&'static str),

    Other(&'static str),
}

impl fmt::Display for LibraryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LibraryError::InvalidData(msg) => write!(f, "{}", msg),
            LibraryError::Other(msg) => write!(f, "{}", msg),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for LibraryError {}

#[derive(Clone, PartialEq)]
pub enum Error<I2C>
where
    I2C: i2c::WriteRead,
{
    /// Errors originating from the I²C implementation.
    I2cError(I2C::Error),

    /// Errors originating from within this library.
    LibraryError(LibraryError),
}

// Custom Debug implementation so that I2C doesn't need to implement Debug (like the one from
// linux-embedded-hal).
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
            Error::LibraryError(err) => f.debug_tuple("Error::LibraryError").field(err).finish(),
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
            Error::I2cError(i2c_error) => write!(f, "I2C Error: {:?}", i2c_error),
            Error::LibraryError(err) => write!(f, "Library Error: {:?}", err),
        }
    }
}

#[cfg(feature = "std")]
impl<I2C> std::error::Error for Error<I2C>
where
    I2C: i2c::WriteRead,
    <I2C as i2c::WriteRead>::Error: std::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2cError(i2c_error) => Some(i2c_error),
            Error::LibraryError(lib_err) => Some(lib_err),
        }
    }
}

impl<I2C> From<LibraryError> for Error<I2C>
where
    I2C: i2c::WriteRead,
{
    fn from(lib_err: LibraryError) -> Self {
        Self::LibraryError(lib_err)
    }
}
