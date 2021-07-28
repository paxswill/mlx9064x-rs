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

    /// Other error messages.
    Other(&'static str),

    /// Failures when decoding a checksum.
    ///
    /// The MLX90641 uses a checksum with its [EEPROM][crate::mlx90641::eeprom].
    Checksum(u16),
}

impl fmt::Display for LibraryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LibraryError::InvalidData(msg) => write!(f, "{}", msg),
            LibraryError::Other(msg) => write!(f, "{}", msg),
            LibraryError::Checksum(invalid_word) => {
                write!(f, "Invalid checksum for data {:#06X}", invalid_word)
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for LibraryError {}

pub enum Error<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
{
    /// Errors originating from the I²C implementation.
    I2cWriteReadError(<I2C as i2c::WriteRead>::Error),

    I2cWriteError(<I2C as i2c::Write>::Error),

    /// Errors originating from within this library.
    LibraryError(LibraryError),
}

// Custom Debug implementation so that I2C doesn't need to implement Debug (like the one from
// linux-embedded-hal).
impl<I2C> fmt::Debug for Error<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
    <I2C as i2c::WriteRead>::Error: fmt::Debug,
    <I2C as i2c::Write>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2cWriteReadError(i2c_error) => f
                .debug_tuple("Error::I2cWriteReadError")
                .field(i2c_error)
                .finish(),
            Error::I2cWriteError(i2c_error) => f
                .debug_tuple("Error::I2cWriteError")
                .field(i2c_error)
                .finish(),
            Error::LibraryError(err) => f.debug_tuple("Error::LibraryError").field(err).finish(),
        }
    }
}

impl<I2C> fmt::Display for Error<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
    <I2C as i2c::WriteRead>::Error: fmt::Debug,
    <I2C as i2c::Write>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2cWriteReadError(i2c_error) => write!(f, "I2C Error: {:?}", i2c_error),
            Error::I2cWriteError(i2c_error) => write!(f, "I2C Error: {:?}", i2c_error),
            Error::LibraryError(err) => write!(f, "Library Error: {:?}", err),
        }
    }
}

#[cfg(feature = "std")]
impl<I2C> std::error::Error for Error<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
    <I2C as i2c::WriteRead>::Error: std::error::Error + 'static,
    <I2C as i2c::Write>::Error: std::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2cWriteReadError(err) => Some(err),
            Error::I2cWriteError(err) => Some(err),
            Error::LibraryError(lib_err) => Some(lib_err),
        }
    }
}

impl<I2C> From<LibraryError> for Error<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
{
    fn from(lib_err: LibraryError) -> Self {
        Self::LibraryError(lib_err)
    }
}
