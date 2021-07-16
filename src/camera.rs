use embedded_hal::blocking::i2c;

use crate::common::*;
use crate::error::Error;
use crate::register::*;

/// Both the MLX90640 and MLX90641 use the same starting address for their EEPROM.
const EEPROM_BASE: [u8; 2] = 0x2400u16.to_be_bytes();

/// Both cameras (MLX90640 and MLX90641) have the same size EEPROM. 0x273F is the last address in
/// the EEPROM, so add one to that to include it, while 0x2400 is the first address. Each address
/// contains a 16-bit value, so we multiply by two to get the number of 8-bit bytes.
const EEPROM_LENGTH: usize = ((0x273F + 1) - 0x2400) * 2;

pub struct Camera<Cam, I2C, const H: usize, const W: usize> {
    /// The I²C bus this camera is accessible on.
    bus: I2C,

    /// The I²C address this camera is accessible at.
    address: u8,

    /// The camera-specific functionality.
    camera: Cam,
}

impl<Cam, I2C, const H: usize, const W: usize> Camera<Cam, I2C, H, W>
where
    Cam: MelexisCamera,
    I2C: i2c::WriteRead,
{
    /// Create a [Camera] for accessing the device at the given I²C address.
    ///
    /// MLX90964\*s can be configured to use any I²C address (except 0x00), but the default address
    /// is 0x33.
    pub fn new(bus: I2C, address: u8) -> Result<Self, Error<I2C>> {
        // We own the bus now, make it mutable.
        let mut bus = bus;
        // Grab the control register values first
        // Need to map from I2C::Error manually as it's an associated type without bounds, so we
        // can't implement From<I2C:Error>
        let control: ControlRegister = read_register(&mut bus, address)?;
        // Dump the EEPROM. Both cameras use the same size and starting offset for their EEPROM.
        let mut eeprom_buf = [0u8; EEPROM_LENGTH];
        bus.write_read(address, &EEPROM_BASE, &mut eeprom_buf)
            .map_err(Error::I2cError)?;
        let camera = Cam::new(control, &mut eeprom_buf[..])?;
        Ok(Self {
            bus: bus,
            address: address,
            camera,
        })
    }
}

fn read_register<'a, R, I2C>(bus: &mut I2C, address: u8) -> Result<R, Error<I2C>>
where
    I2C: i2c::WriteRead,
    R: Register,
{
    // Inner function to reduce the impact of monomorphization for Register. It'll still get
    // duplicated, but it should just be duplicated on I2C, and there should only be one of in an
    // application (usually).
    fn read_register<I2C: i2c::WriteRead>(
        bus: &mut I2C,
        i2c_address: u8,
        register_address: Address,
    ) -> Result<[u8; 2], I2C::Error> {
        let register_address_bytes: [u8; 2] = register_address.into();
        let mut register_bytes = [0u8; 2];
        bus.write_read(i2c_address, &register_address_bytes, &mut register_bytes)?;
        Ok(register_address_bytes)
    }

    let register_address = R::address();
    let register_value = read_register(bus, address, register_address).map_err(Error::I2cError)?;
    let register = R::from(&register_value[..]);
    Ok(register)
}
