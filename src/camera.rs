use embedded_hal::blocking::i2c;
use paste::paste;

use crate::common::*;
use crate::error::Error;
use crate::register::*;

/// Both the MLX90640 and MLX90641 use the same starting address for their EEPROM.
const EEPROM_BASE: [u8; 2] = 0x2400u16.to_be_bytes();

/// Both cameras (MLX90640 and MLX90641) have the same size EEPROM. 0x273F is the last address in
/// the EEPROM, so add one to that to include it, while 0x2400 is the first address. Each address
/// contains a 16-bit value, so we multiply by two to get the number of 8-bit bytes.
const EEPROM_LENGTH: usize = ((0x273F + 1) - 0x2400) * 2;

macro_rules! set_register_field {
    { $register_access:ident, $field:ident, $doc:literal } => {
        set_register_field! {
            $register_access,
            $field,
            bool,
            $doc
        }
    };
    { $register_access:ident, $field:ident, $typ:ty, $doc:literal } => {
    paste! {
        #[doc = $doc]
        pub fn [< set_ $field >](&mut self, new_value: $typ) -> Result<(), Error<I2C>> {
            let mut current = self.$register_access()?;
            if current.$field != new_value {
                current.$field = new_value;
                self.[< set_ $register_access >](current)
            } else {
                Ok(())
            }
        }
    }};
}

/// The shared camera driver for the MLX90640 and MLX90641 thermopiles.
///
/// These cameras offer higher resolutions and faster refresh rates than other common low-cost
/// thermal cameras, but they come with some differences in operation, as well as much more
/// processing required to end up with a grid of temperatures.
///
/// # Subpages and Access Patterns
/// One of the biggest differences you may encounter with these cameras is that not all of the
/// image is updated at once. The imaging area is divided into two [subpages][Subpage], each being
/// updated in turn. The pixels are split into subpages depending on the current [access
/// pattern][AccessPattern]. In chess board mode, the pixels alternate subpages in both the X and
/// Y axes, resulting in a chess or checker board-like pattern:
/// ```
/// 0 1 0 1 0 1 0 1
/// 1 0 1 0 1 0 1 0
/// 0 1 0 1 0 1 0 1
/// 1 0 1 0 1 0 1 0
/// ```
/// The other access mode interleaves each row, so pixels will alternate subpages only on the Y
/// axis. This is also referred to as "TV" mode in the manufacturer's datasheet.
/// ```
/// 0 0 0 0 0 0 0 0
/// 1 1 1 1 1 1 1 1
/// 0 0 0 0 0 0 0 0
/// 1 1 1 1 1 1 1 1
/// ```
/// The default mode is different between these two cameras, and the datasheet either strongly
/// advises against changing the access mode (90640), or doesn't mention the impact of changing the
/// access mode at all (90641).
///
/// The biggest impact to the user of these modules is that you will need to call one of the
/// `generate_image_*` functions for both subpages to get a full image.
#[derive(Clone, Debug)]
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
    /// Create a `Camera` for accessing the device at the given I²C address.
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
            bus,
            address,
            camera,
        })
    }

    fn status_register(&mut self) -> Result<StatusRegister, Error<I2C>> {
        let register = read_register(&mut self.bus, self.address)?;
        Ok(register)
    }

    fn set_status_register(&mut self, register: StatusRegister) -> Result<(), Error<I2C>> {
        write_register(&mut self.bus, self.address, register)
    }

    fn control_register(&mut self) -> Result<ControlRegister, Error<I2C>> {
        let register = read_register(&mut self.bus, self.address)?;
        self.camera.update_control_register(register);
        // Update the resolution as well
        self.resolution_correction = self.camera.resolution_correction();
        Ok(register)
    }

    fn set_control_register(&mut self, register: ControlRegister) -> Result<(), Error<I2C>> {
        write_register(&mut self.bus, self.address, register)?;
        // Trigger yet another read to ensure we have the lastest value for the camera
        self.control_register()?;
        Ok(())
    }

    /// Get the last measured subpage.
    pub fn last_measured_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.status_register()?.last_updated_subpage)
    }

    /// Check if there is new data available, and if so, which subpage.
    pub fn data_available(&mut self) -> Result<Option<Subpage>, Error<I2C>> {
        let register = self.status_register()?;
        Ok(if register.new_data {
            Some(register.last_updated_subpage)
        } else {
            None
        })
    }

    /// Clear the data available flag, signaling to the camera that the controller is ready for
    /// more data.
    ///
    /// This flag can only be reset by the controller.
    pub fn reset_data_available(&mut self) -> Result<(), Error<I2C>> {
        let mut current = self.status_register()?;
        current.new_data = false;
        self.set_status_register(current)
    }

    /// Check if the overwrite enabled flag is set.
    ///
    /// This flag is only effective when `data_hold_enabled` is active.
    pub fn overwrite_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.status_register()?.overwrite_enabled)
    }

    set_register_field! {
        status_register,
        overwrite_enabled,
        "Enabled (or disable) overwriting of data in RAM with new data."
    }

    /// Check if the camera is using subpages.
    ///
    /// When disabled, only one page will be measured. The default is to use subpages.
    pub fn subpages_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.use_subpages)
    }

    set_register_field! {
        control_register,
        use_subpages,
        "Enabled (or disable) the use of subpages."
    }

    /// Check if the "Enable data hold" flag is set.
    ///
    /// When this flag (bit 2 on 0x800D) is set, data is not copied to RAM unless the
    /// `enable_overwrite` flag is set. The default is for this mode to be disabled.
    pub fn data_hold_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.data_hold)
    }

    set_register_field! {
        control_register,
        data_hold,
        "Enabled (or disable) data holding."
    }

    /// Check if the camera is in subpage repeat mode.
    ///
    /// This flag only has an effect if [subpages][Camera::subpages_enabled] is enabled. In subpage
    /// repeat mode, only the subpage set in `selected_subpage` will be measured and updated. When
    /// disabled, the active subpage will alternate between the two. The default is disabled.
    pub fn subpage_repeat(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.subpage_repeat)
    }

    set_register_field! {
        control_register,
        subpage_repeat,
        "Enabled (or disable) subpage repeat mode."
    }

    /// Get the currently selected subpage when [subpage repeat][Camera::subpage_repeat] is enabled.
    ///
    /// This setting only has an effect when `subpage_repeat` is enabled. The default value is
    /// `Subpage::Zero`.
    pub fn selected_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.control_register()?.subpage)
    }

    set_register_field! {
        control_register,
        subpage,
        Subpage,
        "Set the currently selected subpage when [subpage repeat][Camera::subpage_repeat] is enabled."
    }

    /// Read the frame rate from the camera.
    ///
    /// The default frame rate is [2 FPS][FrameRate::Two].
    pub fn frame_rate(&mut self) -> Result<FrameRate, Error<I2C>> {
        Ok(self.control_register()?.frame_rate)
    }

    set_register_field! {
        control_register,
        frame_rate,
        FrameRate,
        "Set camera's frame rate."
    }
    // TODO: Add a special-use function for setting the frame rate in EEPROM.

    /// Get the current resolution of the ADC in the camera.
    ///
    /// The default resolution is [18 bits][Resolution::Eighteen].
    pub fn resolution(&mut self) -> Result<Resolution, Error<I2C>> {
        Ok(self.control_register()?.resolution)
    }

    set_register_field! {
        control_register,
        resolution,
        Resolution,
        "Set ADC resolution within the camera."
    }

    /// Get the current access pattern used by the camera when updating subpages.
    ///
    /// The default for the MLX90640 is the chess patterm while the default for the MLX90641 is the
    /// interleaved pattern.
    pub fn access_pattern(&mut self) -> Result<AccessPattern, Error<I2C>> {
        Ok(self.control_register()?.access_pattern)
    }

    set_register_field! {
        control_register,
        access_pattern,
        AccessPattern,
        "Set the access pattern used by the camera."
    }
}

fn read_register<R, I2C>(bus: &mut I2C, address: u8) -> Result<R, Error<I2C>>
where
    I2C: i2c::WriteRead,
    R: Register,
{
    // Inner function to reduce the impact of monomorphization for Register. It'll still get
    // duplicated, but it should just be duplicated on I2C, and there should only be one of those
    // in an application (usually).
    fn read_register<I2C: i2c::WriteRead>(
        bus: &mut I2C,
        i2c_address: u8,
        register_address: Address,
    ) -> Result<[u8; 2], I2C::Error> {
        let register_address_bytes: [u8; 2] = register_address.into();
        let mut register_bytes = [0u8; 2];
        bus.write_read(i2c_address, &register_address_bytes, &mut register_bytes)?;
        Ok(register_bytes)
    }

    let register_address = R::address();
    let register_value = read_register(bus, address, register_address).map_err(Error::I2cError)?;
    let register = R::from(&register_value[..]);
    Ok(register)
}

fn write_register<R, I2C>(bus: &mut I2C, address: u8, register: R) -> Result<(), Error<I2C>>
where
    I2C: i2c::WriteRead,
    R: Register,
{
    fn write_register<I2C: i2c::WriteRead>(
        bus: &mut I2C,
        i2c_address: u8,
        register_address: [u8; 2],
        register_data: [u8; 2],
    ) -> Result<(), I2C::Error> {
        let combined: [u8; 4] = [
            register_address[0],
            register_address[1],
            register_data[0],
            register_data[1],
        ];
        bus.write_read(i2c_address, &combined, &mut [])?;
        Ok(())
    }
    let register_address = R::address();
    let register_address_bytes: [u8; 2] = register_address.into();
    // Can't use read_register(), as it strips the unused bytes off
    let mut existing_value = [0u8; 2];
    bus.write_read(address, &register_address_bytes, &mut existing_value)
        .map_err(Error::I2cError)?;
    let mut new_bytes: [u8; 2] = register.into();
    new_bytes
        .iter_mut()
        .zip(R::write_mask())
        .zip(existing_value)
        .for_each(|((new_value, mask), old_value)| {
            *new_value &= mask;
            *new_value |= old_value & !mask;
        });
    write_register(bus, address, register_address_bytes, new_bytes).map_err(Error::I2cError)?;
    Ok(())
}

#[cfg(test)]
mod test {
    extern crate std;
    use core::iter::FromIterator;
    use std::vec;

    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction};

    use crate::{mlx90640, I2cRegister, Mlx90640Camera, StatusRegister};

    #[test]
    fn create_mlx90640() {
        // Specifically using a non-default address to make sure assumptions aren't being made
        // about the address.
        let address: u8 = 0x30;
        let eeprom_vec = vec::Vec::from_iter(mlx90640::eeprom_data());
        let expected = [
            // Control register
            Transaction::write_read(address, vec![0x80, 0x0D], vec![0x00, 0x00]),
            // EEPROM
            Transaction::write_read(address, vec![0x24, 0x00], eeprom_vec),
        ];
        let mock_bus = I2cMock::new(&expected);
        Mlx90640Camera::new(mock_bus, address)
            .expect("A MLX90640 camera should be created after loading its data");
        // Test passes if we get this far.
    }

    #[test]
    fn read_register() {
        // Just picking addresses now
        let address = 0x10;
        let expected = [
            // Toggling all of the reserved bits on for kicks.
            Transaction::write_read(address, vec![0x80, 0x0F], vec![0xFF, 0xF0]),
        ];
        let mut mock_bus = I2cMock::new(&expected);
        let register: I2cRegister = super::read_register(&mut mock_bus, address).unwrap();
        assert_eq!(register, I2cRegister::default());
    }

    #[test]
    fn read_write_register() {
        let address = 0x42;
        // Using the status register for this test as it has read-only sections at both ends.
        let expected = [
            // Start with the read-only sections zeroed
            Transaction::write_read(address, vec![0x80, 0x00], vec![0x00, 0xE0]),
            // After modifying the register struct, write_register() will do another read, and all
            // reserved bits will be set, and the last updated subpage will have changed.
            // NOTE: the final byte is 0xC1, not 0xE1. This library handles step mode, which is
            // controlled by the 6th bit which is reserved in later datasheets).
            Transaction::write_read(address, vec![0x80, 0x00], vec![0xFF, 0xC1]),
            // Ensure that the reserved bits are kept, and the new setting (in this case, data
            // overwite) is also set
            Transaction::write_read(address, vec![0x80, 0x00, 0xFF, 0xD1], vec![]),
        ];
        let mut mock_bus = I2cMock::new(&expected);
        let mut status_register: StatusRegister =
            super::read_register(&mut mock_bus, address).unwrap();
        assert!(!status_register.overwrite_enabled);
        status_register.overwrite_enabled = true;
        super::write_register(&mut mock_bus, address, status_register).unwrap();
    }
}
