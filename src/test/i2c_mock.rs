// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
extern crate alloc;

use alloc::collections::VecDeque;
use alloc::rc::Rc;
use core::cell::{Ref, RefCell};
use core::convert::TryInto;
use core::ops::RangeInclusive;

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;

use super::eeprom_data::{mlx90640_datasheet_eeprom, mlx90641_datasheet_eeprom, EEPROM_LENGTH};
use super::mlx90640_example_data;
use crate::{mlx90640, mlx90641, Address};

/// The number of bytes the MLX90640 has of RAM.
///
/// The MLX90640 has its RAM from 0x0400 through 0x07FF, but only uses up through 0x073F. The used
/// range representing 768 pixels along with 64 other addresses (half of which are reserved). Each
/// address corresponds to 16 bits of data.
pub(crate) const MLX90640_RAM_LENGTH: usize = (0x0740 - 0x0400) * 2;

/// The number of bytes the MLX90641 has of RAM.
///
/// The MLX90641 has its RAM from 0x0400 through 0x05BF, representing 192 pixels duplicated across
/// two subpages along with 64 other addresses (half of which are reserved). Each address
/// corresponds to 16 bits of data.
pub(crate) const MLX90641_RAM_LENGTH: usize = (0x05C0 - 0x0400) * 2;

const STATUS_REGISTER_ADDRESS: u16 = 0x8000;

// The lowest 6 bits are documented, but the 6th bit is only documented in earlier versions of the
// datasheet.
const STATUS_REGISTER_WRITE_MASK: [u8; 2] = [0x00, 0x3F];

const CONTROL_REGISTER_ONE_ADDRESS: u16 = 0x800D;

// Only the top three bits of control register 1 are reserved.
const CONTROL_REGISTER_1_WRITE_MASK: [u8; 2] = [0x1F, 0xFF];

// Control register 2 is not documented, so treat it as entirely reserved.
const CONTROL_REGISTER_2_WRITE_MASK: [u8; 2] = [0x00, 0x00];

const I2C_CONFIG_REGISTER_ADDRESS: u16 = 0x800F;

// Only the last four bits of the I2C config register are documented.
const I2C_CONFIG_REGISTER_WRITE_MASK: [u8; 2] = [0x00, 0x0F];

const RECENT_OPERATIONS_QUEUE_LENGTH: usize = 32;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u16)]
enum EepromAddress {
    ControlRegister1 = 0x240C,
    ControlRegister2 = 0x240D,
    I2cConfigRegister = 0x240E,
    I2cAddress = 0x240F,
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum MockError {
    /// The given address shouldn't be accessed.
    IllegalAccess(Address),

    /// The given address should not be written to.
    IllegalWriteAddress(Address),

    /// The given value is illegal for the given location.
    IllegalWriteValue(Address, u16),

    // The given address isn't valid for a device.
    UnknownMemoryAddress(Address),

    /// An unknown I2C address was given.
    UnknownI2cAddress(u8),

    /// The requested operation is not allowed.
    ///
    /// This covers things situations such as:
    /// * A combined write-read transaction with a write amount greater than 2 bytes (so more than
    ///   just an address).
    /// * A write-read transaction with a 0-length read (which causes the camera to reject the next
    ///   operation).
    /// * Read operations that aren't readying a full number of words (each word is two bytes).
    IllegalOperation,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum I2cOperation {
    Write { address: Address, length: usize },
    Read { address: Address, length: usize },
}

#[derive(Clone, Debug)]
pub(crate) struct MockCameraBus<const RAM_LENGTH: usize> {
    i2c_address: u8,
    rom_range: RangeInclusive<u16>,
    ram_range: RangeInclusive<u16>,
    eeprom_range: RangeInclusive<u16>,
    register_range: RangeInclusive<u16>,
    eeprom_data: Rc<RefCell<[u8; EEPROM_LENGTH]>>,
    ram_data: Rc<RefCell<[u8; RAM_LENGTH]>>,
    status_register: Rc<RefCell<[u8; 2]>>,
    control_register: Rc<RefCell<[u8; 2]>>,
    i2c_config_register: Rc<RefCell<[u8; 2]>>,
    recent_operations: Rc<RefCell<VecDeque<I2cOperation>>>,
}

impl MockCameraBus<MLX90640_RAM_LENGTH> {
    pub(crate) fn new_mlx90640(
        i2c_address: u8,
        eeprom: &[u8],
        ram: &[u8],
        control_register: &[u8],
        status_register: &[u8],
        i2c_register: &[u8],
    ) -> Self {
        let mut eeprom_data = [0u8; mlx90640_example_data::EEPROM_DATA.len()];
        eeprom_data.copy_from_slice(eeprom);
        let mut ram_data = [0u8; MLX90640_RAM_LENGTH];
        ram_data.copy_from_slice(ram);
        let mut control_register_owned = [0u8; 2];
        control_register_owned.copy_from_slice(control_register);
        let mut status_register_owned = [0u8; 2];
        status_register_owned.copy_from_slice(status_register);
        let mut i2c_register_owned = [0u8; 2];
        i2c_register_owned.copy_from_slice(i2c_register);
        MockCameraBus {
            i2c_address: i2c_address,
            rom_range: 0x0000..=0x03FF,
            ram_range: (mlx90640::RamAddress::Base.into())..=(mlx90640::RamAddress::End.into()),
            eeprom_range: 0x2400..=0x273F,
            register_range: 0x8000..=0x8016,
            eeprom_data: Rc::new(RefCell::new(eeprom_data)),
            ram_data: Rc::new(RefCell::new(ram_data)),
            status_register: Rc::new(RefCell::new(status_register_owned)),
            control_register: Rc::new(RefCell::new(control_register_owned)),
            i2c_config_register: Rc::new(RefCell::new(i2c_register_owned)),
            recent_operations: Rc::new(RefCell::new(VecDeque::new())),
        }
    }
}

impl<const RAM_LENGTH: usize> MockCameraBus<RAM_LENGTH> {
    fn extract_address(&self, bytes: &[u8]) -> Result<u16, MockError> {
        let mut addr_bytes = [0u8; 2];
        addr_bytes[..].copy_from_slice(&bytes[..2]);
        let address = u16::from_be_bytes(addr_bytes);
        if self.rom_range.contains(&address)
            || self.ram_range.contains(&address)
            || self.eeprom_range.contains(&address)
            || self.register_range.contains(&address)
        {
            Ok(address)
        } else {
            Err(MockError::UnknownMemoryAddress(address.into()))
        }
    }

    /// Access a slice containing `count` 16-bit words, starting at the given address.
    pub(crate) fn get(&self, address: Address, byte_count: usize) -> Result<Ref<[u8]>, MockError> {
        let start_address: u16 = address.into();
        let end_address = start_address + (byte_count / 2) as u16;
        // classify the address and check if it's supposed to be read
        if byte_count % 2 != 0 {
            // The camera's have a word size of 16 bits, so every read should be a multiple of two
            // bytes.
            Err(MockError::IllegalOperation)
        } else if self.rom_range.contains(&start_address) {
            // Shouldn't access the ROM
            Err(MockError::IllegalAccess(address))
        } else if self.ram_range.contains(&start_address) {
            // Can only read from RAM, but must stay within RAM bounds
            if self.ram_range.contains(&(end_address - 1)) {
                let slice_start = (start_address - self.ram_range.start()) as usize * 2;
                let slice_end = slice_start + byte_count;
                let slice_ref = Ref::map(self.ram_data.borrow(), |r| &r[slice_start..slice_end]);
                Ok(slice_ref)
                //Ok(&self.ram_data[slice_start..slice_end])
            } else {
                // Illegal, but use the end address as the given address.
                Err(MockError::IllegalAccess(end_address.into()))
            }
        } else if self.eeprom_range.contains(&start_address) {
            // Same as RAM, but now for the EEPROM bounds
            if self.eeprom_range.contains(&(end_address - 1)) {
                let slice_start = (start_address - self.eeprom_range.start()) as usize * 2;
                let slice_end = slice_start + byte_count;
                let slice_ref = Ref::map(self.eeprom_data.borrow(), |r| &r[slice_start..slice_end]);
                Ok(slice_ref)
            } else {
                Err(MockError::IllegalAccess(end_address.into()))
            }
        } else if self.register_range.contains(&start_address) {
            // There are only three registers, and they're non-contiguous, so only 1 word can be
            // read at a time.
            if byte_count != 2 {
                Err(MockError::IllegalAccess(end_address.into()))
            } else {
                match start_address {
                    STATUS_REGISTER_ADDRESS => {
                        let slice_ref = Ref::map(self.status_register.borrow(), |r| &r[..]);
                        Ok(slice_ref)
                    }
                    CONTROL_REGISTER_ONE_ADDRESS => {
                        let slice_ref = Ref::map(self.control_register.borrow(), |r| &r[..]);
                        Ok(slice_ref)
                    }
                    I2C_CONFIG_REGISTER_ADDRESS => {
                        let slice_ref = Ref::map(self.i2c_config_register.borrow(), |r| &r[..]);
                        Ok(slice_ref)
                    }
                    _ => Err(MockError::IllegalAccess(address)),
                }
            }
        } else {
            Err(MockError::UnknownMemoryAddress(address))
        }
    }

    pub(crate) fn set(&mut self, address: Address, data: &[u8]) -> Result<(), MockError> {
        let start_address: u16 = address.into();
        // Divide by two to get the number of words
        let end_address: u16 = start_address + (data.len() as u16 / 2);
        // Same as get, but a little more restrictive
        if self.rom_range.contains(&start_address) {
            // It's in the name, Read-Only Memory
            Err(MockError::IllegalWriteAddress(address))
        } else if self.ram_range.contains(&start_address) {
            // Only the camera is allowed to write to its RAM.
            Err(MockError::IllegalWriteAddress(address))
        } else if self.eeprom_range.contains(&start_address) {
            // Technically you can write anywhere in the EEPROM, but then the config data is lost
            // and can't be recovered. The configuration register, I2C config register, and the I2C
            // address register are the only safe locations that can be written. There is also a
            // control register 2 documented, but the contents are not documented, so writing
            // anything other than 0x0000 is treated as an error.
            let slice_start = ((start_address - self.eeprom_range.start()) as usize) * 2;
            let slice_end = slice_start + data.len();
            let existing_data = &mut self.eeprom_data.borrow_mut()[slice_start..slice_end];
            // Create a mask of which bytes are writeable
            // At most 4 words will be present.
            let mut write_mask: ArrayVec<u8, { 4 * 2 }> = ArrayVec::new();
            write_mask.extend(CONTROL_REGISTER_1_WRITE_MASK.iter().copied());
            write_mask.extend(CONTROL_REGISTER_2_WRITE_MASK.iter().copied());
            write_mask.extend(I2C_CONFIG_REGISTER_WRITE_MASK.iter().copied());
            // The lower half is the I2C address of the device, meaning the lower 7 bits.
            write_mask.extend([0x00, 0x7F]);
            // Control register 1 is the first address in the sequence
            let write_mask_start =
                ((start_address - EepromAddress::ControlRegister1 as u16) as usize) * 2;
            let write_mask = &write_mask[write_mask_start..data.len()];
            let zipped_chunks = existing_data
                .chunks_exact(2)
                .zip(write_mask.chunks_exact(2))
                .zip(data.chunks_exact(2));
            for (index, ((existing_bytes, mask_bytes), new_bytes)) in zipped_chunks.enumerate() {
                let existing_word = u16::from_be_bytes(existing_bytes.try_into().unwrap());
                let mask_word = u16::from_be_bytes(mask_bytes.try_into().unwrap());
                let new_word = u16::from_be_bytes(new_bytes.try_into().unwrap());
                // Check that the new data isn't overwriting any of the reserved bits.
                if !check_new_against_mask(existing_word, mask_word, new_word) {
                    let bad_address = (start_address + index as u16).into();
                    return Err(MockError::IllegalWriteValue(bad_address, new_word));
                }
            }
            // Only change the values after the entire write has been checked
            existing_data.copy_from_slice(data);
            Ok(())
        } else if self.register_range.contains(&start_address) {
            // There are only three registers, and they're non-contiguous, so only 1 word can be
            // written at a time. This is ignoring control register 2 as it's undocumented besides
            // a passing reference in the datasheet.
            // data length is of bytes, and each word is 2 bytes.
            if data.len() != 2 {
                Err(MockError::IllegalWriteAddress(end_address.into()))
            } else {
                let new_word = u16::from_be_bytes(data.try_into().unwrap());
                let (mask_bytes, existing_bytes) = match start_address {
                    STATUS_REGISTER_ADDRESS => (
                        STATUS_REGISTER_WRITE_MASK,
                        self.status_register.borrow().clone(),
                    ),
                    CONTROL_REGISTER_ONE_ADDRESS => (
                        CONTROL_REGISTER_1_WRITE_MASK,
                        self.control_register.borrow().clone(),
                    ),
                    I2C_CONFIG_REGISTER_ADDRESS => (
                        I2C_CONFIG_REGISTER_WRITE_MASK,
                        self.i2c_config_register.borrow().clone(),
                    ),
                    _ => return Err(MockError::IllegalWriteAddress(address)),
                };
                let mask_word = u16::from_be_bytes(mask_bytes);
                let existing_word = u16::from_be_bytes(existing_bytes.try_into().unwrap());
                if !check_new_against_mask(existing_word, mask_word, new_word) {
                    let bad_address = start_address.into();
                    return Err(MockError::IllegalWriteValue(bad_address, new_word));
                }
                match start_address {
                    STATUS_REGISTER_ADDRESS => {
                        self.status_register.borrow_mut().copy_from_slice(data)
                    }
                    CONTROL_REGISTER_ONE_ADDRESS => {
                        self.control_register.borrow_mut().copy_from_slice(data)
                    }
                    I2C_CONFIG_REGISTER_ADDRESS => {
                        self.i2c_config_register.borrow_mut().copy_from_slice(data)
                    }
                    _ => unreachable!(),
                }
                Ok(())
            }
        } else {
            Err(MockError::UnknownMemoryAddress(address))
        }
    }

    /// Replace the current RAM and status register from the given slices
    ///
    /// This is to simulate a new frame of data being made available. This function does *not*
    /// explicitly set the "new data available" flag. The value in the given status register data
    /// is used as-is.
    pub(crate) fn update_frame(&mut self, ram_data: &[u8], status_register: &[u8]) {
        self.ram_data.borrow_mut().copy_from_slice(ram_data);
        self.status_register
            .borrow_mut()
            .copy_from_slice(status_register);
    }

    /// Set the "new data available" flag in the status register to a new value
    pub(crate) fn set_data_available(&mut self, available: bool) {
        if available {
            self.status_register.borrow_mut()[1] |= 0x8;
        } else {
            self.status_register.borrow_mut()[1] &= 0x7;
        }
    }

    fn add_operation(&self, operation: I2cOperation) {
        let mut recent_ops = self.recent_operations.borrow_mut();
        recent_ops.push_front(operation);
        recent_ops.truncate(RECENT_OPERATIONS_QUEUE_LENGTH);
    }

    pub(crate) fn recent_operations(&self) -> Ref<VecDeque<I2cOperation>> {
        self.recent_operations.borrow()
    }

    pub(crate) fn clear_recent_operations(&self) {
        self.recent_operations.borrow_mut().clear()
    }
}

impl<const RAM_LENGTH: usize> i2c::Write for MockCameraBus<RAM_LENGTH> {
    type Error = MockError;

    fn write(&mut self, i2c_address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if i2c_address != self.i2c_address {
            return Err(MockError::UnknownI2cAddress(i2c_address));
        }
        // Just writes can only happen to registers or EEPROM
        let raw_address = self.extract_address(bytes)?;
        let address: Address = raw_address.into();
        let payload = &bytes[2..];
        self.set(address, payload)?;
        self.add_operation(I2cOperation::Write {
            address,
            length: payload.len(),
        });
        Ok(())
    }
}

impl<const RAM_LENGTH: usize> i2c::WriteRead for MockCameraBus<RAM_LENGTH> {
    type Error = MockError;

    fn write_read(
        &mut self,
        i2c_address: u8,
        write_buffer: &[u8],
        out_buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        if i2c_address != self.i2c_address {
            return Err(MockError::UnknownI2cAddress(i2c_address));
        }
        // Write-reads should only be writing the address, so write_buffer should only be two bytes
        if write_buffer.len() != 2 || out_buffer.len() == 0 {
            return Err(MockError::IllegalOperation);
        }
        let raw_address = self.extract_address(write_buffer)?;
        let address: Address = raw_address.into();
        self.add_operation(I2cOperation::Read {
            address,
            length: out_buffer.len(),
        });
        let read_data = self.get(address, out_buffer.len())?;
        out_buffer.copy_from_slice(&read_data[..]);
        Ok(())
    }
}

fn check_new_against_mask(existing: u16, mask: u16, new: u16) -> bool {
    (new & !mask) == (existing & !mask)
}

pub(crate) fn datasheet_mlx90640_at_address(i2c_address: u8) -> MockCameraBus<MLX90640_RAM_LENGTH> {
    let eeprom_data = mlx90640_datasheet_eeprom();
    // For the RAM, use the example data from the datasheet.
    let mut ram_data = [0u8; MLX90640_RAM_LENGTH];
    ram_data[..(mlx90640::NUM_PIXELS * 2)]
        .iter_mut()
        .zip(b"\x02\x61".iter().copied().cycle())
        .for_each(|(ram_byte, pixel_byte)| *ram_byte = pixel_byte);
    // Copy in the non-pixel values manually, as the other values are reserved
    let t_a_vbe = ((mlx90640::RamAddress::AmbientTemperatureVoltageBe - mlx90640::RamAddress::Base)
        * 2) as usize;
    let cp0 =
        ((mlx90640::RamAddress::CompensationPixelZero - mlx90640::RamAddress::Base) * 2) as usize;
    let gain = ((mlx90640::RamAddress::Gain - mlx90640::RamAddress::Base) * 2) as usize;
    let t_a_ptat = ((mlx90640::RamAddress::AmbientTemperatureVoltage - mlx90640::RamAddress::Base)
        * 2) as usize;
    let cp1 =
        ((mlx90640::RamAddress::CompensationPixelOne - mlx90640::RamAddress::Base) * 2) as usize;
    let v_dd_pix =
        ((mlx90640::RamAddress::PixelSupplyVoltage - mlx90640::RamAddress::Base) * 2) as usize;
    ram_data[t_a_vbe..(t_a_vbe + 2)].copy_from_slice(b"\x4b\xf2");
    ram_data[cp0..(cp0 + 2)].copy_from_slice(b"\xff\xca");
    ram_data[gain..(gain + 2)].copy_from_slice(b"\x18\x81");
    ram_data[t_a_ptat..(t_a_ptat + 2)].copy_from_slice(b"\x06\xaf");
    ram_data[cp1..(cp1 + 2)].copy_from_slice(b"\xff\xc8");
    ram_data[v_dd_pix..(v_dd_pix + 2)].copy_from_slice(b"\xcc\xc5");
    MockCameraBus::new_mlx90640(
        i2c_address,
        &eeprom_data[..],
        &ram_data[..],
        // Setting this status to mean that Subpage 1 is the current subpage, and it is new data.
        b"\x19\x01",
        // Default values for these taken from the datasheet
        b"\x00\x09",
        b"\x00\x00",
    )
}

pub(crate) fn mock_mlx90641_at_address(i2c_address: u8) -> MockCameraBus<MLX90641_RAM_LENGTH> {
    let eeprom_data = mlx90641_datasheet_eeprom();
    // For the RAM, use the example data from the datasheet.
    let mut ram_data = [0u8; MLX90641_RAM_LENGTH];
    ram_data[..(mlx90641::NUM_PIXELS * 2)]
        .iter_mut()
        .zip(b"\x03\xcc".iter().copied().cycle())
        .for_each(|(ram_byte, pixel_byte)| *ram_byte = pixel_byte);
    // Copy in the non-pixel values manually, as the other values are reserved
    let t_a_vbe = ((mlx90641::RamAddress::AmbientTemperatureVoltageBe - mlx90641::RamAddress::Base)
        * 2) as usize;
    let cp0 =
        ((mlx90641::RamAddress::CompensationPixelZero - mlx90641::RamAddress::Base) * 2) as usize;
    let gain = ((mlx90641::RamAddress::Gain - mlx90641::RamAddress::Base) * 2) as usize;
    let t_a_ptat = ((mlx90641::RamAddress::AmbientTemperatureVoltage - mlx90641::RamAddress::Base)
        * 2) as usize;
    let cp1 =
        ((mlx90641::RamAddress::CompensationPixelOne - mlx90641::RamAddress::Base) * 2) as usize;
    let v_dd_pix =
        ((mlx90641::RamAddress::PixelSupplyVoltage - mlx90641::RamAddress::Base) * 2) as usize;
    ram_data[t_a_vbe..(t_a_vbe + 2)].copy_from_slice(b"\x4c\x54");
    ram_data[cp0..(cp0 + 2)].copy_from_slice(b"\xff\x97");
    ram_data[gain..(gain + 2)].copy_from_slice(b"\x26\x06");
    ram_data[t_a_ptat..(t_a_ptat + 2)].copy_from_slice(b"\x06\xd8");
    // The worked example only gives one compensation pixel value, so we're using it for both.
    ram_data[cp1..(cp1 + 2)].copy_from_slice(b"\xff\x97");
    ram_data[v_dd_pix..(v_dd_pix + 2)].copy_from_slice(b"\xcb\x8a");
    MockCameraBus {
        i2c_address,
        rom_range: 0x0000..=0x03FF,
        ram_range: (mlx90641::RamAddress::Base.into())..=(mlx90641::RamAddress::End.into()),
        eeprom_range: 0x2400..=0x273F,
        register_range: 0x8000..=0x8016,
        eeprom_data: Rc::new(RefCell::new(eeprom_data)),
        ram_data: Rc::new(RefCell::new(ram_data)),
        // The worked example uses subpage 0, so mark that as the current subpage with new data.
        status_register: Rc::new(RefCell::new([0x00, 0x08])),
        control_register: Rc::new(RefCell::new([0x09, 0x01])),
        i2c_config_register: Rc::new(RefCell::new([0x00, 0x00])),
        recent_operations: Rc::new(RefCell::new(VecDeque::new())),
    }
}

pub(crate) fn example_mlx90640_at_address(i2c_address: u8) -> MockCameraBus<MLX90640_RAM_LENGTH> {
    MockCameraBus::new_mlx90640(
        i2c_address,
        &mlx90640_example_data::EEPROM_DATA[..],
        &mlx90640_example_data::FRAME_0_DATA[..],
        &mlx90640_example_data::CONTROL_REGISTER[..],
        &mlx90640_example_data::FRAME_0_STATUS_REGISTER[..],
        b"\x00\x00",
    )
}
