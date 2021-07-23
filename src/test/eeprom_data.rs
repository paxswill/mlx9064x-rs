// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

/// The size of the EEPROM for the MLX990640 and MLX909641 in bytes.
///
/// Both cameras (MLX90640 and MLX90641) have the same size EEPROM. 0x273F is the last address in
/// the EEPROM, so add one to that to include it, while 0x2400 is the first address. Each address
/// contains a 16-bit value, so we multiply by two to get the number of 8-bit bytes.
pub(super) const EEPROM_LENGTH: usize = (0x2740 - 0x2400) * 2;

/// Example MLX90640 EEPROM data from the datasheet (from the worked example).
// Each line is 8 bytes. The first two lines are empty, as that data is ignored for calibration
// purposes. The next six lines are the shared calibration data.
const MLX90640_EEPROM_HEADER: &'static [u8] = b"\
    \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
    \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
    \x42\x10\xff\xbb\x02\x02\xf2\x02\xf2\xf2\xe2\xe2\xd1\xe1\xb1\xd1\
    \xf1\x0f\xf0\x0f\xe0\xef\xe0\xef\xe1\xe1\xf3\xf2\xf4\x04\xe5\x04\
    \x79\xa6\x2f\x44\xff\xdd\x22\x10\x33\x33\x22\x33\xef\x01\x9a\xcc\
    \xee\xdc\x10\xff\x22\x21\x33\x33\x23\x33\x01\x12\xee\xff\xbb\xdd\
    \x18\xef\x2f\xf1\x59\x52\x9d\x68\x54\x54\x09\x94\x69\x56\x53\x54\
    \x23\x63\xe4\x46\xfb\xb5\x04\x4b\xf0\x20\x97\x97\x97\x97\x28\x89";

/// Create a buffer with the example MLX909640 EEPROM data.
pub(crate) fn mlx90640_eeprom_data() -> [u8; EEPROM_LENGTH] {
    // Create the EEPROM data by starting with the header and then filling in the rest with the
    // pixel used in the worked example from the datasheet.
    let pixel_data = b"\x08\xa0";
    let mut eeprom_data = [0u8; EEPROM_LENGTH];
    let header_slice = &MLX90640_EEPROM_HEADER[..];
    eeprom_data[..header_slice.len()].copy_from_slice(header_slice);
    eeprom_data[header_slice.len()..]
        .iter_mut()
        .zip(pixel_data.iter().copied().cycle())
        .for_each(|(eeprom_byte, pixel_byte)| *eeprom_byte = pixel_byte);
    eeprom_data
}
