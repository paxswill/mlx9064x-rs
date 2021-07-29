// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use crate::mlx90641;

/// The size of the EEPROM for the MLX990640 and MLX909641 in bytes.
///
/// Both cameras (MLX90640 and MLX90641) have the same size EEPROM. 0x273F is the last address in
/// the EEPROM, so add one to that to include it, while 0x2400 is the first address. Each address
/// contains a 16-bit value, so we multiply by two to get the number of 8-bit bytes.
pub(crate) const EEPROM_LENGTH: usize = (0x2740 - 0x2400) * 2;

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
pub(crate) fn mlx90640_datasheet_eeprom() -> [u8; EEPROM_LENGTH] {
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

/// Example MLX9909641 EEPROM data from the datasheet.
// Same structure as MLX90640_EEPROM_HEADER.
const MLX90641_EEPROM_HEADER: &'static [u8] = b"\
    \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
    \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\
    \x00\x00\xb7\xe8\xd0\x16\x00\x00\x00\x00\xc2\xfd\x1a\x43\xca\x9a\
    \x51\x64\x01\x8c\x01\x8c\x01\x8c\x9c\xb1\x95\x6c\xa5\xcc\x7d\xd1\
    \x6d\x7f\x3c\xd4\x27\xb8\x19\xe6\xf1\x37\x78\x14\x26\x58\xef\x9e\
    \x91\x7f\xf0\x18\xe1\x56\x48\x17\x1c\x80\x23\x3e\xc8\x26\xcf\xfc\
    \xa0\x09\xbb\x53\xf1\x94\xfc\x00\x78\x14\xed\x22\xed\x22\xed\x22\
    \xed\x22\xed\x22\x80\xc8\xed\x22\x41\x90\xed\x22\xda\x58\xed\x22";

pub(crate) fn mlx90641_datasheet_eeprom() -> [u8; EEPROM_LENGTH] {
    let offset_0 = b"\xf8\x49";
    let k_ta_with_k_v = b"\xb8\xc0";
    let sensitivity = b"\xff\xff";
    // This is different than the datasheet! It looks like they used the subpage 0 data, but wanted
    // it to be slightly different so they changed the last digit, but then didn't recompute the checksum.
    let offset_1 = b"\xd8\x47";
    // The MLX90641 has the header data, then offsets for subpage 0 for each pixel, then the
    // sensitivity for each pixel, then a combined K_ta and K_v for each pixel, then offsets for
    // subpage 1 for each pixel.
    let mut eeprom_data = [0u8; EEPROM_LENGTH];
    let header_slice = &MLX90641_EEPROM_HEADER[..];
    eeprom_data[..header_slice.len()].copy_from_slice(header_slice);
    let mut offset = header_slice.len();
    // Multiply by two to get number of bytes
    let pixel_data_length = mlx90641::NUM_PIXELS * 2;
    for single_pixel_data in [offset_0, sensitivity, k_ta_with_k_v, offset_1] {
        eeprom_data[offset..(offset + pixel_data_length)]
            .iter_mut()
            .zip(
                single_pixel_data
                    .iter()
                    .copied()
                    .cycle()
                    .take(pixel_data_length),
            )
            .for_each(|(dest, src)| *dest = src);
        offset += pixel_data_length;
    }
    eeprom_data
}

mod test {
    #[test]
    fn smoke_mlx90640_eeprom() {
        super::mlx90640_datasheet_eeprom();
    }

    #[test]
    fn smoke_mlx90641_eeprom() {
        super::mlx90641_datasheet_eeprom();
    }
}
