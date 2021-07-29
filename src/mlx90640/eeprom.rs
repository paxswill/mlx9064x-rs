// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::slice;

use arrayvec::ArrayVec;
use bytes::Buf;
use embedded_hal::blocking::i2c;

use crate::common::*;
use crate::error::{Error, LibraryError};
use crate::expose_member;
use crate::register::Subpage;
use crate::util::i16_from_bits;

use super::address::EepromAddress;
use super::{NUM_PIXELS, WIDTH};

/// The number of corner temperatures an MLX90640 has.
const NUM_CORNER_TEMPERATURES: usize = 4;

/// The basic temperature range. See discussion on [CalibrationData::basic_range] for more details.
// It's defined as 1 in the datasheet(well, 2, but 1-indexed, so 1 when 0-indexed).
const BASIC_TEMPERATURE_RANGE: usize = 1;

/// The word size of the MLX990640 in terms of 8-bit bytes.
const WORD_SIZE: usize = 16 / 8;

/// MLX990640-specific calibration processing.
#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640Calibration {
    k_v_dd: i16,

    v_dd_25: i16,

    resolution: u8,

    k_v_ptat: f32,

    k_t_ptat: f32,

    v_ptat_25: f32,

    alpha_ptat: u16,

    gain: f32,

    k_s_ta: f32,

    corner_temperatures: [i16; NUM_CORNER_TEMPERATURES],

    k_s_to: [f32; NUM_CORNER_TEMPERATURES],

    alpha_correction: [f32; NUM_CORNER_TEMPERATURES],

    alpha_pixels: [f32; NUM_PIXELS],

    alpha_cp: [f32; 2],

    offset_reference_pixels: [i16; NUM_PIXELS],

    offset_reference_cp: [i16; 2],

    k_v_pattern: [f32; 4],

    k_v_cp: f32,

    k_ta_pixels: [f32; NUM_PIXELS],

    k_ta_cp: f32,

    temperature_gradient_coefficient: Option<f32>,
}

impl Mlx90640Calibration {
    /// Calculate pixel calibration values based off of the row and column data.
    ///
    /// Remainder data will be added in afterwards. This function is used for both offset and
    /// sensitivity (alpha) arrays. The given buffer must be at the word containing the column, row
    /// and remainder scaling factors.
    /// The calculated array, the remainder scaling factor, and the value occupying the 4 bits
    /// preceding the scaling factors are returned (in that order).
    fn calculate_bulk_pixel_calibration<B: Buf>(data: &mut B) -> ([i16; NUM_PIXELS], u8, u8) {
        let (extra_value, row_scale, column_scale, remainder_scale) = {
            let scales = word_to_u4s(data);
            (scales[0], scales[1], scales[2], scales[3])
        };
        let offset_average = data.get_i16();
        let mut pixel_calibration = [offset_average; NUM_PIXELS];
        const VALUES_PER_DATA_ROW: usize = 4;
        // Add row offsets
        for row_chunks in pixel_calibration.chunks_exact_mut(WIDTH * VALUES_PER_DATA_ROW) {
            let rows_coefficients = word_to_i4s(data);
            // Create a nice lazy iterator that converts the values to i16, scales them, and
            // reverses that order of the data (because the data is laid out backwards in the EEPROM).
            let rows_coefficients = core::array::IntoIter::new(rows_coefficients)
                .map(i16::from)
                .map(|coeff| coeff << row_scale)
                .rev();
            for (row, coefficient) in row_chunks.chunks_exact_mut(WIDTH).zip(rows_coefficients) {
                row.iter_mut().for_each(|element| *element += coefficient);
            }
        }
        // Add column offsets. Slightly more involved as the offsets are in row-major order.
        for column_chunk_index in 0..(WIDTH / VALUES_PER_DATA_ROW) {
            // TODO: This could probably be optimized better
            let column_coefficients = word_to_i4s(data);
            // Same deal as the row coefficients, except cycle so that the same iterator can be
            // re-used multiple times.
            let column_coefficients = column_coefficients
                .iter()
                .copied()
                .map(i16::from)
                .map(|coeff| coeff << column_scale)
                .rev();
            for row in pixel_calibration.chunks_exact_mut(WIDTH) {
                let start_index = column_chunk_index * VALUES_PER_DATA_ROW;
                let row_range = start_index..(start_index + VALUES_PER_DATA_ROW);
                row[row_range]
                    .iter_mut()
                    .zip(column_coefficients.clone())
                    .for_each(|(element, coefficient)| *element += coefficient);
            }
        }
        (pixel_calibration, remainder_scale, extra_value)
    }

    /// Generate a chessboard-patterned sequence from four values.
    ///
    /// The given values are ordered:
    /// 1. even row, even column
    /// 2. odd row, even column
    /// 3. even row, odd column
    /// 4. odd row, odd column
    ///
    /// This order is the same order the MLX90640 stores this calibration data in its EEPROM. Also
    /// note that the naming scheme in this library is 0-indexed, while the datasheet is 1-indexed,
    /// meaning even and odd are swapped between the two.
    ///
    /// The returned iterator yields values in row-major order.
    fn repeat_chessboard<T>(source_values: [i8; 4]) -> impl Iterator<Item = T>
    where
        T: From<i8> + core::fmt::Debug,
    {
        let row_even_col_even = source_values[0];
        let row_odd_col_even = source_values[1];
        let row_even_col_odd = source_values[2];
        let row_odd_col_odd = source_values[3];
        // Create a pattern for even or odd rows, starting from column 0
        let even_row_pattern = core::array::IntoIter::new([row_even_col_even, row_even_col_odd]);
        let odd_row_pattern = core::array::IntoIter::new([row_odd_col_even, row_odd_col_odd]);
        // Repeat the pattern across the row
        let even_row = even_row_pattern.cycle().take(WIDTH).map(T::from);
        let odd_row = odd_row_pattern.cycle().take(WIDTH).map(T::from);
        // Then chain the two rows together, repeating to fill the array
        let repeating_rows = even_row.chain(odd_row).cycle();
        repeating_rows.take(NUM_PIXELS)
    }

    /// Calculate the per-pixel K<sub>T<sub>A</sub></sub> values.
    ///
    /// Similar to the offset and sensitivity values, the K<sub>T<sub>A</sub></sub> values have a
    /// per-pixel calibration value that is added to an average value shared by multiple pixels.
    /// Where the offset and sensitivity averages are calculated on a per row and column basis,
    /// this value is chosen from four values, determined by if the row and column indices are even
    /// or odd. The rest of the calculation is performed later, with the rest of the per-pixel
    /// calculations.
    fn generate_k_ta_pixels<B: Buf>(data: &mut B) -> impl Iterator<Item = i16> {
        let source_data: ArrayVec<i8, 4> = (0..4).map(|_| data.get_i8()).collect();
        let source_data = source_data.into_inner().unwrap();
        Self::repeat_chessboard(source_data)
    }

    /// Generate the constants needed for temperature calculations from a dump of the MLX90640
    /// EEPROM.
    ///
    /// The buffer must cover *all* of the EEPROM.
    pub fn from_data(data: &[u8]) -> Result<Self, LibraryError> {
        let mut buf = data;
        let eeprom_length = usize::from(EepromAddress::End - EepromAddress::Base);
        if buf.remaining() < eeprom_length {
            return Err(LibraryError::Other(
                "Not enough space left in buffer to be a full EEPROM dump",
            ));
        }
        // Skip the first 16 words, they're irrelevant
        buf.advance(WORD_SIZE * 16);
        // alpha_PTAT and offset compensation correction scales
        let (mut offset_reference_pixels, offset_correction_remainder_scale, alpha_ptat) =
            Self::calculate_bulk_pixel_calibration(&mut buf);
        let alpha_ptat = alpha_ptat / 4 + 8;
        let (alpha_pixels, alpha_correction_remainder_scale, alpha_scale_exp) =
            Self::calculate_bulk_pixel_calibration(&mut buf);
        let alpha_pixels: ArrayVec<f32, NUM_PIXELS> = core::array::IntoIter::new(alpha_pixels)
            .map(f32::from)
            .collect();
        // Safe to unwrap as the length of pixel arrays are *all* NUM_PIXELS long.
        let mut alpha_pixels = alpha_pixels.into_inner().unwrap();
        // Calculate the actual alpha scaling value from the exponent value. The alpha scaling
        // exponenet also has 30 added to it (not 27 like alpha_scale_cp).
        let alpha_scale = f32::from(alpha_scale_exp + 30).exp2();
        let gain = buf.get_i16();
        let v_ptat_25 = buf.get_i16();
        let (k_v_ptat, kt_ptat_bytes) = word_6_10_split(&mut buf);
        let k_t_ptat = i16_from_bits(&kt_ptat_bytes, 10);
        let k_v_dd = (buf.get_i8() as i16) << 5;
        // The data in EEPROM is unsigned, so we upgrade to a signed type as it's immediately sent
        // negative (by subtracting 256), then multipled by 2^5, and finally has 2^13 subtracted
        // from it.
        let v_dd_25 = ((buf.get_u8() as i16) - 256) * (1 << 5) - (1 << 13);
        // Keep this value around for actual processing once we have kv_scale.
        let k_v_avg = word_to_i4s(&mut buf);
        // TODO: Add interleaved mode compensation. Until then, skip that data
        buf.advance(WORD_SIZE);
        let lazy_k_ta_pixels = Self::generate_k_ta_pixels(&mut buf);
        let unpacked_scales = word_to_u4s(&mut buf);
        // The resolution control calibration value is just two bits in the high half of the byte.
        // The two other two bits are reserved, so we just drop them.
        let resolution = unpacked_scales[0] & 0x3;
        // various scaling constants
        let k_v_scale = f32::from(unpacked_scales[1]).exp2();
        // k_ta_scale1 has 8 added to it.
        let k_ta_scale1 = f32::from(unpacked_scales[2] + 8).exp2();
        // Leaving k_ta_scale2 as just the exponenet, as it's small enough to be used to shift it's
        // operand directly.
        let k_ta_scale2_exp = unpacked_scales[3];
        // We have k_v_scale now, calculate k_v_pattern
        let k_v_pattern: ArrayVec<f32, 4> = core::array::IntoIter::new(k_v_avg)
            .map(|v| f32::from(v) / k_v_scale)
            .collect();
        // Safe to unwrap as the input was only four elements, and the array is only 4 elements.
        let k_v_pattern = k_v_pattern.into_inner().unwrap();
        // Compensation pixel parameters
        let alpha_cp = {
            let (alpha_cp_ratio, alpha_cp_bytes) = word_6_10_split(&mut buf);
            let alpha_cp_ratio = f32::from(alpha_cp_ratio) / 7f32.exp2();
            // NOTE: the alpha scale value read from EEPROM has 27 added to get alpha_scale_cp, but
            // 30 added for alpha_scale_pixel
            let alpha_scale_cp = f32::from(alpha_scale_exp + 27).exp2();
            let alpha_cp0: f32 = f32::from(u16::from_be_bytes(alpha_cp_bytes)) / alpha_scale_cp;
            [alpha_cp0, alpha_cp0 * (1f32 + alpha_cp_ratio)]
        };
        let offset_reference_cp = {
            let (offset_cp_delta, offset_cp_bytes) = word_6_10_split(&mut buf);
            let offset_cp0 = i16_from_bits(&offset_cp_bytes, 10);
            [offset_cp0, offset_cp0 + i16::from(offset_cp_delta)]
        };
        let k_v_cp = f32::from(buf.get_i8()) / k_v_scale;
        let k_ta_cp = f32::from(buf.get_i8()) / k_ta_scale1;
        let k_s_ta = f32::from(buf.get_i8()) / 13f32.exp2();
        let temperature_gradient_coefficient = match buf.get_i8() {
            0 => None,
            n => Some(f32::from(n) / 5f32.exp2()),
        };
        // k_s_to is unscaled until k_s_to_scale is unpacked.
        let mut k_s_to_ranges: ArrayVec<f32, 4> = (0..4).map(|_| f32::from(buf.get_i8())).collect();
        // Fix the ordering of the elements from the EEPROM
        k_s_to_ranges.swap(0, 1);
        k_s_to_ranges.swap(2, 3);
        // Safe to unwrap as I'm just using ArrayVec to collect into an array.
        let mut k_s_to = k_s_to_ranges.into_inner().unwrap();
        // Very similar to the resolution and k_*_scale word a few lines above.
        let unpacked_corner_temps = word_to_u4s(&mut buf);
        // Like before, the top two bits are reserved. This time though, the temperature step
        // is multipled by 10. Also convert to i16 for use in calculations.
        let corner_temperature_step = i16::from(unpacked_corner_temps[0] & 0x3) * 10;
        // corner temperatures need to be multipled by the step and converted to i16
        let ct2 = i16::from(unpacked_corner_temps[1]) * corner_temperature_step;
        let ct3 = i16::from(unpacked_corner_temps[2]) * corner_temperature_step + ct2;
        // k_s_to_scale needs 8 added to it, then take 2 raised to this value.
        let k_s_to_scale = f32::from(unpacked_corner_temps[3] + 8).exp2();
        // -40 and 0 are hard-coded values for CT0 and CT1 (labelled CT1 and CT2 in the datasheet)
        let corner_temperatures = [-40i16, 0, ct2, ct3];
        // Now that we have k_s_to_scale, we can scale k_s_to properly:
        k_s_to.iter_mut().for_each(|k_s_to| *k_s_to /= k_s_to_scale);
        let alpha_correction =
            alpha_correction_coefficients(BASIC_TEMPERATURE_RANGE, &corner_temperatures, &k_s_to);
        // Calculate the rest of the per-pixel data using the remainder/k_ta data
        let mut k_ta_pixels = [0f32; NUM_PIXELS];
        offset_reference_pixels
            .iter_mut()
            .zip(alpha_pixels.iter_mut())
            .zip(lazy_k_ta_pixels)
            .zip(k_ta_pixels.iter_mut())
            .for_each(|(((offset, alpha), k_ta_numerator), k_ta)| {
                // TODO: handle failed pixels (where the pixel data is 0x0000)
                // TODO: outlier/deviant pixels
                let high = buf.get_u8();
                let low = buf.get_u8();
                // Normal dance to extend the sign bit from an i6 to an i8
                let offset_remainder = i16::from(i8::from_ne_bytes([high & 0xFC]) >> 2);
                *offset += offset_remainder << offset_correction_remainder_scale;
                // alpha is going to be a little weird: not only is there the i6-shift-dance, but
                // there's an extra shift right by 4 to drop the k_ta and outlier bits.
                let alpha_remainder = (i16::from_be_bytes([high & 0x03, low]) << 6) >> 10;
                *alpha += f32::from(alpha_remainder << alpha_correction_remainder_scale);
                *alpha /= alpha_scale;
                // To try to keep floating point errors down ad long as possible, do all the
                // operations for the numerator as ints, then convert to floats for the final division.
                let k_ta_remainder = i16::from(i8::from_ne_bytes([low & 0x0E]) >> 1);
                let k_ta_numerator =
                    f32::from(k_ta_numerator + (k_ta_remainder << k_ta_scale2_exp));
                *k_ta = k_ta_numerator / k_ta_scale1;
            });
        Ok(Self {
            k_v_dd,
            v_dd_25,
            resolution,
            k_v_ptat: k_v_ptat.into(),
            k_t_ptat: k_t_ptat.into(),
            v_ptat_25: v_ptat_25.into(),
            alpha_ptat: alpha_ptat.into(),
            gain: gain.into(),
            k_s_ta,
            corner_temperatures,
            k_s_to,
            alpha_correction,
            alpha_pixels,
            alpha_cp,
            offset_reference_pixels,
            offset_reference_cp,
            k_v_pattern,
            k_v_cp,
            k_ta_pixels,
            k_ta_cp,
            temperature_gradient_coefficient,
        })
    }
}

impl<I2C> FromI2C<I2C> for Mlx90640Calibration
where
    I2C: i2c::WriteRead + i2c::Write,
{
    type Error = Error<I2C>;
    type Ok = Self;

    fn from_i2c(bus: &mut I2C, i2c_address: u8) -> Result<Self, Error<I2C>> {
        // Dump the EEPROM. Both cameras use the same size and starting offset for their EEPROM.
        const EEPROM_LENGTH: usize =
            (EepromAddress::End as usize - EepromAddress::Base as usize + 1) * 2;
        let mut eeprom_buf = [0u8; EEPROM_LENGTH];
        let eeprom_base: Address = EepromAddress::Base.into();
        bus.write_read(i2c_address, &eeprom_base.as_bytes(), &mut eeprom_buf)
            .map_err(Error::I2cWriteReadError)?;
        Ok(Self::from_data(&eeprom_buf)?)
    }
}

impl<'a> CalibrationData<'a> for Mlx90640Calibration {
    expose_member!(k_v_dd, i16);
    expose_member!(v_dd_25, i16);
    expose_member!(resolution, u8);
    expose_member!(k_v_ptat, f32);
    expose_member!(k_t_ptat, f32);
    expose_member!(v_ptat_25, f32);
    expose_member!(alpha_ptat, u16);
    expose_member!(gain, f32);
    expose_member!(k_s_ta, f32);

    expose_member!(&corner_temperatures, [i16]);
    expose_member!(&k_s_to, [f32]);
    expose_member!(&alpha_correction, [f32]);

    fn basic_range(&self) -> usize {
        BASIC_TEMPERATURE_RANGE
    }

    type OffsetReferenceIterator = slice::Iter<'a, i16>;

    fn offset_reference_pixels(&'a self, _subpage: Subpage) -> Self::OffsetReferenceIterator {
        self.offset_reference_pixels.iter()
    }

    fn offset_reference_cp(&self, subpage: Subpage) -> i16 {
        self.offset_reference_cp[subpage as usize]
    }

    type AlphaIterator = slice::Iter<'a, f32>;

    fn alpha_pixels(&'a self, _subpage: Subpage) -> Self::AlphaIterator {
        self.alpha_pixels.iter()
    }

    fn alpha_cp(&self, subpage: Subpage) -> f32 {
        self.alpha_cp[subpage as usize]
    }

    type KvIterator = ChessboardIter<'a, f32>;

    fn k_v_pixels(&'a self, _subpage: Subpage) -> Self::KvIterator {
        ChessboardIter::new(&self.k_v_pattern)
    }

    fn k_v_cp(&self, _subpage: Subpage) -> f32 {
        self.k_v_cp
    }

    type KtaIterator = slice::Iter<'a, f32>;

    fn k_ta_pixels(&'a self, _subpage: Subpage) -> Self::KtaIterator {
        self.k_ta_pixels.iter()
    }

    fn k_ta_cp(&self, _subpage: Subpage) -> f32 {
        self.k_ta_cp
    }

    expose_member!(temperature_gradient_coefficient, Option<f32>);
}

#[derive(Clone, Debug, PartialEq)]
pub struct ChessboardIter<'a, T: 'a> {
    index: usize,
    source: &'a [T],
}

impl<'a, T: 'a> ChessboardIter<'a, T> {
    /// Repeat a sequence of values in a chessboard pattern.
    ///
    /// The given slice must have at least four values (if it has more than four, they are
    /// ignored). The values in the slice will be used for values like so (assuming 0-indexed rows
    /// and columns):
    /// 1. even row, even column
    /// 2. odd row, even column
    /// 3. even row, odd column
    /// 4. odd row, odd column
    fn new(source: &'a [T]) -> Self {
        Self { index: 0, source }
    }
}

impl<'a, T: 'a> Iterator for ChessboardIter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < NUM_PIXELS {
            let row = self.index / WIDTH;
            let column = self.index % WIDTH;
            self.index += 1;
            Some(match (row % 2 == 0, column % 2 == 0) {
                (true, true) => &self.source[0],
                (false, true) => &self.source[1],
                (true, false) => &self.source[2],
                (false, false) => &self.source[3],
            })
        } else {
            None
        }
    }
}

/// Split a word into a 6-bit value and a 10-bit value.
///
/// Further conversion for the second value is left to the caller.
fn word_6_10_split<B: Buf>(data: &mut B) -> (i8, [u8; 2]) {
    let mut word = [data.get_u8(), data.get_u8()];
    // Copy out the 6-bit value, and shift it over. As signed right shifts are aritmetic, the sign
    // bit gets extended, and we get the value we wanted.
    let six_bit = i8::from_ne_bytes([word[0]]) >> 2;
    // Mask off the extra from the high byte
    word[0] &= 0x03;
    (six_bit, word)
}

/// Extract four unsigned, 4-bit integers from a buffer
fn word_to_u4s<B: Buf>(data: &mut B) -> [u8; 4] {
    let high = data.get_u8();
    let low = data.get_u8();
    [(high & 0xF0) >> 4, high & 0xF, (low & 0xF0) >> 4, low & 0xF]
}

/// Split a byte into a pair of signed, four-bit integers.
fn u8_to_i4s(byte: u8) -> [i8; 2] {
    // Start by splitting the two numbers into a pair of bytes, with the MSB of the i4 all the way
    // to the left.
    let high = byte & 0xF0;
    let low = (byte & 0xF) << 4;
    // Create i8s from the pair of bytes, then arithmetic shift right by 4 to extend the sign.
    // Endian-ness shouldn't matter as these are single bytes.
    let high = i8::from_ne_bytes([high]);
    let low = i8::from_ne_bytes([low]);
    [high >> 4, low >> 4]
}

/// Split a word (2 bytes) from a buffer into four, 4-bit signed integers.
fn word_to_i4s<B: Buf>(data: &mut B) -> [i8; 4] {
    let high = data.get_u8();
    let low = data.get_u8();
    let high = u8_to_i4s(high);
    let low = u8_to_i4s(low);
    [high[0], high[1], low[0], low[1]]
}

#[cfg(test)]
pub(crate) mod test {
    #[cfg(feature = "std")]
    extern crate std;
    #[cfg(feature = "std")]
    use std::{print, println};

    use arrayvec::ArrayVec;

    use crate::common::CalibrationData;
    use crate::mlx90640::{HEIGHT, NUM_PIXELS, WIDTH};
    use crate::register::Subpage;
    use crate::test::mlx90640_eeprom_data;

    use super::Mlx90640Calibration;

    pub(crate) fn eeprom() -> Mlx90640Calibration {
        let mut eeprom_bytes = mlx90640_eeprom_data();
        Mlx90640Calibration::from_data(&mut eeprom_bytes).expect("The EEPROM data to be parsed.")
    }

    #[test]
    fn word_6_10_split() {
        fn check(mut data: &[u8], little: i8, remainder: [u8; 2]) {
            let split = super::word_6_10_split(&mut data);
            assert_eq!(
                split.0, little,
                "word_6_10_split failed to split the upper 6 bits off"
            );
            assert_eq!(
                split.1, remainder,
                "word_6_10_split failed to split the lower 10 bits off"
            );
        }
        // Start with basic premise, that the first 6 bits are split off
        check(b"\xfc\x00", -1, [0x00, 0x00]);
        // Then check the "reverse"
        check(b"\x03\xff", 0, [0x03, 0xFF]);
        // Negative max i6, with u10::MAX as well
        check(b"\x83\xff", -32, [0x03, 0xFF]);
    }

    #[test]
    fn word_to_u4s() {
        let mut sequence: &[u8] = b"\x12\x34";
        assert_eq!(super::word_to_u4s(&mut sequence), [1, 2, 3, 4]);
        let mut max_min: &[u8] = b"\xf0\xf0";
        assert_eq!(super::word_to_u4s(&mut max_min), [0xF, 0, 0xF, 0]);
        let mut min_max: &[u8] = b"\x0f\x0f";
        assert_eq!(super::word_to_u4s(&mut min_max), [0, 0xF, 0, 0xF]);
    }

    #[test]
    fn u8_to_i4s() {
        assert_eq!(super::u8_to_i4s(0x44), [4, 4]);
        assert_eq!(super::u8_to_i4s(0x88), [-8, -8]);
        assert_eq!(super::u8_to_i4s(0x48), [4, -8]);
        assert_eq!(super::u8_to_i4s(0x84), [-8, 4]);
    }

    #[test]
    fn repeat_chessboard() {
        // The pattern order is (for row, column): EE, OE, EO, OO
        let pattern = [1, 2, 3, 4];
        let test_pattern: ArrayVec<i8, NUM_PIXELS> =
            Mlx90640Calibration::repeat_chessboard(pattern).collect();
        // Print the test pattern (when std is available), as that makes it much easier to see
        // what's going on.
        #[cfg(feature = "std")]
        for row in 0..HEIGHT {
            for column in 0..WIDTH {
                let index = row * WIDTH + column;
                print!("{} ", test_pattern[index]);
            }
            println!();
        }
        for column in 0..WIDTH {
            for row in 0..HEIGHT {
                let index = row * WIDTH + column;
                let expected = match (row % 2, column % 2) {
                    (0, 0) => 1,
                    (1, 0) => 2,
                    (0, 1) => 3,
                    (1, 1) => 4,
                    (_, _) => unreachable!(),
                };
                assert_eq!(
                    test_pattern[index], expected,
                    "pattern incorrect at index {}",
                    index
                );
            }
        }
    }

    /// Check that it can even create itself from a buffer.
    #[test]
    fn smoke() {
        eeprom();
    }

    // Ordering these tests in the same order as the data sheet's worked example.

    macro_rules! datasheet_test {
        ($name:ident, $value:literal) => {
            #[test]
            fn $name() {
                assert_eq!(eeprom().$name(), $value);
            }
        };
    }

    datasheet_test!(resolution, 2);
    datasheet_test!(k_v_dd, -3168);
    datasheet_test!(v_dd_25, -13056);
    datasheet_test!(v_dd_0, 3.3);
    // NOTE: This is not the actual k_v_ptat, this is the unscaled value!
    datasheet_test!(k_v_ptat, 22f32);
    // Again, unscaled value
    datasheet_test!(k_t_ptat, 338f32);
    datasheet_test!(v_ptat_25, 12273f32);
    datasheet_test!(alpha_ptat, 9);
    datasheet_test!(gain, 6383f32);

    #[test]
    fn pixel_offset() {
        let e = eeprom();
        let offsets0: ArrayVec<i16, NUM_PIXELS> =
            e.offset_reference_pixels(Subpage::One).copied().collect();
        let offsets1: ArrayVec<i16, NUM_PIXELS> =
            e.offset_reference_pixels(Subpage::Zero).copied().collect();
        // MLX90640 doesn't vary offsets on subpage
        assert_eq!(offsets0, offsets1);
        let index = 11 * WIDTH + 15;
        let pixel = offsets0[index];
        assert_eq!(pixel, -75);
    }

    #[test]
    fn pixel_k_ta() {
        let e = eeprom();
        let k_ta0: ArrayVec<f32, NUM_PIXELS> = e.k_ta_pixels(Subpage::One).copied().collect();
        let k_ta1: ArrayVec<f32, NUM_PIXELS> = e.k_ta_pixels(Subpage::Zero).copied().collect();
        // MLX90640 doesn't vary k_ta on subpage
        assert_eq!(k_ta0, k_ta1);
        let index = 11 * WIDTH + 15;
        let pixel = k_ta0[index];
        assert_eq!(pixel, 0.005126953125);
    }

    #[test]
    fn k_v_pixels() {
        let e = eeprom();
        // Again, no difference between subpages here
        assert_eq!(e.k_v_pixels(Subpage::Zero), e.k_v_pixels(Subpage::One));
        let index = 11 * WIDTH + 15;
        assert_eq!(e.k_v_pixels(Subpage::Zero).nth(index).unwrap(), &0.5);
    }

    #[test]
    fn emissivity() {
        assert_eq!(eeprom().emissivity(), None);
    }

    #[test]
    fn offset_reference_cp() {
        let e = eeprom();
        assert_eq!(e.offset_reference_cp(Subpage::Zero), -75);
        assert_eq!(e.offset_reference_cp(Subpage::One), -77);
    }

    #[test]
    fn k_ta_cp() {
        let e = eeprom();
        // k_ta doesn't vary on subpage
        assert_eq!(e.k_ta_cp(Subpage::Zero), e.k_ta_cp(Subpage::One));
        assert_eq!(e.k_ta_cp(Subpage::Zero), 0.00457763671875);
    }

    #[test]
    fn k_v_cp() {
        let e = eeprom();
        // Also no subpage difference here
        assert_eq!(e.k_v_cp(Subpage::Zero), e.k_v_cp(Subpage::One));
        assert_eq!(e.k_v_cp(Subpage::Zero), 0.5);
    }

    #[test]
    fn temperature_gradient_coefficient() {
        assert_eq!(eeprom().temperature_gradient_coefficient(), Some(1f32));
    }

    #[test]
    fn alpha_cp() {
        let e = eeprom();
        assert_eq!(e.alpha_cp(Subpage::Zero), 4.07453626394272E-9);
        assert_eq!(e.alpha_cp(Subpage::One), 3.851710062200835E-9);
    }

    datasheet_test!(k_s_ta, -0.001953125);

    #[test]
    fn pixel_alpha() {
        let e = eeprom();
        let alpha0: ArrayVec<f32, NUM_PIXELS> = e.alpha_pixels(Subpage::One).copied().collect();
        let alpha1: ArrayVec<f32, NUM_PIXELS> = e.alpha_pixels(Subpage::Zero).copied().collect();
        // MLX90640 doesn't vary alpha on subpage
        assert_eq!(alpha0, alpha1);
        let index = 11 * WIDTH + 15;
        let pixel = alpha0[index];
        assert_eq!(pixel, 1.262233122690854E-7);
    }

    #[test]
    fn k_s_to() {
        let e = eeprom();
        assert_eq!(e.k_s_to()[1], -0.00080108642578125);
    }

    #[test]
    fn corner_temperatures() {
        let e = eeprom();
        let ct = e.corner_temperatures();
        // The first two values are hard-coded, but testing for completeness.
        assert_eq!(ct[0], -40);
        assert_eq!(ct[1], 0);
        // These are loaded from EEPROM
        assert_eq!(ct[2], 160);
        assert_eq!(ct[3], 320);
    }
}
