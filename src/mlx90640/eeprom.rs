// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::fmt::Debug;
use core::ops;
use core::slice;

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;

use crate::common::*;
use crate::error::{Error, LibraryError};
use crate::expose_member;
use crate::register::Subpage;
use crate::util::{i16_from_bits, Buffer};

use super::address::EepromAddress;
use super::{Mlx90640, NUM_PIXELS, WIDTH};

/// The number of corner temperatures an MLX90640 has.
const NUM_CORNER_TEMPERATURES: usize = 4;

/// The word size of the MLX90640 in terms of 8-bit bytes.
const WORD_SIZE: usize = 16 / 8;

/// MLX990640-specific calibration processing.
#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640Calibration<F> {
    k_v_dd: i16,

    v_dd_25: i16,

    resolution: u8,

    k_v_ptat: F,

    k_t_ptat: F,

    v_ptat_25: F,

    alpha_ptat: F,

    gain: F,

    k_s_ta: F,

    corner_temperatures: [i16; NUM_CORNER_TEMPERATURES],

    k_s_to: [F; NUM_CORNER_TEMPERATURES],

    alpha_correction: [F; NUM_CORNER_TEMPERATURES],

    alpha_pixels: [F; NUM_PIXELS],

    alpha_cp: [F; 2],

    offset_reference_pixels: [i16; NUM_PIXELS],

    offset_reference_cp: [i16; 2],

    k_v_pattern: [F; 4],

    k_v_cp: F,

    k_ta_pixels: [F; NUM_PIXELS],

    k_ta_cp: F,

    temperature_gradient_coefficient: Option<F>,
}

impl<F> Mlx90640Calibration<F> {
    /// Calculate pixel calibration values based off of the row and column data.
    ///
    /// Remainder data will be added in afterwards. This function is used for both offset and
    /// sensitivity (alpha) arrays. The given buffer must be at the word containing the column, row
    /// and remainder scaling factors.
    /// The calculated array, the remainder scaling factor, and the value occupying the 4 bits
    /// preceding the scaling factors are returned (in that order).
    fn calculate_bulk_pixel_calibration(data: &mut &[u8]) -> ([i16; NUM_PIXELS], u8, u8) {
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
    fn generate_k_ta_pixels(data: &mut &[u8]) -> impl Iterator<Item = i16> {
        let source_data: ArrayVec<i8, 4> = (0..4).map(|_| data.get_i8()).collect();
        let source_data = source_data.into_inner().unwrap();
        Self::repeat_chessboard(source_data)
    }
}

impl<F> Mlx90640Calibration<F>
where
    F: Debug + FloatConstants + From<i16> + From<u16> + From<i8> + From<u8>,
    F: ops::DivAssign<F> + ops::AddAssign<F>,
{
    /// Generate the constants needed for temperature calculations from a dump of the MLX90640
    /// EEPROM.
    ///
    /// The buffer must cover *all* of the EEPROM.
    pub fn from_data(data: &[u8]) -> Result<Self, LibraryError> {
        let mut buf = data;
        let eeprom_length = usize::from(EepromAddress::End - EepromAddress::Base);
        if buf.len() < eeprom_length {
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
        let alpha_pixels: ArrayVec<F, NUM_PIXELS> = core::array::IntoIter::new(alpha_pixels)
            .map(<F as From<i16>>::from)
            .collect();
        // Safe to unwrap as the length of pixel arrays are *all* NUM_PIXELS long.
        let mut alpha_pixels = alpha_pixels.into_inner().unwrap();
        // Calculate the actual alpha scaling value from the exponent value. The alpha scaling
        // exponenet also has 30 added to it (not 27 like alpha_scale_cp).
        let alpha_scale = <F as From<u8>>::from(alpha_scale_exp + 30).exp2();
        let gain = buf.get_i16();
        let v_ptat_25 = buf.get_i16();
        let (k_v_ptat, kt_ptat_bytes) = word_6_10_split(&mut buf);
        // k_v_ptat is scaled by 2^12
        let k_v_ptat = <F as From<i8>>::from(k_v_ptat) / <F as From<u16>>::from(4096);
        // k_t_ptat is scaled by 2^3
        let k_t_ptat =
            <F as From<i16>>::from(i16_from_bits(&kt_ptat_bytes, 10)) / <F as From<u8>>::from(8);
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
        let k_v_scale = <F as From<u8>>::from(unpacked_scales[1]).exp2();
        // k_ta_scale1 has 8 added to it.
        let k_ta_scale1 = <F as From<u8>>::from(unpacked_scales[2] + 8).exp2();
        // Leaving k_ta_scale2 as just the exponenet, as it's small enough to be used to shift it's
        // operand directly.
        let k_ta_scale2_exp = unpacked_scales[3];
        // We have k_v_scale now, calculate k_v_pattern
        let k_v_pattern: ArrayVec<F, 4> = core::array::IntoIter::new(k_v_avg)
            .map(|v| <F as From<i8>>::from(v) / k_v_scale)
            .collect();
        // Safe to unwrap as the input was only four elements, and the array is only 4 elements.
        let k_v_pattern = k_v_pattern.into_inner().unwrap();
        // Compensation pixel parameters
        let alpha_cp = {
            let (alpha_cp_ratio, alpha_cp_bytes) = word_6_10_split(&mut buf);
            let alpha_cp_ratio =
                <F as From<i8>>::from(alpha_cp_ratio) / <F as From<u8>>::from(7).exp2();
            // NOTE: the alpha scale value read from EEPROM has 27 added to get alpha_scale_cp, but
            // 30 added for alpha_scale_pixel
            let alpha_scale_cp = <F as From<u8>>::from(alpha_scale_exp + 27).exp2();
            let alpha_cp0 =
                <F as From<u16>>::from(u16::from_be_bytes(alpha_cp_bytes)) / alpha_scale_cp;
            [alpha_cp0, alpha_cp0 * (F::ONE + alpha_cp_ratio)]
        };
        let offset_reference_cp = {
            let (offset_cp_delta, offset_cp_bytes) = word_6_10_split(&mut buf);
            let offset_cp0 = i16_from_bits(&offset_cp_bytes, 10);
            [offset_cp0, offset_cp0 + i16::from(offset_cp_delta)]
        };
        let k_v_cp = <F as From<i8>>::from(buf.get_i8()) / k_v_scale;
        let k_ta_cp = <F as From<i8>>::from(buf.get_i8()) / k_ta_scale1;
        let k_s_ta = <F as From<i8>>::from(buf.get_i8()) / <F as From<u8>>::from(13).exp2();
        let temperature_gradient_coefficient = match buf.get_i8() {
            0 => None,
            n => Some(<F as From<i8>>::from(n) / <F as From<u8>>::from(5).exp2()),
        };
        // k_s_to is unscaled until k_s_to_scale is unpacked.
        let mut k_s_to_ranges: ArrayVec<F, 4> = (0..4)
            .map(|_| <F as From<i8>>::from(buf.get_i8()))
            .collect();
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
        let ct2 = i16::from(unpacked_corner_temps[2]) * corner_temperature_step;
        let ct3 = i16::from(unpacked_corner_temps[1]) * corner_temperature_step + ct2;
        // k_s_to_scale needs 8 added to it, then take 2 raised to this value.
        let k_s_to_scale = <F as From<u8>>::from(unpacked_corner_temps[3] + 8).exp2();
        // -40 and 0 are hard-coded values for CT0 and CT1 (labelled CT1 and CT2 in the datasheet)
        let corner_temperatures = [-40i16, 0, ct2, ct3];
        // Now that we have k_s_to_scale, we can scale k_s_to properly:
        k_s_to.iter_mut().for_each(|k_s_to| *k_s_to /= k_s_to_scale);
        let basic_range =
            <<Self as CalibrationData<F>>::Camera as MelexisCamera<F>>::BASIC_TEMPERATURE_RANGE;
        let alpha_correction =
            alpha_correction_coefficients(basic_range, &corner_temperatures, &k_s_to);
        // Calculate the rest of the per-pixel data using the remainder/k_ta data
        let mut k_ta_pixels = [F::ZERO; NUM_PIXELS];
        offset_reference_pixels
            .iter_mut()
            .zip(alpha_pixels.iter_mut())
            .zip(lazy_k_ta_pixels)
            .zip(k_ta_pixels.iter_mut())
            .for_each(|(((offset, alpha), k_ta_rc), k_ta)| {
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
                *alpha +=
                    <F as From<i16>>::from(alpha_remainder << alpha_correction_remainder_scale);
                *alpha /= alpha_scale;
                // To try to keep floating point errors down as long as possible, do all the
                // operations for the numerator as ints, then convert to floats for the final division.
                let k_ta_remainder = i16::from(i8::from_ne_bytes([low & 0x0E]) << 4 >> 5);
                let k_ta_numerator =
                    <F as From<i16>>::from(k_ta_rc + (k_ta_remainder << k_ta_scale2_exp));
                *k_ta = k_ta_numerator / k_ta_scale1;
            });
        Ok(Self {
            k_v_dd,
            v_dd_25,
            resolution,
            k_v_ptat,
            k_t_ptat,
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

impl<I2C, F> FromI2C<I2C> for Mlx90640Calibration<F>
where
    I2C: i2c::WriteRead + i2c::Write,
    F: Debug + FloatConstants + From<i16> + From<u16> + From<i8> + From<u8>,
    F: ops::DivAssign<F> + ops::AddAssign<F>,
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

impl<'a, F> CalibrationData<'a, F> for Mlx90640Calibration<F>
where
    F: 'a + FloatConstants + From<i8>,
{
    type Camera = Mlx90640;

    expose_member!(k_v_dd, i16);
    expose_member!(v_dd_25, i16);
    expose_member!(resolution, u8);
    expose_member!(k_v_ptat, F);
    expose_member!(k_t_ptat, F);
    expose_member!(v_ptat_25, F);
    expose_member!(alpha_ptat, F);
    expose_member!(gain, F);
    expose_member!(k_s_ta, F);

    expose_member!(&corner_temperatures, [i16]);
    expose_member!(&k_s_to, [F]);
    expose_member!(&alpha_correction, [F]);

    type OffsetReferenceIterator = slice::Iter<'a, i16>;

    fn offset_reference_pixels(&'a self, _subpage: Subpage) -> Self::OffsetReferenceIterator {
        self.offset_reference_pixels.iter()
    }

    fn offset_reference_cp(&self, subpage: Subpage) -> i16 {
        self.offset_reference_cp[subpage as usize]
    }

    type AlphaIterator = slice::Iter<'a, F>;

    fn alpha_pixels(&'a self, _subpage: Subpage) -> Self::AlphaIterator {
        self.alpha_pixels.iter()
    }

    fn alpha_cp(&self, subpage: Subpage) -> F {
        self.alpha_cp[subpage as usize]
    }

    type KvIterator = ChessboardIter<'a, F>;

    fn k_v_pixels(&'a self, _subpage: Subpage) -> Self::KvIterator {
        ChessboardIter::new(&self.k_v_pattern)
    }

    fn k_v_cp(&self, _subpage: Subpage) -> F {
        self.k_v_cp
    }

    type KtaIterator = slice::Iter<'a, F>;

    fn k_ta_pixels(&'a self, _subpage: Subpage) -> Self::KtaIterator {
        self.k_ta_pixels.iter()
    }

    fn k_ta_cp(&self, _subpage: Subpage) -> F {
        self.k_ta_cp
    }

    expose_member!(temperature_gradient_coefficient, Option<F>);
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
fn word_6_10_split(data: &mut &[u8]) -> (i8, [u8; 2]) {
    let mut word = [data.get_u8(), data.get_u8()];
    // Copy out the 6-bit value, and shift it over. As signed right shifts are aritmetic, the sign
    // bit gets extended, and we get the value we wanted.
    let six_bit = i8::from_ne_bytes([word[0]]) >> 2;
    // Mask off the extra from the high byte
    word[0] &= 0x03;
    (six_bit, word)
}

/// Extract four unsigned, 4-bit integers from a buffer
fn word_to_u4s(data: &mut &[u8]) -> [u8; 4] {
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
fn word_to_i4s(data: &mut &[u8]) -> [i8; 4] {
    let high = data.get_u8();
    let low = data.get_u8();
    let high = u8_to_i4s(high);
    let low = u8_to_i4s(low);
    [high[0], high[1], low[0], low[1]]
}

#[cfg(test)]
#[generic_tests::define]
pub(crate) mod test {
    #[cfg(feature = "std")]
    extern crate std;
    #[cfg(feature = "std")]
    use std::{print, println};

    use core::fmt::Debug;

    use arrayvec::ArrayVec;
    use float_cmp::{assert_approx_eq, ApproxEq};
    use num_traits::NumCast;

    use crate::common::{CalibrationData, FloatConstants};
    use crate::mlx90640::{HEIGHT, NUM_PIXELS, WIDTH};
    use crate::register::Subpage;
    use crate::test::{mlx90640_datasheet_eeprom, mlx90640_example_data};

    use super::Mlx90640Calibration;

    fn datasheet_eeprom<F>() -> Mlx90640Calibration<F>
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let mut eeprom_bytes = mlx90640_datasheet_eeprom();
        Mlx90640Calibration::from_data(&mut eeprom_bytes).expect("The EEPROM data to be parsed.")
    }

    fn example_eeprom<F>() -> Mlx90640Calibration<F>
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let mut example_bytes = &mlx90640_example_data::EEPROM_DATA[..];
        Mlx90640Calibration::from_data(&mut example_bytes)
            .expect("The example data should be parseable")
    }

    #[test]
    fn word_6_10_split<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
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
    fn word_to_u4s<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let mut sequence: &[u8] = b"\x12\x34";
        assert_eq!(super::word_to_u4s(&mut sequence), [1, 2, 3, 4]);
        let mut max_min: &[u8] = b"\xf0\xf0";
        assert_eq!(super::word_to_u4s(&mut max_min), [0xF, 0, 0xF, 0]);
        let mut min_max: &[u8] = b"\x0f\x0f";
        assert_eq!(super::word_to_u4s(&mut min_max), [0, 0xF, 0, 0xF]);
    }

    #[test]
    fn u8_to_i4s<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(super::u8_to_i4s(0x44), [4, 4]);
        assert_eq!(super::u8_to_i4s(0x88), [-8, -8]);
        assert_eq!(super::u8_to_i4s(0x48), [4, -8]);
        assert_eq!(super::u8_to_i4s(0x84), [-8, 4]);
    }

    #[test]
    fn repeat_chessboard<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // The pattern order is (for row, column): EE, OE, EO, OO
        let pattern = [1, 2, 3, 4];
        let test_pattern: ArrayVec<i8, NUM_PIXELS> =
            Mlx90640Calibration::<()>::repeat_chessboard(pattern).collect();
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
    fn smoke<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        datasheet_eeprom::<F>();
        example_eeprom::<F>();
    }

    // Ordering these tests in the same order as the data sheet's worked example.
    #[test]
    fn resolution<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(datasheet_eeprom::<F>().resolution(), 2);
        assert_eq!(
            example_eeprom::<F>().resolution(),
            mlx90640_example_data::RESOLUTION_EE
        );
    }

    #[test]
    fn k_v_dd<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(datasheet_eeprom::<F>().k_v_dd(), -3168);
        assert_eq!(
            example_eeprom::<F>().k_v_dd(),
            mlx90640_example_data::K_V_DD
        );
    }

    #[test]
    fn v_dd_25<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(datasheet_eeprom::<F>().v_dd_25(), -13056);
        assert_eq!(
            example_eeprom::<F>().v_dd_25(),
            mlx90640_example_data::V_DD_25
        );
    }

    #[test]
    fn v_dd_0<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(datasheet_eeprom::<F>().v_dd_0(), F::THREE_POINT_THREE);
        assert_eq!(example_eeprom::<F>().v_dd_0(), F::THREE_POINT_THREE);
    }

    #[test]
    fn k_v_ptat<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_approx_eq!(
            F,
            datasheet_eeprom::<F>().k_v_ptat(),
            <F as NumCast>::from(0.0053710938f64).unwrap()
        );
        assert_approx_eq!(
            F,
            example_eeprom::<F>().k_v_ptat(),
            <F as NumCast>::from(mlx90640_example_data::K_V_PTAT).unwrap()
        );
    }

    #[test]
    fn k_t_ptat<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().k_t_ptat(),
            <F as NumCast>::from(42.25f64).unwrap()
        );
        assert_eq!(
            example_eeprom::<F>().k_t_ptat(),
            <F as NumCast>::from(mlx90640_example_data::K_T_PTAT).unwrap()
        );
    }

    #[test]
    fn v_ptat_25<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().v_ptat_25(),
            <F as NumCast>::from(12273f64).unwrap()
        );
        assert_eq!(
            example_eeprom::<F>().v_ptat_25(),
            <F as NumCast>::from(mlx90640_example_data::V_PTAT_25).unwrap()
        );
    }

    #[test]
    fn alpha_ptat<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().alpha_ptat(),
            <F as NumCast>::from(9f64).unwrap()
        );
        assert_eq!(
            example_eeprom::<F>().alpha_ptat(),
            <F as NumCast>::from(mlx90640_example_data::ALPHA_PTAT).unwrap()
        );
    }

    #[test]
    fn gain<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().gain(),
            <F as NumCast>::from(6383.0f64).unwrap()
        );
        assert_eq!(
            example_eeprom::<F>().gain(),
            <F as NumCast>::from(mlx90640_example_data::GAIN_EE).unwrap()
        );
    }

    fn test_pixels_common<T: PartialEq + Debug + Copy>(
        datasheet_data: [ArrayVec<T, NUM_PIXELS>; 2],
        example_data: [ArrayVec<T, NUM_PIXELS>; 2],
        datasheet_expected: T,
        example_expected: &[T; NUM_PIXELS],
        subpage: Option<Subpage>,
        check: &dyn Fn(T, T) -> bool,
    ) {
        if subpage.is_none() {
            assert_eq!(datasheet_data[0], datasheet_data[1]);
            assert_eq!(example_data[0], example_data[1]);
        }
        // Test the single pixel from the datasheet
        let datasheet_index = 11 * WIDTH + 15;
        let subpage_index: usize = subpage.unwrap_or(Subpage::Zero).into();
        let pixel = datasheet_data[subpage_index][datasheet_index];
        assert!(check(pixel, datasheet_expected));
        // Check all the pixels from the full example
        let offset_pairs = example_data[subpage_index]
            .iter()
            .zip(example_expected.iter());
        for (index, (actual, expected)) in offset_pairs.enumerate() {
            assert!(
                check(*actual, *expected),
                "[pixel {:?}]: Expected {:?}, Actual: {:?}",
                index,
                expected,
                actual
            );
        }
    }

    fn test_pixels_approx<T>(
        datasheet_data: [ArrayVec<T, NUM_PIXELS>; 2],
        example_data: [ArrayVec<T, NUM_PIXELS>; 2],
        datasheet_expected: T,
        example_expected: &[T; NUM_PIXELS],
        subpage: Option<Subpage>,
    ) where
        T: ApproxEq + PartialEq + Debug + Copy,
    {
        let check = |actual: T, expected: T| actual.approx_eq(expected, T::Margin::default());
        test_pixels_common(
            datasheet_data,
            example_data,
            datasheet_expected,
            example_expected,
            subpage,
            &check,
        );
    }

    fn test_pixels<T: PartialEq + core::fmt::Debug + core::fmt::Display + Copy>(
        datasheet_data: [ArrayVec<T, NUM_PIXELS>; 2],
        example_data: [ArrayVec<T, NUM_PIXELS>; 2],
        datasheet_expected: T,
        example_expected: &[T; NUM_PIXELS],
        subpage: Option<Subpage>,
    ) {
        let check = |actual: T, expected: T| actual == expected;
        test_pixels_common(
            datasheet_data,
            example_data,
            datasheet_expected,
            example_expected,
            subpage,
            &check,
        );
    }

    #[test]
    fn pixel_offset<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let datasheet = datasheet_eeprom::<F>();
        let datasheet_offsets: [ArrayVec<i16, NUM_PIXELS>; 2] = [
            datasheet
                .offset_reference_pixels(Subpage::Zero)
                .copied()
                .collect(),
            datasheet
                .offset_reference_pixels(Subpage::One)
                .copied()
                .collect(),
        ];
        let example = example_eeprom::<F>();
        let example_offsets: [ArrayVec<i16, NUM_PIXELS>; 2] = [
            example
                .offset_reference_pixels(Subpage::Zero)
                .copied()
                .collect(),
            example
                .offset_reference_pixels(Subpage::One)
                .copied()
                .collect(),
        ];
        test_pixels(
            datasheet_offsets,
            example_offsets,
            -75,
            &mlx90640_example_data::OFFSET_REFERENCE_PIXELS,
            // MLX90640 doesn't vary offsets on subpage
            None,
        );
    }

    #[test]
    fn pixel_k_ta<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let datasheet = datasheet_eeprom::<F>();
        let datasheet_k_ta: [ArrayVec<F, NUM_PIXELS>; 2] = [
            datasheet.k_ta_pixels(Subpage::Zero).copied().collect(),
            datasheet.k_ta_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom::<F>();
        let example_k_ta: [ArrayVec<F, NUM_PIXELS>; 2] = [
            example.k_ta_pixels(Subpage::Zero).copied().collect(),
            example.k_ta_pixels(Subpage::One).copied().collect(),
        ];
        let expected_example_pixels: [F; NUM_PIXELS] = mlx90640_example_data::K_TA_PIXELS
            .iter()
            .map(|n| <F as NumCast>::from(*n).unwrap())
            .collect::<ArrayVec<F, NUM_PIXELS>>()
            .into_inner()
            .unwrap();
        test_pixels_approx(
            datasheet_k_ta,
            example_k_ta,
            <F as NumCast>::from(0.005126953125f64).unwrap(),
            &expected_example_pixels,
            // MLX90640 doesn't vary k_ta on subpage
            None,
        );
    }

    #[test]
    fn k_v_pixels<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let datasheet = datasheet_eeprom::<F>();
        let datasheet_k_v: [ArrayVec<F, NUM_PIXELS>; 2] = [
            datasheet.k_v_pixels(Subpage::Zero).copied().collect(),
            datasheet.k_v_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom::<F>();
        let example_k_v: [ArrayVec<F, NUM_PIXELS>; 2] = [
            example.k_v_pixels(Subpage::Zero).copied().collect(),
            example.k_v_pixels(Subpage::One).copied().collect(),
        ];
        let expected_example_pixels: [F; NUM_PIXELS] = mlx90640_example_data::K_V_PIXELS
            .iter()
            .map(|n| <F as NumCast>::from(*n).unwrap())
            .collect::<ArrayVec<F, NUM_PIXELS>>()
            .into_inner()
            .unwrap();
        test_pixels_approx(
            datasheet_k_v,
            example_k_v,
            <F as NumCast>::from(0.5f64).unwrap(),
            &expected_example_pixels,
            // MLX90640 doesn't vary k_v on subpage
            None,
        );
    }

    #[test]
    fn emissivity<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(datasheet_eeprom::<F>().emissivity(), None);
        assert_eq!(example_eeprom::<F>().emissivity(), None);
    }

    #[test]
    fn offset_reference_cp<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // datasheet
        let datasheet = datasheet_eeprom::<F>();
        assert_eq!(datasheet.offset_reference_cp(Subpage::Zero), -75);
        assert_eq!(datasheet.offset_reference_cp(Subpage::One), -77);
        // example
        let example = example_eeprom::<F>();
        assert_eq!(
            example.offset_reference_cp(Subpage::Zero),
            mlx90640_example_data::OFFSET_CP[0],
        );
        assert_eq!(
            example.offset_reference_cp(Subpage::One),
            mlx90640_example_data::OFFSET_CP[1],
        );
    }

    #[test]
    fn k_ta_cp<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // datasheet
        let datasheet = datasheet_eeprom::<F>();
        let expected: F = <F as NumCast>::from(0.00457763671875f64).unwrap();
        assert_eq!(datasheet.k_ta_cp(Subpage::Zero), expected);
        assert_eq!(datasheet.k_ta_cp(Subpage::One), expected);
        // example
        let example = example_eeprom::<F>();
        assert_approx_eq!(
            F,
            example.k_ta_cp(Subpage::Zero),
            <F as NumCast>::from(mlx90640_example_data::K_TA_CP).unwrap()
        );
        assert_approx_eq!(
            F,
            example.k_ta_cp(Subpage::One),
            <F as NumCast>::from(mlx90640_example_data::K_TA_CP).unwrap()
        );
    }

    #[test]
    fn k_v_cp<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // datasheet
        let datasheet = datasheet_eeprom::<F>();
        let expected = <F as NumCast>::from(0.5f64).unwrap();
        assert_eq!(datasheet.k_v_cp(Subpage::Zero), expected);
        assert_eq!(datasheet.k_v_cp(Subpage::One), expected);
        // example
        let example = example_eeprom::<F>();
        assert_eq!(
            example.k_v_cp(Subpage::Zero),
            <F as NumCast>::from(mlx90640_example_data::K_V_CP).unwrap()
        );
        assert_eq!(
            example.k_v_cp(Subpage::One),
            <F as NumCast>::from(mlx90640_example_data::K_V_CP).unwrap()
        );
    }

    #[test]
    fn temperature_gradient_coefficient<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().temperature_gradient_coefficient(),
            Some(F::ONE)
        );
        assert_eq!(
            example_eeprom::<F>().temperature_gradient_coefficient(),
            // 0 TGC means None
            None,
        );
    }

    #[test]
    fn alpha_cp<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // datasheet
        let datasheet = datasheet_eeprom::<F>();
        assert_eq!(
            datasheet.alpha_cp(Subpage::Zero),
            <F as NumCast>::from(4.07453626394272E-9f64).unwrap()
        );
        assert_eq!(
            datasheet.alpha_cp(Subpage::One),
            <F as NumCast>::from(3.851710062200835E-9f64).unwrap()
        );
        // example
        let example = example_eeprom::<F>();
        assert_eq!(
            example.alpha_cp(Subpage::Zero),
            <F as NumCast>::from(mlx90640_example_data::ALPHA_CP[0]).unwrap()
        );
        assert_eq!(
            example.alpha_cp(Subpage::One),
            <F as NumCast>::from(mlx90640_example_data::ALPHA_CP[1]).unwrap()
        );
    }

    #[test]
    fn k_s_ta<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().k_s_ta(),
            <F as NumCast>::from(-0.001953125f64).unwrap()
        );
        assert_approx_eq!(
            F,
            example_eeprom::<F>().k_s_ta(),
            <F as NumCast>::from(mlx90640_example_data::K_S_TA).unwrap()
        );
    }

    #[test]
    fn pixel_alpha<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        // MLX90640 doesn't vary alpha on subpage
        let datasheet = datasheet_eeprom::<F>();
        let datasheet_alpha: [ArrayVec<F, NUM_PIXELS>; 2] = [
            datasheet.alpha_pixels(Subpage::Zero).copied().collect(),
            datasheet.alpha_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom::<F>();
        let example_alpha: [ArrayVec<F, NUM_PIXELS>; 2] = [
            example.alpha_pixels(Subpage::Zero).copied().collect(),
            example.alpha_pixels(Subpage::One).copied().collect(),
        ];
        let expected_example_alpha_pixels: [F; NUM_PIXELS] = mlx90640_example_data::ALPHA_PIXELS
            .iter()
            .map(|n| <F as NumCast>::from(*n).unwrap())
            .collect::<ArrayVec<F, NUM_PIXELS>>()
            .into_inner()
            .unwrap();
        test_pixels_approx(
            datasheet_alpha,
            example_alpha,
            <F as NumCast>::from(1.262233122690854E-7f64).unwrap(),
            &expected_example_alpha_pixels,
            None,
        );
    }

    #[test]
    fn k_s_to<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        assert_eq!(
            datasheet_eeprom::<F>().k_s_to()[1],
            <F as NumCast>::from(-0.00080108642578125).unwrap()
        );
        let example = example_eeprom::<F>();
        let expected_example = mlx90640_example_data::K_S_TO
            .iter()
            .map(|n| <F as NumCast>::from(*n).unwrap());
        let example_pairs = example.k_s_to().iter().zip(expected_example);
        for (actual, expected) in example_pairs {
            assert_approx_eq!(F, *actual, expected);
        }
    }

    #[test]
    fn corner_temperatures<F>()
    where
        F: Debug
            + FloatConstants
            + ApproxEq
            + From<f32>
            + From<i16>
            + From<u16>
            + From<i8>
            + From<u8>,
    {
        let e = datasheet_eeprom::<F>();
        let ct = e.corner_temperatures();
        // The first two values are hard-coded, but testing for completeness.
        assert_eq!(ct[0], -40);
        assert_eq!(ct[1], 0);
        // These are loaded from EEPROM
        assert_eq!(ct[2], 160);
        assert_eq!(ct[3], 320);

        // Full example test
        assert_eq!(
            example_eeprom::<F>().corner_temperatures(),
            mlx90640_example_data::CORNER_TEMPERATURES
        );
    }

    #[instantiate_tests(<f32>)]
    mod f32 {}

    #[instantiate_tests(<f64>)]
    mod f64 {}
}
