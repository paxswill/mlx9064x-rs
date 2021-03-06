// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::iter;
use core::slice;

use arrayvec::ArrayVec;
use bitvec::array::BitArray;
use bitvec::slice::BitSlice;
use embedded_hal::blocking::i2c;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use crate::common::*;
use crate::error::{Error, LibraryError};
use crate::expose_member;
use crate::register::{AccessPattern, Resolution, Subpage};
use crate::util::{i16_from_bits, is_bit_set, Buffer, WORD_SIZE};

use super::address::EepromAddress;
use super::Mlx90640;

/// The number of corner temperatures an MLX90640 has.
const NUM_CORNER_TEMPERATURES: usize = 4;

type FlagSlice = BitSlice<usize>;
type FlagArray = BitArray<[usize; Mlx90640::NUM_PIXELS / usize::BITS as usize]>;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90640Calibration {
    k_v_dd: i16,

    v_dd_25: i16,

    resolution: Resolution,

    k_v_ptat: f32,

    k_t_ptat: f32,

    v_ptat_25: f32,

    alpha_ptat: f32,

    gain: f32,

    k_s_ta: f32,

    corner_temperatures: [i16; NUM_CORNER_TEMPERATURES],

    k_s_to: [f32; NUM_CORNER_TEMPERATURES],

    alpha_correction: [f32; NUM_CORNER_TEMPERATURES],

    alpha_pixels: [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],

    alpha_cp: [f32; 2],

    offset_reference_pixels: [i16; <Self as CalibrationData>::Camera::NUM_PIXELS],

    offset_reference_cp: [i16; 2],

    k_v_pattern: [f32; 4],

    k_v_cp: f32,

    k_ta_pixels: [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],

    k_ta_cp: f32,

    temperature_gradient_coefficient: Option<f32>,

    interleave_correction_pixels: [f32; 6],

    interleave_correction_cp: f32,

    failed_pixels: FlagArray,

    outlier_pixels: FlagArray,
}

impl Mlx90640Calibration {
    /// Calculate pixel calibration values based off of the row and column data.
    ///
    /// Remainder data will be added in afterwards. This function is used for both offset and
    /// sensitivity (alpha) arrays. The given buffer must be at the word containing the column, row
    /// and remainder scaling factors.
    /// The calculated array, the remainder scaling factor, and the value occupying the 4 bits
    /// preceding the scaling factors are returned (in that order).
    fn calculate_bulk_pixel_calibration(
        data: &mut &[u8],
    ) -> ([i16; <Self as CalibrationData>::Camera::NUM_PIXELS], u8, u8) {
        let (extra_value, row_scale, column_scale, remainder_scale) = {
            let scales = word_to_u4s(data);
            (scales[0], scales[1], scales[2], scales[3])
        };
        let offset_average = data.get_i16();
        let mut pixel_calibration = [offset_average; Mlx90640::NUM_PIXELS];
        const VALUES_PER_DATA_ROW: usize = 4;
        // Add row offsets
        for row_chunks in pixel_calibration.chunks_exact_mut(Mlx90640::WIDTH * VALUES_PER_DATA_ROW)
        {
            let rows_coefficients = word_to_i4s(data);
            // Create a nice lazy iterator that converts the values to i16, scales them, and
            // reverses that order of the data (because the data is laid out backwards in the
            // EEPROM).
            let rows_coefficients = rows_coefficients
                .into_iter()
                //let rows_coefficients = core::array::IntoIter::new(rows_coefficients)
                .map(i16::from)
                .map(|coeff| coeff << row_scale)
                .rev();
            for (row, coefficient) in row_chunks
                .chunks_exact_mut(Mlx90640::WIDTH)
                .zip(rows_coefficients)
            {
                row.iter_mut().for_each(|element| *element += coefficient);
            }
        }
        // Add column offsets. Slightly more involved as the offsets are in row-major order.
        for column_chunk_index in 0..(Mlx90640::WIDTH / VALUES_PER_DATA_ROW) {
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
            for row in pixel_calibration.chunks_exact_mut(Mlx90640::WIDTH) {
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
        let even_row_pattern = [row_even_col_even, row_even_col_odd].into_iter();
        let odd_row_pattern = [row_odd_col_even, row_odd_col_odd].into_iter();
        // Repeat the pattern across the row
        let even_row = even_row_pattern.cycle().take(Mlx90640::WIDTH).map(T::from);
        let odd_row = odd_row_pattern.cycle().take(Mlx90640::WIDTH).map(T::from);
        // Then chain the two rows together, repeating to fill the array
        let repeating_rows = even_row.chain(odd_row).cycle();
        repeating_rows.take(Mlx90640::NUM_PIXELS)
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

    /// Read and convert the chessboard to interlaced correction parameters
    ///
    /// There are three correction constants. The first is used for the compensation pixels, and
    /// the last two are used to create a pattern of correction values used when calculating the
    /// pixel offsets.
    fn get_access_pattern_corrections(data: &mut &[u8]) -> (f32, [f32; 6]) {
        let word = data.get_u16();
        // The correction factors are 5 bits, 5 bits, then 6 bits, for factors 3, 2, 1.
        let chess_1_raw = i16_from_bits(&(word & 0x003F).to_be_bytes(), 6);
        let chess_2_raw = i16_from_bits(&((word & 0x07C0) >> 6).to_be_bytes(), 5);
        let chess_3_raw = i16_from_bits(&((word & 0xF800) >> 11).to_be_bytes(), 5);
        // factor 1 is scaled by 2^4
        let chess_1 = f32::from(chess_1_raw) / 16f32;
        // factor 2 is scaled by 2 (no exponent)
        let chess_2 = f32::from(chess_2_raw) / 2f32;
        // factor 3 is scaled by 2^3
        let chess_3 = f32::from(chess_3_raw) / 8f32;
        // See the docstring for InterleaveCorrectionIter for the explanation for this pattern
        let b = chess_2 - chess_3;
        let c = chess_2 + chess_3;
        let pattern = [-chess_3, b, -c, chess_3, -b, c];
        (chess_1, pattern)
    }

    /// Extract the temperature sensitivity range values.
    fn get_temperature_range_data(
        buf: &mut &[u8],
    ) -> (
        [i16; NUM_CORNER_TEMPERATURES],
        [f32; NUM_CORNER_TEMPERATURES],
    ) {
        // k_s_to is unscaled until k_s_to_scale is unpacked.
        let mut k_s_to_ranges: ArrayVec<f32, 4> = (0..4).map(|_| f32::from(buf.get_i8())).collect();
        // Fix the ordering of the elements from the EEPROM
        k_s_to_ranges.swap(0, 1);
        k_s_to_ranges.swap(2, 3);
        // Safe to unwrap as I'm just using ArrayVec to collect into an array.
        let mut k_s_to = k_s_to_ranges.into_inner().unwrap();
        // Very similar to the resolution and k_*_scale word a few lines above.
        let unpacked_corner_temps = word_to_u4s(buf);
        // Like before, the top two bits are reserved. This time though, the temperature step
        // is multipled by 10. Also convert to i16 for use in calculations.
        let corner_temperature_step = i16::from(unpacked_corner_temps[0] & 0x3) * 10;
        // corner temperatures need to be multipled by the step and converted to i16
        let ct2 = i16::from(unpacked_corner_temps[2]) * corner_temperature_step;
        let ct3 = i16::from(unpacked_corner_temps[1]) * corner_temperature_step + ct2;
        // k_s_to_scale needs 8 added to it, then take 2 raised to this value.
        let k_s_to_scale = f32::from(unpacked_corner_temps[3] + 8).exp2();
        // -40 and 0 are hard-coded values for CT0 and CT1 (labelled CT1 and CT2 in the datasheet)
        let corner_temperatures = [-40i16, 0, ct2, ct3];
        // Now that we have k_s_to_scale, we can scale k_s_to properly:
        k_s_to.iter_mut().for_each(|k_s_to| *k_s_to /= k_s_to_scale);
        (corner_temperatures, k_s_to)
    }

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
        let alpha_pixels: ArrayVec<f32, { Mlx90640::NUM_PIXELS }> =
            alpha_pixels.into_iter().map(f32::from).collect();
        // Safe to unwrap as the length of pixel arrays are *all* Mlx90640::NUM_PIXELS long.
        let mut alpha_pixels = alpha_pixels.into_inner().unwrap();
        // Calculate the actual alpha scaling value from the exponent value. The alpha scaling
        // exponenet also has 30 added to it (not 27 like alpha_scale_cp).
        let alpha_scale = f32::from(alpha_scale_exp + 30).exp2();
        let gain = buf.get_i16();
        let v_ptat_25 = buf.get_i16();
        let (k_v_ptat, kt_ptat_bytes) = word_6_10_split(&mut buf);
        // k_v_ptat is scaled by 2^12
        let k_v_ptat = f32::from(k_v_ptat) / 4096f32;
        // k_t_ptat is scaled by 2^3
        let k_t_ptat = f32::from(i16_from_bits(&kt_ptat_bytes, 10)) / 8f32;
        let k_v_dd = (buf.get_i8() as i16) << 5;
        // The data in EEPROM is unsigned, so we upgrade to a signed type as it's immediately sent
        // negative (by subtracting 256), then multipled by 2^5, and finally has 2^13 subtracted
        // from it.
        let v_dd_25 = ((buf.get_u8() as i16) - 256) * (1 << 5) - (1 << 13);
        // Keep this value around for actual processing once we have kv_scale.
        let k_v_avg = word_to_i4s(&mut buf);
        let (interleave_correction_cp, interleave_correction_pixels) =
            Self::get_access_pattern_corrections(&mut buf);
        let lazy_k_ta_pixels = Self::generate_k_ta_pixels(&mut buf);
        let unpacked_scales = word_to_u4s(&mut buf);
        // The resolution control calibration value is just two bits in the high half of the byte.
        // The two other two bits are reserved, so we just drop them.
        let resolution_byte = unpacked_scales[0] & 0x3;
        let resolution = Resolution::from_raw(resolution_byte as u16)?;
        // various scaling constants
        let k_v_scale = f32::from(unpacked_scales[1]).exp2();
        // k_ta_scale1 has 8 added to it.
        let k_ta_scale1 = f32::from(unpacked_scales[2] + 8).exp2();
        // Leaving k_ta_scale2 as just the exponenet, as it's small enough to be used to shift it's
        // operand directly.
        let k_ta_scale2_exp = unpacked_scales[3];
        // We have k_v_scale now, calculate k_v_pattern
        let k_v_pattern: ArrayVec<f32, 4> = k_v_avg
            .into_iter()
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
        let (corner_temperatures, k_s_to) = Self::get_temperature_range_data(&mut buf);
        let basic_range = <Self as CalibrationData>::Camera::BASIC_TEMPERATURE_RANGE;
        let alpha_correction =
            alpha_correction_coefficients(basic_range, &corner_temperatures, &k_s_to);
        let mut failed_pixels = FlagArray::default();
        let mut outlier_pixels = FlagArray::default();
        // Calculate the rest of the per-pixel data using the remainder/k_ta data
        let mut k_ta_pixels = [0f32; Mlx90640::NUM_PIXELS];
        offset_reference_pixels
            .iter_mut()
            .zip(alpha_pixels.iter_mut())
            .zip(lazy_k_ta_pixels)
            .zip(k_ta_pixels.iter_mut())
            .zip(failed_pixels.iter_mut())
            .zip(outlier_pixels.iter_mut())
            .for_each(
                |(((((offset, alpha), k_ta_rc), k_ta), mut failed), mut outlier)| {
                    let high = buf.get_u8();
                    let low = buf.get_u8();
                    // If both bytes are 0, this is a failed pixel
                    if high == 0 && low == 0 {
                        failed.set(true);
                    } else {
                        // Normal dance to extend the sign bit from an i6 to an i8
                        let offset_remainder = i16::from(i8::from_ne_bytes([high & 0xFC]) >> 2);
                        *offset += offset_remainder << offset_correction_remainder_scale;
                        // alpha is going to be a little weird: not only is there the i6-shift-dance,
                        // but there's an extra shift right by 4 to drop the k_ta and outlier bits.
                        let alpha_remainder = (i16::from_be_bytes([high & 0x03, low]) << 6) >> 10;
                        *alpha += f32::from(alpha_remainder << alpha_correction_remainder_scale);
                        *alpha /= alpha_scale;
                        // To try to keep floating point errors down as long as possible, do all the
                        // operations for the numerator as ints, then convert to floats for the final
                        // division.
                        let k_ta_remainder = i16::from(i8::from_ne_bytes([low & 0x0E]) << 4 >> 5);
                        let k_ta_numerator =
                            f32::from(k_ta_rc + (k_ta_remainder << k_ta_scale2_exp));
                        *k_ta = k_ta_numerator / k_ta_scale1;
                        // If the last bit of the low byte is set, this is an outlier (or deviant)
                        // pixel.
                        outlier.set(is_bit_set(low, 0))
                    }
                },
            );
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
            interleave_correction_pixels,
            interleave_correction_cp,
            failed_pixels,
            outlier_pixels,
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
    type Camera = Mlx90640;

    expose_member!(k_v_dd, i16);
    expose_member!(v_dd_25, i16);
    expose_member!(resolution, Resolution);
    expose_member!(k_v_ptat, f32);
    expose_member!(k_t_ptat, f32);
    expose_member!(v_ptat_25, f32);
    expose_member!(alpha_ptat, f32);
    expose_member!(gain, f32);
    expose_member!(k_s_ta, f32);

    expose_member!(&corner_temperatures, [i16]);
    expose_member!(&k_s_to, [f32]);
    expose_member!(&alpha_correction, [f32]);

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

    type AccessPatternCompensation = PixelAccessPatternCompensation<'a>;

    /// A sequence of per-pixel correction values that are added to the pixl gain value.
    ///
    /// The MLX90640 can be used in interleaved mode, but for optimal performance a correction
    /// needs to be applied. This value is summed with the pixel gain value and reference offset
    /// (the reference offset being scaled relative to the temperature difference).
    fn access_pattern_compensation_pixels(
        &'a self,
        access_pattern: AccessPattern,
    ) -> Self::AccessPatternCompensation {
        PixelAccessPatternCompensation::new(access_pattern, &self.interleave_correction_pixels[..])
    }

    /// Equivalent to [`Self::access_pattern_compensation_pixels`] for compensation pixels.
    fn access_pattern_compensation_cp(
        &self,
        subpage: Subpage,
        access_pattern: AccessPattern,
    ) -> Option<f32> {
        match (subpage, access_pattern) {
            (Subpage::One, AccessPattern::Interleave) => Some(self.interleave_correction_cp),
            _ => None,
        }
    }

    type FailedPixels = &'a FlagSlice;

    fn failed_pixels(&'a self) -> Self::FailedPixels {
        &self.failed_pixels
    }

    type OutlierPixels = &'a FlagSlice;

    fn outlier_pixels(&'a self) -> Self::OutlierPixels {
        &self.outlier_pixels
    }
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
        if self.index < Mlx90640::NUM_PIXELS {
            let row = self.index / Mlx90640::WIDTH;
            let column = self.index % Mlx90640::WIDTH;
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

/// An iterator for the interleaved reading pattern values
///
#[doc = include_str!("../katex.html")]
///
/// These values are only used if the interleaved access mode is being active. It's described in
/// section 11.1.3.1 of the MLX90640 datasheet, but the formula is overly complicated:
///
/// $$
/// \begin{split}
/// IL\_{\textit{Pattern}} =& {} int\left(\frac{\mathtt{pixel\\_index} - 1}{32}\right) -
/// int\left(\frac{int\left(\frac{\mathtt{pixel\\_index} - 1}{32}\right)}{2}\right) \* 2 \newline
/// \textit{Conversion}\_{\textit{pattern}} =& \left(int\left(\frac{\mathtt{pixel\\_index} - 3}{4}\right)\right. \newline
/// &\qquad {} - int\left(\frac{\mathtt{pixel\\_index} - 2}{4}\right) \newline
/// &\qquad {} + int\left(\frac{\mathtt{pixel\\_index}}{4}\right) \newline
/// &\qquad \left. {} - int\left(\frac{\mathtt{pixel\\_index} - 1}{4}\right)
/// \right) * (1 - 2 * IL\_{\textit{Pattern}})
/// \end{split}
/// $$
///
/// Can be simplified to
///
/// $$
/// \begin{split}
/// IL\_{\textit{Pattern}} =& {} \mathtt{pixel\\_index} \bmod 2 \newline
/// \textit{Conversion}\_{\textit{pattern}} =& \left(\mathtt{pixel\\_index} \bmod 2\right) \newline
/// &\qquad {} * \left(\left(\mathtt{pixel\\_index} \bmod 4 \right) - 2\right) \newline
/// &\qquad {} * \left(1 - \left(int\left(\frac{\mathtt{pixel\\_index}}{32}\right) \bmod 2 * 2\right)\right)
/// \end{split}
/// $$
///
/// Those values are used for the actual correction formula:
///
/// \begin{align*}
/// \text{pix}\_{OS(i, j)} &= pix\_{gain(i, j)} \newline
/// &\qquad \colorbox{yellow}{$ {} + IL\_{CHESS\_3} \* \left(2 * IL\_{\textit{Pattern}} - 1 \right)$} \newline
/// &\qquad \colorbox{yellow}{${} - IL\_{CHESS\_2} \* \textit{Conversion}\_{\textit{pattern}}$} \newline
/// &\qquad {} - \textit{offset}\_{(i, j)} \newline
/// &\qquad {} \* (1 + K_{T\_{a}(i, j)} \* (T_a - T_{a_0})) \newline
/// &\qquad {} \* (1 + K_{V(i, j)} \* (V_{DD} - V_{DD_0}))
/// \end{align*}
///
/// This isn't stated in the datasheet, but if you actually calculate the values from
/// section 11.2.3.1, you see that there are two, four element patterns. The patterns are
/// alternated for each row, and the elements are repeated across the row.
/// ```text
/// -A  B -A -C
///  A -B  A  C
/// ```
///
/// Where $A = IL\_{CHESS\_3}$, $B = IL\_{CHESS\_2} - IL\_{CHESS\_3}$, and $C = IL\_{CHESS\_2} + IL\_{CHESS\_3}$
#[derive(Clone, Debug)]
pub enum PixelAccessPatternCompensation<'a> {
    Chess(iter::Take<iter::Repeat<Option<&'a f32>>>),
    Interleave {
        index: usize,
        /// The pattern values, split across two arrays of three elements
        patterns: &'a [f32],
    },
}

impl<'a> PixelAccessPatternCompensation<'a> {
    /// Create a new interleaved correction iterator from a slice of correction constants
    ///
    /// The slice must be 6 elements long, with the values laid out as `[-A, B, -C, A, -B, C]`.
    fn new(access_pattern: AccessPattern, patterns: &'a [f32]) -> Self {
        match access_pattern {
            AccessPattern::Chess => Self::Chess(iter::repeat(None).take(Mlx90640::NUM_PIXELS)),
            AccessPattern::Interleave => Self::Interleave { index: 0, patterns },
        }
    }
}

impl<'a> Iterator for PixelAccessPatternCompensation<'a> {
    type Item = Option<&'a f32>;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            PixelAccessPatternCompensation::Chess(inner) => inner.next(),
            PixelAccessPatternCompensation::Interleave { index, patterns } => {
                if *index < Mlx90640::NUM_PIXELS {
                    // We just need the even/odd-ness of the row index
                    let row_sign = (*index / Mlx90640::WIDTH) % 2;
                    let column_index = match *index % 4 {
                        0 | 2 => 0,
                        1 => 1,
                        3 => 2,
                        _ => unreachable!(),
                    };
                    let pattern_index = row_sign * 3 + column_index;
                    *index += 1;
                    Some(Some(&patterns[pattern_index]))
                } else {
                    None
                }
            }
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
#[allow(clippy::excessive_precision)]
pub(crate) mod test {
    #[cfg(feature = "std")]
    extern crate std;
    #[cfg(feature = "std")]
    use std::{print, println};

    use arrayvec::ArrayVec;
    use float_cmp::{assert_approx_eq, ApproxEq};
    use mlx9064x_test_data::{mlx90640_datasheet_eeprom, mlx90640_example_data};

    use crate::common::{CalibrationData, FlaggedPixels, MelexisCamera};
    use crate::mlx90640::address::EepromAddress;
    use crate::mlx90640::Mlx90640;
    use crate::register::{AccessPattern, Resolution, Subpage};

    use super::{Mlx90640Calibration, WORD_SIZE};

    fn datasheet_eeprom() -> Mlx90640Calibration {
        let eeprom_bytes = mlx90640_datasheet_eeprom();
        Mlx90640Calibration::from_data(&eeprom_bytes).expect("The EEPROM data to be parsed.")
    }

    fn example_eeprom() -> Mlx90640Calibration {
        Mlx90640Calibration::from_data(mlx90640_example_data::EEPROM_DATA)
            .expect("The example data should be parseable")
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
        let test_pattern: ArrayVec<i8, { Mlx90640::NUM_PIXELS }> =
            Mlx90640Calibration::repeat_chessboard(pattern).collect();
        // Print the test pattern (when std is available), as that makes it much easier to see
        // what's going on.
        #[cfg(feature = "std")]
        for row in 0..Mlx90640::HEIGHT {
            for column in 0..Mlx90640::WIDTH {
                let index = row * Mlx90640::WIDTH + column;
                print!("{} ", test_pattern[index]);
            }
            println!();
        }
        for column in 0..Mlx90640::WIDTH {
            for row in 0..Mlx90640::HEIGHT {
                let index = row * Mlx90640::WIDTH + column;
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
        datasheet_eeprom();
        example_eeprom();
    }

    // Ordering these tests in the same order as the data sheet's worked example.
    #[test]
    fn resolution() {
        assert_eq!(datasheet_eeprom().resolution(), Resolution::Eighteen);
        assert_eq!(
            example_eeprom().resolution(),
            Resolution::from_raw(mlx90640_example_data::RESOLUTION_EE as u16).unwrap()
        );
    }

    #[test]
    fn k_v_dd() {
        assert_eq!(datasheet_eeprom().k_v_dd(), -3168);
        assert_eq!(example_eeprom().k_v_dd(), mlx90640_example_data::K_V_DD);
    }

    #[test]
    fn v_dd_25() {
        assert_eq!(datasheet_eeprom().v_dd_25(), -13056);
        assert_eq!(example_eeprom().v_dd_25(), mlx90640_example_data::V_DD_25);
    }

    #[test]
    fn v_dd_0() {
        assert_eq!(datasheet_eeprom().v_dd_0(), 3.3);
        assert_eq!(example_eeprom().v_dd_0(), 3.3);
    }

    #[test]
    fn k_v_ptat() {
        assert_approx_eq!(
            f32,
            datasheet_eeprom().k_v_ptat(),
            0.005371094,
            epsilon = 0.000000001
        );
        assert_approx_eq!(
            f32,
            example_eeprom().k_v_ptat(),
            mlx90640_example_data::K_V_PTAT,
            epsilon = 0.00001
        );
    }

    #[test]
    fn k_t_ptat() {
        // These values are scaled by 1/8 and are not too large so they can be exactly represented
        // in an f32.
        assert_eq!(datasheet_eeprom().k_t_ptat(), 42.25);
        assert_eq!(example_eeprom().k_t_ptat(), mlx90640_example_data::K_T_PTAT);
    }

    #[test]
    fn v_ptat_25() {
        // These values are integers for the 640.
        assert_eq!(datasheet_eeprom().v_ptat_25(), 12273f32);
        assert_eq!(
            example_eeprom().v_ptat_25(),
            mlx90640_example_data::V_PTAT_25
        );
    }

    #[test]
    fn alpha_ptat() {
        // The example values are both integers
        assert_eq!(datasheet_eeprom().alpha_ptat(), 9f32);
        assert_eq!(
            example_eeprom().alpha_ptat(),
            mlx90640_example_data::ALPHA_PTAT
        );
    }
    #[test]

    fn gain() {
        // The EEPROM gain value is an integer for the 640
        assert_eq!(datasheet_eeprom().gain(), 6383f32);
        assert_eq!(example_eeprom().gain(), mlx90640_example_data::GAIN_EE);
    }

    fn test_pixels_common<T: PartialEq + core::fmt::Debug + core::fmt::Display + Copy>(
        datasheet_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        example_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        datasheet_expected: T,
        example_expected: &[T; Mlx90640::NUM_PIXELS],
        subpage: Option<Subpage>,
        check: &dyn Fn(T, T) -> bool,
    ) {
        if subpage.is_none() {
            assert_eq!(datasheet_data[0], datasheet_data[1]);
            assert_eq!(example_data[0], example_data[1]);
        }
        // Test the single pixel from the datasheet
        let datasheet_index = 11 * Mlx90640::WIDTH + 15;
        let subpage_index: usize = subpage.unwrap_or(Subpage::Zero).into();
        let pixel = datasheet_data[subpage_index][datasheet_index];
        assert!(
            check(pixel, datasheet_expected),
            "[datasheet pixel]:\n{:>10}: `{:?}`,\n{:>10}: `{:?}`,",
            "expected",
            datasheet_expected,
            "actual",
            pixel
        );
        // Check all the pixels from the full example
        let offset_pairs = example_data[subpage_index]
            .iter()
            .zip(example_expected.iter());
        for (index, (actual, expected)) in offset_pairs.enumerate() {
            assert!(
                check(*actual, *expected),
                "[pixel {:?}]:\n{:>10}: `{:?}`,\n{:>10}: `{:?}`,",
                index,
                "expected",
                expected,
                "actual",
                actual
            );
        }
    }

    fn test_pixels_approx<T>(
        datasheet_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        example_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        datasheet_expected: T,
        example_expected: &[T; Mlx90640::NUM_PIXELS],
        subpage: Option<Subpage>,
        margin: Option<<T as ApproxEq>::Margin>,
    ) where
        T: ApproxEq + PartialEq + core::fmt::Debug + core::fmt::Display + Copy,
    {
        let check = |actual: T, expected: T| actual.approx_eq(expected, margin.unwrap_or_default());
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
        datasheet_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        example_data: [ArrayVec<T, { Mlx90640::NUM_PIXELS }>; 2],
        datasheet_expected: T,
        example_expected: &[T; Mlx90640::NUM_PIXELS],
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
    fn pixel_offset() {
        let datasheet = datasheet_eeprom();
        let datasheet_offsets: [ArrayVec<i16, { Mlx90640::NUM_PIXELS }>; 2] = [
            datasheet
                .offset_reference_pixels(Subpage::Zero)
                .copied()
                .collect(),
            datasheet
                .offset_reference_pixels(Subpage::One)
                .copied()
                .collect(),
        ];
        let example = example_eeprom();
        let example_offsets: [ArrayVec<i16, { Mlx90640::NUM_PIXELS }>; 2] = [
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
    fn pixel_k_ta() {
        let datasheet = datasheet_eeprom();
        let datasheet_k_ta: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            datasheet.k_ta_pixels(Subpage::Zero).copied().collect(),
            datasheet.k_ta_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom();
        let example_k_ta: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            example.k_ta_pixels(Subpage::Zero).copied().collect(),
            example.k_ta_pixels(Subpage::One).copied().collect(),
        ];
        test_pixels_approx(
            datasheet_k_ta,
            example_k_ta,
            0.005126953125,
            &mlx90640_example_data::K_TA_PIXELS,
            // MLX90640 doesn't vary k_ta on subpage
            None,
            // The example spreadsheet goes out to eight decimal places
            Some((10E-8, 2).into()),
        );
    }

    #[test]
    fn k_v_pixels() {
        let datasheet = datasheet_eeprom();
        let datasheet_k_v: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            datasheet.k_v_pixels(Subpage::Zero).copied().collect(),
            datasheet.k_v_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom();
        let example_k_v: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            example.k_v_pixels(Subpage::Zero).copied().collect(),
            example.k_v_pixels(Subpage::One).copied().collect(),
        ];
        test_pixels_approx(
            datasheet_k_v,
            example_k_v,
            0.5f32,
            &mlx90640_example_data::K_V_PIXELS,
            // MLX90640 doesn't vary k_v on subpage
            None,
            // The values in the datasheet are all exactly representable in a float, so no need for
            // an explicit margin.
            None,
        );
    }

    #[test]
    fn emissivity() {
        assert_eq!(datasheet_eeprom().emissivity(), None);
        assert_eq!(example_eeprom().emissivity(), None);
    }

    #[test]
    fn offset_reference_cp() {
        // datasheet
        let datasheet = datasheet_eeprom();
        assert_eq!(datasheet.offset_reference_cp(Subpage::Zero), -75);
        assert_eq!(datasheet.offset_reference_cp(Subpage::One), -77);
        // example
        let example = example_eeprom();
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
    fn k_ta_cp() {
        // The 640 doesn't vary k_ta_cp on subpage.
        // datasheet
        let datasheet = datasheet_eeprom();
        // This value is more precise than an f32, so skipping an explicit margin
        let expected = 0.00457763671875;
        assert_approx_eq!(f32, datasheet.k_ta_cp(Subpage::Zero), expected);
        assert_approx_eq!(f32, datasheet.k_ta_cp(Subpage::One), expected);
        // example
        let example = example_eeprom();
        // The example value is only given to 6 decimal places
        assert_approx_eq!(
            f32,
            example.k_ta_cp(Subpage::Zero),
            mlx90640_example_data::K_TA_CP,
            epsilon = 10E-6
        );
        assert_approx_eq!(
            f32,
            example.k_ta_cp(Subpage::One),
            mlx90640_example_data::K_TA_CP,
            epsilon = 10E-6
        );
    }

    #[test]
    fn k_v_cp() {
        // Both the datasheet and the example spreadsheet values are fractional powers of 2 and can
        // be exactly represented by floats.
        // datasheet
        let datasheet = datasheet_eeprom();
        let expected = 0.5;
        assert_eq!(datasheet.k_v_cp(Subpage::Zero), expected);
        assert_eq!(datasheet.k_v_cp(Subpage::One), expected);
        // example
        let example = example_eeprom();
        assert_eq!(example.k_v_cp(Subpage::Zero), mlx90640_example_data::K_V_CP);
        assert_eq!(example.k_v_cp(Subpage::One), mlx90640_example_data::K_V_CP);
    }

    #[test]
    fn temperature_gradient_coefficient() {
        assert_eq!(
            datasheet_eeprom().temperature_gradient_coefficient(),
            Some(1f32)
        );
        assert_eq!(
            example_eeprom().temperature_gradient_coefficient(),
            // 0 TGC means None
            None,
        );
    }

    #[test]
    fn alpha_cp() {
        // datasheet
        let datasheet = datasheet_eeprom();
        // The datasheet values go out to 11 decimal places
        assert_approx_eq!(
            f32,
            datasheet.alpha_cp(Subpage::Zero),
            4.07453626394272E-9,
            epsilon = 10E-11
        );
        assert_approx_eq!(
            f32,
            datasheet.alpha_cp(Subpage::One),
            3.851710062200835E-9,
            epsilon = 10E-11
        );
        // example
        let example = example_eeprom();
        // The example data goes out to 19 decimal places (but only 12 significant figures).
        assert_approx_eq!(
            f32,
            example.alpha_cp(Subpage::Zero),
            mlx90640_example_data::ALPHA_CP[0],
            epsilon = 10E-19
        );
        assert_approx_eq!(
            f32,
            example.alpha_cp(Subpage::One),
            mlx90640_example_data::ALPHA_CP[1],
            epsilon = 10E-19
        );
    }

    #[test]
    fn k_s_ta() {
        assert_approx_eq!(
            f32,
            datasheet_eeprom().k_s_ta(),
            -0.001953125,
            // The datasheet value is specified out to 9 decimal places
            epsilon = 10E-9
        );
        assert_approx_eq!(
            f32,
            example_eeprom().k_s_ta(),
            mlx90640_example_data::K_S_TA,
            // Example value is only specified out to 6 decimal places
            epsilon = 10E-6
        );
    }

    #[test]
    fn pixel_alpha() {
        // MLX90640 doesn't vary alpha on subpage
        let datasheet = datasheet_eeprom();
        let datasheet_alpha: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            datasheet.alpha_pixels(Subpage::Zero).copied().collect(),
            datasheet.alpha_pixels(Subpage::One).copied().collect(),
        ];
        let example = example_eeprom();
        let example_alpha: [ArrayVec<f32, { Mlx90640::NUM_PIXELS }>; 2] = [
            example.alpha_pixels(Subpage::Zero).copied().collect(),
            example.alpha_pixels(Subpage::One).copied().collect(),
        ];
        test_pixels_approx(
            datasheet_alpha,
            example_alpha,
            1.262233122690854E-7,
            &mlx90640_example_data::ALPHA_PIXELS,
            None,
            // The example data goes out to 19 decimal places (but only 12 significant figures).
            Some((10E-19, 2).into()),
        );
    }

    #[test]
    fn k_s_to() {
        assert_approx_eq!(
            f32,
            datasheet_eeprom().k_s_to()[1],
            -0.00080108642578125,
            // Datasheet value is exactly reproduced above, so 17 decimal places
            epsilon = 10E-17
        );
        let example = example_eeprom();
        let example_pairs = example
            .k_s_to()
            .iter()
            .zip(mlx90640_example_data::K_S_TO.iter());
        for (actual, expected) in example_pairs {
            assert_approx_eq!(
                f32,
                *actual,
                *expected,
                // The example values are only specified out to 6 decimal places
                epsilon = 10E-6
            );
        }
    }

    #[test]
    fn corner_temperatures() {
        let e = datasheet_eeprom();
        let ct = e.corner_temperatures();
        // The first two values are hard-coded, but testing for completeness.
        assert_eq!(ct[0], -40);
        assert_eq!(ct[1], 0);
        // These are loaded from EEPROM
        assert_eq!(ct[2], 160);
        assert_eq!(ct[3], 320);

        // Full example test
        assert_eq!(
            example_eeprom().corner_temperatures(),
            mlx90640_example_data::CORNER_TEMPERATURES
        );
    }

    #[test]
    fn access_pattern_compensation_chess() {
        let e = datasheet_eeprom();
        for compensation in e.access_pattern_compensation_pixels(AccessPattern::Chess) {
            assert!(
                compensation.is_none(),
                "There is no access pattern compensation in chess mode"
            );
        }
    }

    #[test]
    fn access_pattern_compensation_interleave() {
        let e = datasheet_eeprom();
        // Using the variable names described in the `PixelAccessPatternCompensation` docstring.
        const IL_CHESS_3: f32 = 0.125;
        const IL_CHESS_2: f32 = 3.0;
        const A: f32 = IL_CHESS_3;
        const B: f32 = IL_CHESS_2 - IL_CHESS_3;
        const C: f32 = IL_CHESS_2 + IL_CHESS_3;
        let compensation_iter = e
            .access_pattern_compensation_pixels(AccessPattern::Interleave)
            .enumerate();
        let mut count = 0;
        for (pixel_index, compensation) in compensation_iter {
            let row = pixel_index / Mlx90640::WIDTH;
            let column = pixel_index % Mlx90640::WIDTH;
            // Start with the expected values for an even row
            let expected = if column % 2 == 0 {
                -A
            } else if column % 4 == 3 {
                -C
            } else {
                B
            };
            // Negate the value for an odd row
            let expected = if row % 2 == 0 { expected } else { -expected };
            assert_eq!(
                Some(&expected),
                compensation,
                "[pixel {:?}]:\n{:>10}: `{:?}`,\n{:>10}: `{:?}`,",
                pixel_index,
                "expected",
                Some(expected),
                "actual",
                compensation
            );
            count += 1;
        }
        assert_eq!(count, Mlx90640::NUM_PIXELS);
    }

    #[test]
    fn failed_pixels_example() {
        // The worked example has no failed pixels
        let example = example_eeprom();
        assert!(!example.failed_pixels().any());
    }

    #[test]
    fn failed_pixels_induced() {
        // Starting with the datasheet EEPROM, mark pixel 10 as failed
        let mut eeprom_bytes = mlx90640_datasheet_eeprom();
        const PIXEL_NUM: usize = 10;
        const PIXEL_OFFSET: usize =
            EepromAddress::PixelCalibrationStart.byte_offset() + PIXEL_NUM * WORD_SIZE;
        eeprom_bytes[PIXEL_OFFSET] = 0;
        // both bytes need to be 0 for it to be a failed pixel
        eeprom_bytes[PIXEL_OFFSET + 1] = 0;
        let datasheet =
            Mlx90640Calibration::from_data(&eeprom_bytes).expect("The EEPROM data to be parsed.");
        let failures = datasheet.failed_pixels();
        assert!(failures.any());
        let mut failed_iter = failures.iter_flagged();
        assert_eq!(failed_iter.next(), Some(10));
        assert_eq!(failed_iter.next(), None);
        // A failed pixel is not marked as an outlier
        let outliers = datasheet.outlier_pixels();
        assert!(!outliers.any());
    }

    #[test]
    fn outlier_pixels_example() {
        // The worked example has no outlying pixels
        let example = example_eeprom();
        assert!(!example.failed_pixels().any());
    }

    #[test]
    fn outlier_pixels_induced() {
        // Starting with the datasheet EEPROM, mark pixel 10 as an outlier
        let mut eeprom_bytes = mlx90640_datasheet_eeprom();
        const PIXEL_NUM: usize = 10;
        const PIXEL_OFFSET: usize =
            EepromAddress::PixelCalibrationStart.byte_offset() + PIXEL_NUM * WORD_SIZE;
        // Only the last bit of the lower byte is set for an outlier
        eeprom_bytes[PIXEL_OFFSET + 1] |= 1;
        let datasheet =
            Mlx90640Calibration::from_data(&eeprom_bytes).expect("The EEPROM data to be parsed.");
        let outliers = datasheet.outlier_pixels();
        assert!(outliers.any());
        let mut outlier_iter = outliers.iter_flagged();
        assert_eq!(outlier_iter.next(), Some(PIXEL_NUM));
        assert_eq!(outlier_iter.next(), None);
        // An outlier pixel is not marked as a failure
        let failures = datasheet.failed_pixels();
        assert!(!failures.any());
    }
}
