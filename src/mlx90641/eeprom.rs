// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

//! MLX90641-specific EEPROM handling

use core::slice;

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use crate::common::*;
use crate::error::{Error, LibraryError};
use crate::expose_member;
use crate::register::Subpage;
use crate::util::{i16_from_bits, Buffer};

use super::address::EepromAddress;
use super::hamming::validate_checksum;
use super::Mlx90641;

/// The number of corner temperatures an MLX90641 has.
const NUM_CORNER_TEMPERATURES: usize = 8;

/// The word size of the MLX90641 in terms of 8-bit bytes.
const WORD_SIZE: usize = 16 / 8;

#[derive(Clone, Debug, PartialEq)]
pub struct Mlx90641Calibration {
    k_v_dd: i16,

    v_dd_25: i16,

    resolution: u8,

    k_v_ptat: f32,

    k_t_ptat: f32,

    v_ptat_25: f32,

    alpha_ptat: f32,

    gain: f32,

    k_s_ta: f32,

    corner_temperatures: [i16; NUM_CORNER_TEMPERATURES],

    k_s_to: [f32; NUM_CORNER_TEMPERATURES],

    alpha_correction: [f32; NUM_CORNER_TEMPERATURES],

    emissivity: Option<f32>,

    alpha_pixels: [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],

    alpha_cp: f32,

    offset_reference_pixels: [[i16; <Self as CalibrationData>::Camera::NUM_PIXELS]; 2],

    offset_reference_cp: i16,

    k_v_pixels: [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],

    k_v_cp: f32,

    k_ta_pixels: [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],

    k_ta_cp: f32,

    temperature_gradient_coefficient: Option<f32>,
}

impl Mlx90641Calibration {
    pub fn from_data(mut buf: &[u8]) -> Result<Self, LibraryError> {
        // Much like the MLX90640 implementation, this is a mess of a function as the data is
        // scattered across the EEPROM.
        // Skip the first 16 words, they're not used for calibration data
        buf.advance(WORD_SIZE * 16);
        // Offset scale is the upper 6 bits. The other bits are reserved.
        let (offset_scale, _) = get_6_5_split(&mut buf)?;
        let offset_average_bytes = get_combined_word(&mut buf)?;
        let offset_average = i16_from_bits(&offset_average_bytes[..], 11);
        // the next two words are reserved.
        buf.advance(WORD_SIZE * 2);
        let k_ta_average = get_hamming_i16(&mut buf)?;
        let k_ta_scales = get_6_5_split(&mut buf)?;
        let k_v_average = get_hamming_i16(&mut buf)?;
        let k_v_scales = get_6_5_split(&mut buf)?;
        let alpha_reference = Self::get_sensitivity_reference(&mut buf)?;
        let k_s_ta = f32::from(get_hamming_i16(&mut buf)?) / 15f32.exp2();
        let emissivity = f32::from(get_hamming_i16(&mut buf)?) / 9f32.exp2();
        let gain = u16::from_be_bytes(get_combined_word(&mut buf)?);
        // TODO: These two parameters might need to be upsized to 32-bit ints
        let v_dd_25 = get_hamming_i16(&mut buf)? << 5;
        let k_v_dd = get_hamming_i16(&mut buf)? << 5;
        let v_ptat_25 = u16::from_be_bytes(get_combined_word(&mut buf)?);
        // Scaled by 2^3
        let k_t_ptat = f32::from(get_hamming_i16(&mut buf)?) / 8f32;
        // Scaled by 2^12
        let k_v_ptat = f32::from(get_hamming_i16(&mut buf)?) / 4096f32;
        // Scaled by 2^7 (not 11, as the address map says)
        let alpha_ptat = f32::from(get_hamming_u16(&mut buf)?) / 128f32;
        let alpha_cp = f32::from(get_hamming_u16(&mut buf)?);
        let alpha_cp_scale = f32::from(get_hamming_u16(&mut buf)?);
        let alpha_cp = alpha_cp / alpha_cp_scale.exp2();
        let offset_reference_cp = i16::from_be_bytes(get_combined_word(&mut buf)?);
        let k_ta_cp = Self::get_scaled_cp_constant(&mut buf)?;
        let k_v_cp = Self::get_scaled_cp_constant(&mut buf)?;
        let (resolution, tgc) = Self::get_resolution_with_tgc(&mut buf)?;
        let (corner_temperatures, k_s_to) = Self::get_temperature_range_data(&mut buf)?;
        let basic_range = <Self as CalibrationData>::Camera::BASIC_TEMPERATURE_RANGE;
        let alpha_correction =
            alpha_correction_coefficients(basic_range, &corner_temperatures, &k_s_to);
        // TODO: false pixel detection
        let pixel_offsets_0 = Self::get_pixel_offsets(&mut buf, offset_scale, offset_average)?;
        let alpha_pixels = Self::get_pixel_sensitivities(&mut buf, alpha_reference)?;
        let (k_ta_pixels, k_v_pixels) = Self::get_pixel_temperature_constants(
            &mut buf,
            k_ta_average,
            k_ta_scales,
            k_v_average,
            k_v_scales,
        )?;
        let pixel_offsets_1 = Self::get_pixel_offsets(&mut buf, offset_scale, offset_average)?;
        Ok(Self {
            k_v_dd,
            v_dd_25,
            resolution,
            k_v_ptat,
            k_t_ptat,
            v_ptat_25: v_ptat_25.into(),
            alpha_ptat,
            gain: gain.into(),
            k_s_ta,
            corner_temperatures,
            k_s_to,
            alpha_correction,
            emissivity: Some(emissivity),
            alpha_pixels,
            alpha_cp,
            offset_reference_pixels: [pixel_offsets_0, pixel_offsets_1],
            offset_reference_cp,
            k_v_pixels,
            k_v_cp,
            k_ta_pixels,
            k_ta_cp,
            temperature_gradient_coefficient: Some(tgc),
        })
    }

    /// Calculate $\alpha\_{\textit{reference}}$ for each "row" of pixels
    ///
    /// $$
    /// \alpha\_{\textit{reference}\_N} =
    /// \frac{\textit{Row}\_{\textit{max}\_N}}{2^{\alpha\_{\textit{scale}\_N}}} \newline
    /// $$
    ///
    /// The rows are defined as 32 contiguous pixels, so can better be thought of as pairs of rows.
    /// The scale values are stored as unsigned integers, two per word, with the upper 6 bits being
    /// the scale for the first row, then the lower 5 bits being the scale for the next row, and so
    /// on for three words, starting at 0x2419. Each scale value also need to be added to 20.
    /// $\textit{Row}\_{\textit{max}}$ is stored as an unsigned 11-bit integer, with one value per
    /// word, starting at 0x241C.
    #[doc = include_str!("../katex.html")]
    fn get_sensitivity_reference(buf: &mut &[u8]) -> Result<[f32; 6], LibraryError> {
        let mut scales: ArrayVec<u8, 6> = ArrayVec::new();
        for _ in 0..3 {
            let (first_scale, second_scale) = get_6_5_split(buf)?;
            scales.push(first_scale + 20);
            scales.push(second_scale + 20);
        }
        let mut a_reference = [0f32; 6];
        for (dest, scale) in a_reference.iter_mut().zip(scales) {
            let row_max = get_hamming_u16(buf)?;
            *dest = f32::from(row_max) / f32::from(scale).exp2();
        }
        Ok(a_reference)
    }

    /// Calculate $K\_{V\_{CP}}$ or $K\_{T\_{a\_{CP}}}$ values
    ///
    /// These two values are stored in one word in the EEPROM, with the upper five bits being the
    /// scale, and the lower six bits the unscaled value.
    fn get_scaled_cp_constant(buf: &mut &[u8]) -> Result<f32, LibraryError> {
        let word = get_hamming_u16(buf)?;
        let raw_scale = (word & 0x07C0) >> 6;
        let value_bytes = (word & 0x003F).to_be_bytes();
        // the values are signed
        let value_unscaled = i16_from_bits(&value_bytes[..], 6);
        let scale = f32::from(raw_scale).exp2();
        Ok(f32::from(value_unscaled) / scale)
    }

    /// Read the calibrated ADC resolution and thermal gradient compensation value from the EEPROM
    ///
    /// The values are returned as a tuple, with the resolution first, followed by the thermal
    /// gradient compensation (TGC) value. The TGC is pre-scaled, and needs no further calculations
    /// applied.
    fn get_resolution_with_tgc(buf: &mut &[u8]) -> Result<(u8, f32), LibraryError> {
        let word = get_hamming_u16(buf)?;
        let resolution = (word & 0x0600) >> 9;
        let tgc_bytes = (word & 0x01FF).to_be_bytes();
        let tgc_unscaled = i16_from_bits(&tgc_bytes[..], 9);
        // Scaled by 2^6
        let tgc = f32::from(tgc_unscaled) / 64f32;
        Ok((resolution as u8, tgc))
    }

    /// Extract the corner temperatures and $K\_{s\_{T\_o}}$ values
    fn get_temperature_range_data(
        buf: &mut &[u8],
    ) -> Result<
        (
            [i16; NUM_CORNER_TEMPERATURES],
            [f32; NUM_CORNER_TEMPERATURES],
        ),
        LibraryError,
    > {
        let scale = f32::from(get_hamming_u16(buf)?).exp2();
        // The first five corner temperatures are hard-coded to these values, while the last three
        // are read from the EEPROM.
        let mut corner_temperatures: [i16; NUM_CORNER_TEMPERATURES] =
            [-40, -20, 0, 80, 120, 0, 0, 0];
        let mut k_s_to = [0f32; NUM_CORNER_TEMPERATURES];
        // The first five k_s_to values come first in the EEPROM, then come pairs of corner
        // temperature, k_s_to for the remiaining values.
        for dest in k_s_to[..5].iter_mut() {
            let unscaled = get_hamming_i16(buf)?;
            *dest = f32::from(unscaled) / scale;
        }
        let paired_iter = corner_temperatures[5..]
            .iter_mut()
            .zip(k_s_to[5..].iter_mut());
        for (ct, k_s_to) in paired_iter {
            // These are actually 11-bit integers, so they won't be truncated converting them to i16.
            *ct = get_hamming_u16(buf)? as i16;
            let unscaled = get_hamming_i16(buf)?;
            *k_s_to = f32::from(unscaled) / scale;
        }
        Ok((corner_temperatures, k_s_to))
    }

    fn get_pixel_offsets(
        buf: &mut &[u8],
        offset_scale: u8,
        offset_average: i16,
    ) -> Result<[i16; <Self as CalibrationData>::Camera::NUM_PIXELS], LibraryError> {
        let mut pixel_offsets = [0i16; <Self as CalibrationData>::Camera::NUM_PIXELS];
        let scale = 2i16.pow(offset_scale as u32);
        for pixel_offset in pixel_offsets.iter_mut() {
            // NOTE: There's a chance this will overflow, if offset_scale is too large
            let offset_raw = get_hamming_i16(buf)?;
            let scaled_offset = offset_raw * scale;
            *pixel_offset = offset_average + scaled_offset;
        }
        Ok(pixel_offsets)
    }

    fn get_pixel_sensitivities(
        buf: &mut &[u8],
        alpha_reference: [f32; 6],
    ) -> Result<[f32; <Self as CalibrationData>::Camera::NUM_PIXELS], LibraryError> {
        let mut pixel_sensitivites = [0f32; <Self as CalibrationData>::Camera::NUM_PIXELS];
        // The sensitivity reference value is shared across a band of 32 pixels, so chunk the
        // pixels by that, and xip the reference value in
        let referenced_rows = alpha_reference
            .iter()
            .zip(pixel_sensitivites.chunks_exact_mut(32));
        for (reference, row) in referenced_rows {
            for pixel_sensitivity in row {
                let raw_alpha = f32::from(get_hamming_u16(buf)?);
                // The datasheet is a little hard to read for this, but alpha_EE is divided by
                // (2^{11} - 1) = 2047
                *pixel_sensitivity = (raw_alpha / 2047f32) * reference;
            }
        }
        Ok(pixel_sensitivites)
    }

    fn get_pixel_temperature_constants(
        buf: &mut &[u8],
        k_ta_average: i16,
        k_ta_scales: (u8, u8),
        k_v_average: i16,
        k_v_scales: (u8, u8),
    ) -> Result<
        (
            [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],
            [f32; <Self as CalibrationData>::Camera::NUM_PIXELS],
        ),
        LibraryError,
    > {
        let mut k_ta_pixels = [0f32; <Self as CalibrationData>::Camera::NUM_PIXELS];
        let mut k_v_pixels = [0f32; <Self as CalibrationData>::Camera::NUM_PIXELS];
        let k_ta_scale1 = f32::from(k_ta_scales.0).exp2();
        let k_ta_scale2 = f32::from(k_ta_scales.1).exp2();
        let k_v_scale1 = f32::from(k_v_scales.0).exp2();
        let k_v_scale2 = f32::from(k_v_scales.1).exp2();
        let scale_fn = |raw_value: i8, avg: i16, scale1: f32, scale2: f32| {
            let numerator = f32::from(raw_value) * scale2 + f32::from(avg);
            numerator / scale1
        };
        for (k_ta, k_v) in k_ta_pixels.iter_mut().zip(k_v_pixels.iter_mut()) {
            let (k_ta_raw, k_v_raw) = get_6_5_split(buf)?;
            let k_ta_raw = i16_from_bits(&[k_ta_raw], 6) as i8;
            let k_v_raw = i16_from_bits(&[k_v_raw], 5) as i8;
            *k_ta = scale_fn(k_ta_raw, k_ta_average, k_ta_scale1, k_ta_scale2);
            *k_v = scale_fn(k_v_raw, k_v_average, k_v_scale1, k_v_scale2);
        }
        Ok((k_ta_pixels, k_v_pixels))
    }
}

impl<I2C> FromI2C<I2C> for Mlx90641Calibration
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

impl<'a> CalibrationData<'a> for Mlx90641Calibration {
    type Camera = Mlx90641;

    expose_member!(k_v_dd, i16);
    expose_member!(v_dd_25, i16);
    expose_member!(resolution, u8);
    expose_member!(k_v_ptat, f32);
    expose_member!(k_t_ptat, f32);
    expose_member!(v_ptat_25, f32);
    expose_member!(alpha_ptat, f32);
    expose_member!(gain, f32);
    expose_member!(k_s_ta, f32);

    expose_member!(&corner_temperatures, [i16]);
    expose_member!(&k_s_to, [f32]);
    expose_member!(&alpha_correction, [f32]);

    expose_member!(emissivity, Option<f32>);

    type OffsetReferenceIterator = slice::Iter<'a, i16>;

    fn offset_reference_pixels(&'a self, subpage: Subpage) -> Self::OffsetReferenceIterator {
        match subpage {
            Subpage::Zero => self.offset_reference_pixels[0].iter(),
            Subpage::One => self.offset_reference_pixels[1].iter(),
        }
    }

    fn offset_reference_cp(&self, _subpage: Subpage) -> i16 {
        self.offset_reference_cp
    }

    type AlphaIterator = slice::Iter<'a, f32>;

    fn alpha_pixels(&'a self, _subpage: Subpage) -> Self::AlphaIterator {
        self.alpha_pixels.iter()
    }

    fn alpha_cp(&self, _subpage: Subpage) -> f32 {
        self.alpha_cp
    }

    type KvIterator = slice::Iter<'a, f32>;

    fn k_v_pixels(&'a self, _subpage: Subpage) -> Self::KvIterator {
        self.k_v_pixels.iter()
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

/// Pop a word out of a buffer, decoding the checksum and then stripping it off
fn get_hamming_u16(buf: &mut &[u8]) -> Result<u16, LibraryError> {
    let codeword = buf.get_u16();
    validate_checksum(codeword)
}

/// Pop a word out of a buffer, decoding the checksum and then stripping it off
fn get_hamming_i16(buf: &mut &[u8]) -> Result<i16, LibraryError> {
    let bytes = get_hamming_u16(buf)?.to_be_bytes();
    Ok(i16_from_bits(&bytes[..], 11))
}

/// Read two successive words from the buffer, combining them into one
///
/// Since the MLX90641 EEPROM has a Hamming code in the upper five bits, it can only fit eleven
/// bits of data in each word. Some values need 16-bits though, so they're split across two words.
/// This function combines the bits and returns two bytes.
fn get_combined_word(buf: &mut &[u8]) -> Result<[u8; 2], LibraryError> {
    let upper = get_hamming_u16(buf)?;
    let lower = get_hamming_u16(buf)?;
    // TODO: Follow up with Melexis to see if the high word could have more than 5 bits set
    let combined = (upper << 5) | lower;
    Ok(combined.to_be_bytes())
}

/// Split a word into two values: the upper six bits and the lower five bits
fn get_6_5_split(buf: &mut &[u8]) -> Result<(u8, u8), LibraryError> {
    let word = get_hamming_u16(buf)?;
    let upper = (word & 0x07E0) >> 5;
    let lower = word & 0x001F;
    Ok((upper as u8, lower as u8))
}

#[cfg(test)]
pub(crate) mod test {
    use arrayvec::ArrayVec;

    use crate::common::{CalibrationData, MelexisCamera};
    use crate::mlx90641::eeprom::NUM_CORNER_TEMPERATURES;
    use crate::mlx90641::Mlx90641;
    use crate::register::Subpage;
    use crate::test::mlx90641_datasheet_eeprom;

    use super::Mlx90641Calibration;

    // The example is testing pixel (6, 9), so (5, 8) zero-indexed
    const TEST_PIXEL_INDEX: usize = 5 * Mlx90641::WIDTH + 8;

    pub(crate) fn datasheet_eeprom() -> Mlx90641Calibration {
        let mut eeprom_bytes = mlx90641_datasheet_eeprom();
        Mlx90641Calibration::from_data(&mut eeprom_bytes).expect("The EEPROM data to be parsed.")
    }

    #[test]
    fn get_hamming_u16() {
        // Cherry picking values from the examples in the datasheet, in this case Vdd_25,
        // alpha(6, 9), and emissivity
        let data = b"\x26\x58\xff\xff\x19\xe6";
        let mut buf = &data[..];
        assert_eq!(super::get_hamming_u16(&mut buf), Ok(0x0658));
        assert_eq!(super::get_hamming_u16(&mut buf), Ok(0x07FF));
        assert_eq!(super::get_hamming_u16(&mut buf), Ok(0x01e6));
        assert_eq!(buf.len(), 0);
    }

    #[test]
    fn get_hamming_i16() {
        // Some of the same values as the u16 test, but some of the expected values are different
        // as they're signed.
        let data = b"\x26\x58\x19\xe6";
        let mut buf = &data[..];
        assert_eq!(super::get_hamming_i16(&mut buf), Ok(-424));
        assert_eq!(super::get_hamming_i16(&mut buf), Ok(486));
        assert_eq!(buf.len(), 0);
    }

    #[test]
    fn get_combined_word() {
        // There's only a couple of values split across two words, so test all of the examples
        // available. In order: Pixel offset reference (Offset_average), gain, ptat (v_ptat_25),
        // offset_cp.
        let data = b"\xb7\xe8\xd0\x16\xf1\x37\x78\x14\x91\x7f\xf0\x18\xcf\xfc\xa0\x09";
        let mut buf = &data[..];
        assert_eq!(
            super::get_combined_word(&mut buf),
            Ok(64790u16.to_be_bytes())
        );
        assert_eq!(
            super::get_combined_word(&mut buf),
            Ok(9972u16.to_be_bytes())
        );
        assert_eq!(
            super::get_combined_word(&mut buf),
            Ok(12280u16.to_be_bytes())
        );
        assert_eq!(
            super::get_combined_word(&mut buf),
            Ok(65417u16.to_be_bytes())
        );
        assert_eq!(buf.len(), 0);
    }

    /// Check that it can even create itself from a buffer.
    #[test]
    fn smoke() {
        datasheet_eeprom();
    }

    // Ordering these tests in the same order as the data sheet's worked example.
    #[test]
    fn resolution() {
        assert_eq!(datasheet_eeprom().resolution(), 2);
    }

    #[test]
    fn k_v_dd() {
        assert_eq!(datasheet_eeprom().k_v_dd(), -3136);
    }

    #[test]
    fn v_dd_25() {
        assert_eq!(datasheet_eeprom().v_dd_25(), -13568);
    }

    #[test]
    fn v_dd_0() {
        assert_eq!(datasheet_eeprom().v_dd_0(), 3.3);
    }

    // Slight variance from the datasheet's example: I added one more digit (the last 4)
    #[test]
    fn k_v_ptat() {
        assert_eq!(datasheet_eeprom().k_v_ptat(), 0.0056152344);
    }

    #[test]
    fn k_t_ptat() {
        assert_eq!(datasheet_eeprom().k_t_ptat(), 42.75);
    }

    #[test]
    fn v_ptat_25() {
        assert_eq!(datasheet_eeprom().v_ptat_25(), 12280f32);
    }

    #[test]
    fn alpha_ptat() {
        assert_eq!(datasheet_eeprom().alpha_ptat(), 9f32);
    }

    #[test]
    fn gain() {
        assert_eq!(datasheet_eeprom().gain(), 9972f32);
    }

    #[test]
    fn pixel_offset() {
        let e = datasheet_eeprom();
        let offsets0: ArrayVec<i16, { Mlx90641::NUM_PIXELS }> =
            e.offset_reference_pixels(Subpage::Zero).copied().collect();
        let offsets1: ArrayVec<i16, { Mlx90641::NUM_PIXELS }> =
            e.offset_reference_pixels(Subpage::One).copied().collect();
        assert_eq!(offsets0[TEST_PIXEL_INDEX], -673);
        // NOTE: This is a larger departure from the datasheet's worked example. At least as of
        // revision 3 of the MLX90641 datasheet, the math in section 11.2.2.5.3 for
        // pix_OS_ref_SP1(6,9) is incorrect, stating that -746 + 71 * 2^0 = -671.
        // It actually equals -675.
        assert_eq!(offsets1[TEST_PIXEL_INDEX], -675);
    }

    #[test]
    fn k_ta_pixels() {
        let e = datasheet_eeprom();
        let k_ta_pixels0: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.k_ta_pixels(Subpage::Zero).copied().collect();
        let k_ta_pixels1: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.k_ta_pixels(Subpage::One).copied().collect();
        // Subpage is ignored for K_Ta
        assert_eq!(k_ta_pixels0, k_ta_pixels1);
        // Slight difference from the datasheet example: the last digit (used to be 9) was extended
        // to "89"
        assert_eq!(k_ta_pixels0[TEST_PIXEL_INDEX], 0.0031013489);
    }

    #[test]
    fn k_v_pixels() {
        let e = datasheet_eeprom();
        let k_v_pixels0: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.k_v_pixels(Subpage::Zero).copied().collect();
        let k_v_pixels1: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.k_v_pixels(Subpage::One).copied().collect();
        // Subpage is ignored for K_V
        assert_eq!(k_v_pixels0, k_v_pixels1);
        assert_eq!(k_v_pixels0[TEST_PIXEL_INDEX], 0.3251953);
    }

    #[test]
    fn emissivity() {
        // Another variance from the datasheet: added two more digits (75)
        assert_eq!(datasheet_eeprom().emissivity(), Some(0.94921875));
    }

    #[test]
    fn offset_reference_cp() {
        let e = datasheet_eeprom();
        // No difference between subpages
        assert_eq!(
            e.offset_reference_cp(Subpage::Zero),
            e.offset_reference_cp(Subpage::One)
        );
        assert_eq!(e.offset_reference_cp(Subpage::One), -119);
    }

    #[test]
    fn k_ta_cp() {
        let e = datasheet_eeprom();
        // k_ta doesn't vary on subpage
        assert_eq!(e.k_ta_cp(Subpage::Zero), e.k_ta_cp(Subpage::One));
        assert_eq!(e.k_ta_cp(Subpage::Zero), 0.0023193359);
    }

    #[test]
    fn k_v_cp() {
        let e = datasheet_eeprom();
        // Also no subpage difference here
        assert_eq!(e.k_v_cp(Subpage::Zero), e.k_v_cp(Subpage::One));
        assert_eq!(e.k_v_cp(Subpage::Zero), 0.3125);
    }

    #[test]
    fn temperature_gradient_coefficient() {
        assert_eq!(
            datasheet_eeprom().temperature_gradient_coefficient(),
            Some(0f32)
        );
    }

    #[test]
    fn alpha_cp() {
        let e = datasheet_eeprom();
        // MLX90641 doesn't vary on subpage
        let expected = 3.01952240988612E-9;
        assert_eq!(e.alpha_cp(Subpage::Zero), e.alpha_cp(Subpage::One));
        assert_eq!(e.alpha_cp(Subpage::One), expected);
    }

    #[test]
    fn k_s_ta() {
        assert_eq!(datasheet_eeprom().k_s_ta(), -0.002197265625);
    }

    #[test]
    fn pixel_alpha() {
        let e = datasheet_eeprom();
        let alpha0: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.alpha_pixels(Subpage::One).copied().collect();
        let alpha1: ArrayVec<f32, { Mlx90641::NUM_PIXELS }> =
            e.alpha_pixels(Subpage::Zero).copied().collect();
        // MLX90641 doesn't vary alpha on subpage
        assert_eq!(alpha0, alpha1);
        let pixel = alpha0[TEST_PIXEL_INDEX];
        assert_eq!(pixel, 3.45520675182343E-7);
    }

    #[test]
    fn k_s_to() {
        let e = datasheet_eeprom();
        // The example is a bit lazy here
        // More adding precision here: the last digit (used to be 7) was replaced by "695"
        let expected: [f32; NUM_CORNER_TEMPERATURES] = [-0.00069999695; NUM_CORNER_TEMPERATURES];
        for (actual, expected) in e.k_s_to().iter().zip(expected.iter()) {
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn corner_temperatures() {
        let e = datasheet_eeprom();
        let ct = e.corner_temperatures();
        assert_eq!(ct.len(), super::NUM_CORNER_TEMPERATURES);
        // The first five values are hard-coded, but testing for completeness.
        assert_eq!(ct[0], -40);
        assert_eq!(ct[1], -20);
        assert_eq!(ct[2], 0);
        assert_eq!(ct[3], 80);
        assert_eq!(ct[4], 120);
        // These are loaded from EEPROM
        assert_eq!(ct[5], 200);
        assert_eq!(ct[6], 400);
        assert_eq!(ct[7], 600);
    }
}
