// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::convert::TryInto;

use embedded_hal::blocking::i2c;

use crate::common::{Address, CalibrationData, MelexisCamera};
use crate::register::Subpage;

/// Constant needed a few times for the final pixel temperature calculations.
const KELVINS_TO_CELSIUS: f32 = 273.15;

fn delta_v<'a, Clb: CalibrationData<'a>>(calibration: &'a Clb, v_dd_pixel: i16) -> f32 {
    f32::from(v_dd_pixel - calibration.v_dd_25()) / f32::from(calibration.k_v_dd())
}

fn v_dd<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    resolution_correction: f32,
    delta_v: f32,
) -> f32 {
    delta_v * resolution_correction + calibration.v_dd_0()
}

fn v_ptat_art<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    t_a_ptat: i16,
    t_a_v_be: i16,
) -> f32 {
    // Use i32 to prevent overflowing (which *will* happen if you stay as i16, these values
    // are large enough).
    let denom = t_a_ptat as i32 * calibration.alpha_ptat() as i32 + t_a_v_be as i32;
    // Take the loss in precision when forcing a conversion to f32.
    f32::from(t_a_ptat) / denom as f32 * 18f32.exp2()
}

/// Calculate the ambient temperature, and cache it for later use.
fn ambient_temperature<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    v_ptat_art: f32,
    delta_v: f32,
) -> f32 {
    // Should probably change these in Calibration so they're already floats
    let k_v_ptat = f32::from(calibration.k_v_ptat()) / 12f32.exp2();
    let v_ptat_25 = f32::from(calibration.v_ptat_25());
    let numerator = (v_ptat_art / (1f32 + k_v_ptat * delta_v)) - v_ptat_25;
    let k_t_ptat = f32::from(calibration.k_t_ptat()) / 3f32.exp2();
    numerator / k_t_ptat + 25f32
}

pub struct RamData {
    t_a_v_be: i16,
    t_a_ptat: i16,
    v_dd_pixel: i16,
    gain: i16,
    compensation_pixel: i16,
}

impl RamData {
    /// Read a value from the camera's RAM.
    ///
    /// All values in RAM are signed 16-bit integers, so this function also converts the raw values
    /// into `i16`.
    fn read_ram_value<I2C>(
        bus: &mut I2C,
        i2c_address: u8,
        ram_address: Address,
    ) -> Result<i16, I2C::Error>
    where
        I2C: i2c::WriteRead,
    {
        let address_bytes = ram_address.as_bytes();
        let mut scratch = [0u8; 2];
        bus.write_read(i2c_address, &address_bytes[..], &mut scratch[..])?;
        Ok(i16::from_be_bytes(scratch))
    }

    /// Read the non-pixel data from the specified camera over I²C.
    pub fn from_i2c<I2C, Cam>(
        bus: &mut I2C,
        i2c_address: u8,
        subpage: Subpage,
    ) -> Result<Self, I2C::Error>
    where
        I2C: i2c::WriteRead,
        Cam: MelexisCamera,
    {
        let t_a_v_be = Self::read_ram_value(bus, i2c_address, Cam::t_a_v_be())?;
        let t_a_ptat = Self::read_ram_value(bus, i2c_address, Cam::t_a_ptat())?;
        let v_dd_pixel = Self::read_ram_value(bus, i2c_address, Cam::v_dd_pixel())?;
        let gain = Self::read_ram_value(bus, i2c_address, Cam::gain())?;
        let compensation_pixel =
            Self::read_ram_value(bus, i2c_address, Cam::compensation_pixel(subpage))?;
        Ok(Self {
            t_a_v_be,
            t_a_ptat,
            v_dd_pixel,
            gain,
            compensation_pixel,
        })
    }
}

/// Values that're common to all pixels when calculating raw IR values.
#[derive(Debug, PartialEq)]
pub struct CommonIrData {
    gain: f32,
    v_dd: f32,
    emissivity: f32,
    t_a: f32,
}

impl CommonIrData {
    pub fn new<'a, Clb>(
        resolution_correction: f32,
        emissivity: f32,
        calibration: &'a Clb,
        ram: &RamData,
    ) -> Self
    where
        Clb: CalibrationData<'a>,
    {
        let delta_v = delta_v(calibration, ram.v_dd_pixel);
        let v_dd = v_dd(calibration, resolution_correction, delta_v);
        // Labelled V_PTAT in the formulas, but T_a_PTAT in the memory map.
        let v_ptat_art = v_ptat_art(calibration, ram.t_a_ptat, ram.t_a_v_be);
        let t_a = ambient_temperature(calibration, v_ptat_art, delta_v);
        let gain = f32::from(calibration.gain()) / f32::from(ram.gain);
        Self {
            gain,
            v_dd,
            emissivity,
            t_a,
        }
    }
}

/// The per-pixel calculations to get a raw measurement of infrared radiation.
fn per_pixel_v_ir(
    pixel_data: i16,
    common: &CommonIrData,
    reference_offset: i16,
    k_v: f32,
    k_ta: f32,
) -> f32 {
    let pixel_gain = f32::from(pixel_data) * common.gain;
    let mut pixel_offset = pixel_gain
        - f32::from(reference_offset)
            * (1f32 + k_ta * (common.t_a - 25f32))
            * (1f32 + k_v * (common.v_dd - 3.3f32));
    pixel_offset /= common.emissivity;
    pixel_offset
}

/// Generate and copy a "raw" thermal image for the given subpage into the provided slice.
///
/// This is one of the fundamental functions in this crate. It reads the raw data off of the
/// camera, then copies it into the provided buffer (but only the pixels for the given
/// subpage!). The copied data is the raw infrared measurement (in volts I think) from the
/// camera, and has not been compensated for pixel sensitivity or ambient temperature. It *is*
/// useful though for applications where just an "image" is needed but the actual temperatures
/// are not.
#[allow(clippy::too_many_arguments)]
pub fn raw_pixels_to_ir_data<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    resolution_correction: f32,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [f32],
) -> f32
where
    Clb: CalibrationData<'a>,
    Px: Iterator<Item = bool>,
{
    // Knock out the values common to all pixels first.
    let common = CommonIrData::new(resolution_correction, emissivity, calibration, &ram);
    // Compensation pixels are only used if temperature gradient compensation is being used.
    let compensation_pixel_offset = calibration.temperature_gradient_coefficient().map(|tgc| {
        // TODO: There's a note in the datasheet advising a moving average filter (length >=
        // 16) on the compensation pixel gain.
        let compensation_pixel_offset = per_pixel_v_ir(
            ram.compensation_pixel,
            &common,
            calibration.offset_reference_cp(subpage),
            calibration.k_v_cp(subpage),
            calibration.k_ta_cp(subpage),
        );
        // Premultiplying by the TGC here
        tgc * compensation_pixel_offset
    });
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    destination
        .iter_mut()
        // Chunk into two byte segments, for each 16-bit value
        .zip(pixel_data.chunks_exact(2))
        // Zip up the corresponding values from the calibration data.
        // Skipping alpha (sensitivity) as that's more related to temperature calculations
        .zip(calibration.offset_reference_pixels(subpage))
        .zip(calibration.k_v_pixels(subpage))
        .zip(calibration.k_ta_pixels(subpage))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        // feeling a little lispy in here with all these parentheses
        .for_each(|((((output, pixel_slice), reference_offset), k_v), k_ta)| {
            // Safe to unwrap as this is from chunks_exact(2)
            let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
            let pixel_data = i16::from_be_bytes(pixel_bytes);
            let mut pixel_offset =
                per_pixel_v_ir(pixel_data, &common, *reference_offset, *k_v, *k_ta);
            // I hope the branch predictor/compiler is smart enough to realize there's
            // almost always going to be only one hot branch here
            if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                pixel_offset -= compensation_pixel_offset;
            }
            *output = pixel_offset;
        });
    common.t_a
}

fn t_ar(t_a: f32, t_r: f32, emissivity: f32) -> f32 {
    // Again, start with the steps common to all pixels
    let t_a_k4 = (t_a + KELVINS_TO_CELSIUS).powi(4);
    let t_r_k4 = (t_r + KELVINS_TO_CELSIUS).powi(4);
    t_r_k4 - ((t_r_k4 - t_a_k4) / emissivity)
}

/// Calculate K<sub>s<sub>T<sub>o</sub></sub></sub> and the sensitivity correction coefficient.
///
/// The sensitivity correction coefficient isn't named as such in the datasheet, it's just been
/// extracted from the "Normalizing to sensitivity" section. It is equal to 1 +
/// K<sub>s<sub>T<aub>a</sub></sub></sub> * (T<sub>a</sub> - T<sub>a<sub>0)
fn basic_k_s_t<'a, Clb>(calibration: &'a Clb, t_a: f32) -> (f32, f32)
where
    Clb: CalibrationData<'a>,
{
    let basic_index = calibration.basic_range();
    let k_s_to = calibration.k_s_to();
    let k_s_to_basic = k_s_to[basic_index];
    let k_s_ta = calibration.k_s_ta();
    let t_a0 = 25f32;
    // Little bit of optimization; this factor is shared by all pixels
    let alpha_coefficient = 1f32 + k_s_ta * (t_a - t_a0);
    (k_s_to_basic, alpha_coefficient)
}

/// The per-pixel calculations to go from a raw measurement to a temperature.
fn per_pixel_temperature(v_ir: f32, alpha: f32, t_ar: f32, k_s_to: f32) -> f32 {
    // This function is a mess of raising floats to the third and fourth powers, doing some
    // operations, then taking the fourth root of everything.
    let s_x = k_s_to * (alpha.powi(3) * v_ir + alpha.powi(4) * t_ar).powf(0.25);

    let t_o_root = (v_ir / (alpha * (1f32 - k_s_to * KELVINS_TO_CELSIUS) + s_x) + t_ar).powf(0.25);
    t_o_root - KELVINS_TO_CELSIUS
}

pub fn raw_ir_to_temperatures<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    t_a: f32,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [f32],
) where
    Clb: CalibrationData<'a>,
    Px: Iterator<Item = bool>,
{
    // TODO: this step could probably be optimized a little bit (if needed) by pushing this
    // calculation to the calibration loading step.
    let alpha_compensation_pixel = calibration
        .temperature_gradient_coefficient()
        .map(|tgc| calibration.alpha_cp(subpage) * tgc);
    let (k_s_to_basic, alpha_coefficient) = basic_k_s_t(calibration, t_a);
    // TODO: design a way to provide T-r, the reflected temperature. Basically, the temperature
    // of the surrounding environment (but not T_a, which is basically the temperature of the
    // sensor itself). For now hard-coding this to 8 degrees lower than T_a.
    let t_r = t_a - 8.0;
    let t_ar = t_ar(t_a, t_r, emissivity);

    destination
        .iter_mut()
        .zip(calibration.alpha_pixels(subpage))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        .for_each(|(output, alpha)| {
            let v_ir = *output;
            let compensated_alpha = match alpha_compensation_pixel {
                Some(alpha_compensation_pixel) => alpha - alpha_compensation_pixel,
                None => *alpha,
            } * alpha_coefficient;
            *output = per_pixel_temperature(v_ir, compensated_alpha, t_ar, k_s_to_basic);
        });
}

#[allow(clippy::too_many_arguments)]
pub fn raw_pixels_to_temperatures<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    resolution_correction: f32,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [f32],
) -> f32
where
    Clb: CalibrationData<'a>,
    Px: Iterator<Item = bool>,
{
    // Knock out the values common to all pixels first.
    let common = CommonIrData::new(resolution_correction, emissivity, calibration, &ram);
    // Compensation pixels are only used if temperature gradient compensation is being used.
    let compensation_pixel_offset = calibration.temperature_gradient_coefficient().map(|tgc| {
        // TODO: There's a note in the datasheet advising a moving average filter (length >=
        // 16) on the compensation pixel gain.
        let compensation_pixel_offset = per_pixel_v_ir(
            ram.compensation_pixel,
            &common,
            calibration.offset_reference_cp(subpage),
            calibration.k_v_cp(subpage),
            calibration.k_ta_cp(subpage),
        );
        // Premultiplying by the TGC here
        tgc * compensation_pixel_offset
    });
    // TODO: this step could probably be optimized a little bit (if needed) by pushing this
    // calculation to the calibration loading step.
    let alpha_compensation_pixel = calibration
        .temperature_gradient_coefficient()
        .map(|tgc| calibration.alpha_cp(subpage) * tgc);
    let (k_s_to_basic, alpha_coefficient) = basic_k_s_t(calibration, common.t_a);
    // TODO: design a way to provide T-r, the reflected temperature. Basically, the temperature
    // of the surrounding environment (but not T_a, which is basically the temperature of the
    // sensor itself). For now hard-coding this to 8 degrees lower than T_a.
    let t_r = common.t_a - 8.0;
    let t_ar = t_ar(common.t_a, t_r, emissivity);
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    destination
        .iter_mut()
        // Chunk into two byte segments, for each 16-bit value
        .zip(pixel_data.chunks_exact(2))
        // Zip up the corresponding values from the calibration data.
        // Skipping alpha (sensitivity) as that's more related to temperature calculations
        .zip(calibration.offset_reference_pixels(subpage))
        .zip(calibration.k_v_pixels(subpage))
        .zip(calibration.k_ta_pixels(subpage))
        .zip(calibration.alpha_pixels(subpage))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        // feeling a little lispy in here with all these parentheses
        .for_each(
            |(((((output, pixel_slice), reference_offset), k_v), k_ta), alpha)| {
                // Safe to unwrap as this is from chunks_exact(2)
                let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
                let pixel_data = i16::from_be_bytes(pixel_bytes);
                let mut v_ir = per_pixel_v_ir(pixel_data, &common, *reference_offset, *k_v, *k_ta);
                // I hope the branch predictor/compiler is smart enough to realize there's
                // almost always going to be only one hot branch here
                if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                    v_ir -= compensation_pixel_offset;
                }
                let compensated_alpha = match alpha_compensation_pixel {
                    Some(alpha_compensation_pixel) => alpha - alpha_compensation_pixel,
                    None => *alpha,
                } * alpha_coefficient;
                *output = per_pixel_temperature(v_ir, compensated_alpha, t_ar, k_s_to_basic);
            },
        );
    common.t_a
}

#[cfg(test)]
mod test {
    use float_cmp::{approx_eq, F32Margin};

    use crate::test::mlx90640_eeprom_data;
    use crate::{mlx90640, CalibrationData, Subpage};

    fn mlx90640_calibration() -> mlx90640::Mlx90640Calibration {
        let eeprom_data = mlx90640_eeprom_data();
        mlx90640::Mlx90640Calibration::from_data(&eeprom_data)
            .expect("Mlx90640Calibration should be able to be created from the example data")
    }

    // The super bare, single function tests with magic number are using values from the worked
    // example in the MLX90640 datasheet.

    #[test]
    fn delta_v() {
        let clb = mlx90640_calibration();
        assert_eq!(super::delta_v(&clb, -13115), 0.018623737)
    }

    #[test]
    fn v_dd() {
        let clb = mlx90640_calibration();
        let resolution_correction = 0.5;
        // Datasheet has ≈3.319
        approx_eq!(
            f32,
            super::v_dd(&clb, resolution_correction, 0.018623737),
            3.3186,
            F32Margin::default()
        );
    }

    #[test]
    fn v_ptat_art() {
        let clb = mlx90640_calibration();
        assert_eq!(super::v_ptat_art(&clb, 1711, 19442), 12873.57952);
    }

    #[test]
    fn t_a() {
        let clb = mlx90640_calibration();
        let delta_v = super::delta_v(&clb, -13115);
        let v_ptat_art = super::v_ptat_art(&clb, 1711, 19442);
        // The datasheet is a bit more precise than I can get with f32, so approx_eq here
        approx_eq!(
            f32,
            super::ambient_temperature(&clb, v_ptat_art, delta_v),
            39.18440152,
            epsilon = 0.000002
        );
    }

    #[test]
    // also called "pixel offset" in a few places
    fn v_ir() {
        let clb = mlx90640_calibration();
        // The worked example uses pixel(12, 16) (and we use 0-indexing, so subtract one) and
        // subpage 1
        let pixel_index = 11 * mlx90640::WIDTH + 15;
        let offset = clb
            .offset_reference_pixels(Subpage::One)
            .nth(pixel_index)
            .unwrap();
        let k_v = clb.k_v_pixels(Subpage::One).nth(pixel_index).unwrap();
        let k_ta = clb.k_ta_pixels(Subpage::One).nth(pixel_index).unwrap();
        // Taken from the worked example
        let common = super::CommonIrData {
            gain: 1.01753546947234,
            v_dd: 3.319,
            emissivity: 1.0,
            t_a: 39.18440152,
        };
        let raw_pixel: i16 = 609;
        let v_ir = super::per_pixel_v_ir(raw_pixel, &common, *offset, *k_v, *k_ta);
        // I'm getting 700.89, which is close enough considering how many places to lose precision
        // there are in this step.
        approx_eq!(f32, v_ir, 700.882495690866, epsilon = 0.01);
    }

    #[test]
    fn pixel_temperature() {
        let clb = mlx90640_calibration();
        let basic_index = clb.basic_range();
        let k_s_to = clb.k_s_to()[basic_index];
        // The worked example is using TGC, which is done before per_pixel_temparature() is called,
        // so these values are hard-coded from the datasheet.
        let alpha = 1.1876487360496E-7;
        // Pile of values from previous steps.
        let v_ir = 679.250909123826;
        let t_ar = 9516495632.56;
        let t_o = super::per_pixel_temperature(v_ir, alpha, t_ar, k_s_to);
        assert_eq!(t_o, 80.36331);
    }
}
