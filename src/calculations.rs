// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

//! Calculations for turning sensor output into temperatures
//!
#![doc = include_str!("katex.html")]
//!
//! The MLX90640 and MLX90641 datasheets have roughly a third of their pages dedicated to
//! mathematical formulas, which can be a little intimidating. Fortunately most of the formulas
//! can be simplified by assuming that treating some raw bits as a signed integer is a "free"
//! operation, and that shifting bits to the left or right is simpler to write using *<<* and *>>*.
//! In other words:
//! * If you see an operation similar to
//!   $$
//!   \mathrm{If } K_{Foo} \gt (2^n - 1) \to K_{Foo} = K_{Foo} - (2^{n + 1})
//!   $$
//!   It is converting an unsigned integer to a signed one. The portions in parentheses are
//!   typically written as the actual value instead of a formula, so for example instead of
//!   *2<sup>7</sup>&nbsp;-&nbsp;1* and *2<sup>8</sup>*, 127 and 256 will be used.
//! * Masking a value off with a logical AND, followed by a division by a power of two. This is
//!   just extracting some of the bits out of a larger word (the [word size] for these cameras is
//!   16 bits).
//! * Building off of the previous item, multiplication and division by powers of two are used
//!   extensively in the datasheet to perform operations that would typically be written as a
//!   [logical bit shift] to the left (division) or right (multiplication). Be aware that not all
//!   divisions by a power of two can be rewritten as bit shifts; sometimes they're converting a
//!   [fixed-point] value to a floating-point one.
//!
//! [logical bit shift]: https://en.wikipedia.org/wiki/Logical_shift
//! [fixed-point]: https://en.wikipedia.org/wiki/Fixed-point_arithmetic
//! [word size]: https://en.wikipedia.org/wiki/Word_(computer_architecture)
//!
//! An example of these points from section 11.1.1 of the MLX90640 datasheet:
//!
//! \\begin{align*}
//! K_{Vdd} =& \frac{\eeprom{0x2433} \And \text{ 0xFF00}}{2^8} \newline
//! &\mathrm{If } K_{Vdd} \gt 127 \to K_{Vdd} = K_{Vdd} - 256
//! \\end{align*}
//!
//! Can be written in Rust as:
//!
//! ```no_run
//! fn read_eeprom(address: u16) -> u16 {
//!     unimplemented!();
//! }
//! let raw_k_v_dd = ((read_eeprom(0x2433) & 0xFF00) >> 8) as u8;
//! let k_v_dd = i8::from_be_bytes(raw_k_v_dd.to_be_bytes());
//! ```
//! Non-standard integer widths (like 4 or 10) can easily be converted to signed integers with
//! [sign extension].
//!
//! [sign extension]: https://en.wikipedia.org/wiki/Sign_extension
//!
//! # Glossary
//! <dl>
//! <dt>
//! $\alpha$, alpha
//! </dt><dd>
//! Sensitivity coefficient.
//! </dd>
//! <dt>
//! <var>CP</var>
//! </dt><dd>
//! Compensation pixel.
//! </dd>
//! <dt>
//! $\varepsilon$, emissivity
//! </dt><dd>
//! A measure of how well a material emits IR radiation. For a better explanation, see
//! <a href="https://en.wikipedia.org/wiki/Emissivity">Wikipedia</a>.
//! </dd>
//! <dt>
//! <var>K</var>
//! </dt><dd>
//! Prefix for constants.
//! </dd>
//! <dt>
//! PTAT
//! </dt><dd>
//! Proportional to ambient temperature
//! </dd>
//! <dt>
//! $T_a$
//! </dt><dd>
//! Ambient temperature of the camera.
//! </dd>
//! <dt>
//! $T_{a_{0}}$
//! </dt><dd>
//! Ambient temperature reference, 25.0 ℃. If it looks like 0, it's probably "o" as this value is
//! really only used in one place.
//! </dd>
//! <dt>
//! $T_o$
//! </dt><dd>
//! Object temperature, meaning the temperature an individual pixel has detected for an object.
//! </dd>
//! <dt>
//! $V_{DD}$
//! </dt><dd>
//! Pixel supply voltage.
//! </dd>
//! <dt>
//! $V_{DD_{25}}$
//! </dt><dd>
//! Pixel supply voltage reference at 25.0 ℃.
//! </dd>
//! </dl>

use core::convert::TryInto;

use embedded_hal::blocking::i2c;

// Various floating point operations are not implemented in core, so we use libm to provide them as
// needed.
#[cfg_attr(feature = "std", allow(unused_imports))]
use num_traits::Float;

use crate::common::{Address, CalibrationData, MelexisCamera};
use crate::register::Subpage;
use crate::AccessPattern;

/// Constant needed a few times for the final pixel temperature calculations.
const KELVINS_TO_CELSIUS: f32 = 273.15;

/// Calculate $\Delta V$
///
/// This is part of the process for calculating the current ambient temperature ($T_a$)
/// and is calculated once per frame. It is documented in sections 11.1.2 and 11.2.2.3 in both
/// datasheets. This function is used for constructing [`CommonIrData`].
///
/// $$
/// \Delta V = \frac{V_{DD_{pix}} - V_{DD_{25}}}{K_{V_{DD}}}
/// $$
///
/// The constants $V_{DD_{25}}$ and $K_{V_{DD}}$ are retrieved from the `calibration` argument,
/// while $V_{DD_{pix}}$ (`v_dd_pixel`) is read from the camera's RAM.
#[doc = include_str!("katex.html")]
pub fn delta_v<'a, Clb: CalibrationData<'a>>(calibration: &'a Clb, v_dd_pixel: i16) -> f32 {
    f32::from(v_dd_pixel - calibration.v_dd_25()) / f32::from(calibration.k_v_dd())
}

/// Calculate $V_{DD}$
///
/// The output of this function is used in multiple places in calculating the temperature of each
/// pixel, and only needs to be calculated once for each frame. It is documented in sections
/// 11.1.17, 11.2.2.1, and 11.2.2.2 in the MLX90640 datasheet and 11.1.1, 11.1.18, 11.2.2.1, and
/// 11.2.2.2 in the MLX90641 datasheet. This function is used for constructing [`CommonIrData`].
///
/// \begin{align*}
/// V\_{DD} &= \frac{\text{Resolution}\_\text{corr} \* V\_{DD\_{pix}} - V\_{DD\_{25}}}{K\_{V\_{DD}}} +
///          V\_{DD\_0}\newline
///         &= \text{Resolution}\_\text{corr} * \Delta V + V\_{DD\_0}
/// \end{align*}
///
/// $V_{DD_{0}}$ is retrieved from the `calibration` argument, while $\Delta V$ (`delta_v`) is the
/// value returned from [`delta_v`]. The resolution correction factor calculation is camera model
/// specific, and is performed by an implementation of
/// [`MelexisCamera::resolution_correction`][mlx-cam-res].
///
/// [mlx-cam-res]: crate::common::MelexisCamera::resolution_correction
#[doc = include_str!("katex.html")]
pub fn v_dd<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    resolution_correction: f32,
    delta_v: f32,
) -> f32 {
    delta_v * resolution_correction + calibration.v_dd_0()
}

/// Calculate $V\_{PTAT\_{art}}$
///
/// This is part of the process in calculating the current ambient temperature ($T_{a}$)
/// and is calculated once per frame. It is documented in sections 11.1.2 and 11.2.2.3 in both
/// datasheets. It is used for constructing [`CommonIrData`].
///
/// $$
/// V_{PTAT_{art}} = \frac{T_{a_{PTAT}}}{T_{a_{PTAT}} \* \text{Alpha}\_{PTAT} + T\_{a_{V_{BE}}}} \* 2^{18}
/// $$
///
/// $Alpha_{PTAT}$ is retrieved from the `calibration` argument, while $T_{a_{PTAT}}$ (`t_a_ptat`)
/// and $T_{a_{V_{BE}}}$ (`t_a_v_be`) are read from the camera's RAM.
#[doc = include_str!("katex.html")]
pub fn v_ptat_art<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    t_a_ptat: i16,
    t_a_v_be: i16,
) -> f32 {
    let denom = f32::from(t_a_ptat) * calibration.alpha_ptat() + f32::from(t_a_v_be);
    // Take the loss in precision when forcing a conversion to f32.
    f32::from(t_a_ptat) / denom * 18f32.exp2()
}

/// Calculate the ambient temperature ($T_a$)
///
/// The ambient temperature is used when calculating temperature of each pixel, and is also a
/// useful value by itself. It is calculated once per frame, and is used when constructing
/// [`CommonIrData`]. It is documented in sections 11.1.2 and 11.2.2.3 in both datasheets.
///
/// $$
/// T_a = \frac{\frac{V_{PTAT_{art}}}{1 + K_{V_{PTAT}} \* \Delta V} - V_{PTAT_{25}}}{K_{T_{PTAT}}} + 25
/// $$
///
/// $K_{V_{PTAT}}$, $K_{T_{PTAT}}$, and
/// $V_{PTAT_{25}}$, are taken from `calibration`, while
/// $V_{PTAT_{art}}$ (`v_ptat_art`) and $\Delta V$ (`delta_v`) are the results of
/// [`v_ptat_art`] and [`delta_v`] respectively.
#[doc = include_str!("katex.html")]
pub fn ambient_temperature<'a, Clb: CalibrationData<'a>>(
    calibration: &'a Clb,
    v_ptat_art: f32,
    delta_v: f32,
) -> f32 {
    let v_ptat_25 = calibration.v_ptat_25();
    let numerator = (v_ptat_art / (1f32 + calibration.k_v_ptat() * delta_v)) - v_ptat_25;
    numerator / calibration.k_t_ptat() + 25f32
}

/// The non-pixel values read from the camera's RAM for each frame.
///
/// This structure is the non-EEPROM, non-register input when [creating
/// `CommonIrData`][CommonIrData::new].
#[doc = include_str!("katex.html")]
#[derive(Clone, Copy, Debug)]
pub struct RamData {
    /// $T_{a_{V_{BE}}}$
    ///
    /// This value is labelled as $T_{a_{V_{BE}}}$ on the EEPROM map, but is labelled $V_{BE}$
    /// elsewhere in the datasheet.
    pub t_a_v_be: i16,

    /// $T_{a_{PTAT}}$
    ///
    /// This value is labelled as $T_{a_{PTAT}}$ on the EEPROM map, but is labelled $V_{PTAT}$
    /// elsewhere in the datasheet.
    pub t_a_ptat: i16,

    /// $V_{DD_{pix}}$
    pub v_dd_pixel: i16,

    /// The gain value for the current frame.
    pub gain: i16,

    /// The compensation pixel for the current frame.
    pub compensation_pixel: i16,
}

#[doc = include_str!("katex.html")]
impl RamData {
    /// Read a value from the camera's RAM.
    ///
    /// All values in RAM are signed 16-bit integers, so this function also converts the raw values
    /// into [`i16`].
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

    /// Read the non-pixel values from the specified camera over I²C
    ///
    /// The non-pixel values are $T_{a_{V_{BE}}}$,
    /// $T_{a_{PTAT}}$, $V_{DD_{pix}}$, gain and the corresponding compensation pixel for the given
    /// subpage.
    pub fn from_i2c<I2C, Cam>(
        bus: &mut I2C,
        i2c_address: u8,
        subpage: Subpage,
    ) -> Result<Self, I2C::Error>
    where
        I2C: i2c::WriteRead,
        Cam: MelexisCamera,
    {
        let t_a_v_be = Self::read_ram_value(bus, i2c_address, Cam::T_A_V_BE)?;
        let t_a_ptat = Self::read_ram_value(bus, i2c_address, Cam::T_A_PTAT)?;
        let v_dd_pixel = Self::read_ram_value(bus, i2c_address, Cam::V_DD_PIXEL)?;
        let gain = Self::read_ram_value(bus, i2c_address, Cam::GAIN)?;
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

/// Values that are common to all pixels in a given frame.
///
/// These values only need to be calculated once per frame and are then shared for the per-pixel
/// calculations later.
#[doc = include_str!("katex.html")]
#[derive(Debug, PartialEq)]
pub struct CommonIrData {
    /// The gain parameter ($K_{gain}$)
    ///
    /// The raw data from each pixel is first multipled by the "gain parameter" before further
    /// processing. The "gain parameter" is calculated by dividing the value read from RAM by the
    /// "gain coefficient" that is calculated from the calibration EEPROM. The parameter is
    /// documented in section 12.2.2.4 in both datasheets, while the coefficient is documented in
    /// section 11.1.7 in both datasheets.
    pub gain: f32,

    /// The pixel supply voltage ($V_{dd}$)
    ///
    /// This is calculated using the [`delta_v`] and [`v_dd`] functions.
    pub v_dd: f32,

    /// The [emissivity] ($\varepsilon$) to use when calculating the pixel temperatures.
    ///
    /// The MLX90641 can store an emissivity value in EEPROM, but the MLX90640 does not and a value
    /// must be provided. The high-level API defaults to the value in EEPROM, or 1.0 if that is not
    /// available.
    ///
    /// [emissivity]: https://en.wikipedia.org/wiki/Emissivity
    pub emissivity: f32,

    /// The ambient temperature ($T_{a}$)
    ///
    /// The name of this value is slightly inaccurate as it's not so much the ambient temperature
    /// of the air surrounding the camera, but the temperature of the sensor itself, which is
    /// typically a few degrees warmer than the surrounding air. This value is calculated using
    /// [`ambient_temperature`].
    pub t_a: f32,
}

impl CommonIrData {
    /// Create a new `CommonIrData`.
    ///
    /// The three base sources of data for the fields in this structure are the calibration EEPROM
    /// (`calibration`, `emissivity` in some cases, and half of `resolution_correction`), the
    /// device's control register (the other half of `resolution_correction`), and the current
    /// frame's RAM.
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
        let gain = calibration.gain() / f32::from(ram.gain);
        Self {
            gain,
            v_dd,
            emissivity,
            t_a,
        }
    }
}

/// Calculate a measurement of raw IR captured by a single pixel
///
/// This function combines all per-pixel calculations related to the raw IR data as described by
/// section 11.2.2.5 in both datasheets. If just an "image" is required but not the actual
/// temperature, the datasheets recommend stopping at this step instead of continuing on with the
/// calculations performed by [`per_pixel_temperature`].
///
/// \begin{align*}
/// \text{pix}\_{gain(i, j)} &= \text{[Pixel value from RAM]} * K_{gain} \newline
/// \text{pix}\_{OS(i, j)} &= pix\_{gain(i, j)} \newline
/// &\hspace{4em}- \textit{offset}\_{(i, j)} \newline
/// &\hspace{4em}\* (1 + K_{T\_{a}(i, j)} \* (T_a - T_{a_0})) \newline
/// &\hspace{4em}\* (1 + K_{V(i, j)} \* (V_{DD} - V_{DD_0})) \newline
/// V\_{IR(i, j)} &= \frac{pix\_{OS(i, j)}}{\varepsilon}
/// \end{align*}
///
/// [`CalibrationData`] provides the per-pixel [offset], $K_{V}$ ([`k_v_pixels`]), and $K_{T_{a}}$
/// ([`k_ta_pixels`]) values, while $K_{gain}$, $V_{DD}$, $T_{a}$, and emissivity ($\varepsilon$)
/// are provided by [`CommonIrData`]. $V_{DD_0}$ and $T_{a_0}$ are defined as 3.3 and 25
/// respectively.
///
/// [offset]: CalibrationData::offset_reference_pixels
/// [`k_v_pixels`]: CalibrationData::k_v_pixels
/// [`k_ta_pixels`]: CalibrationData::k_ta_pixels
///
/// This function is also used for calculating the compensation pixel values when using thermal
/// gradient compensation.
#[doc = include_str!("katex.html")]
#[inline]
pub fn per_pixel_v_ir(
    pixel_data: i16,
    common: &CommonIrData,
    reference_offset: i16,
    k_v: f32,
    k_ta: f32,
    access_mode_compensation: Option<f32>,
) -> f32 {
    let pixel_gain = f32::from(pixel_data) * common.gain;
    let mut pixel_offset = pixel_gain
        - (f32::from(reference_offset) + access_mode_compensation.unwrap_or(0.0f32))
            * (1f32 + k_ta * (common.t_a - 25f32))
            * (1f32 + k_v * (common.v_dd - 3.3f32));
    pixel_offset /= common.emissivity;
    pixel_offset
}

/// Calculate a measurement of raw IR data for all pixels
///
/// This function applies [`per_pixel_v_ir`] to each pixel belonging to the given subpage, writing
/// the IR measurements to the `destination` argument. Thermal gradient compensation is also
/// applied if supported by the camera. The ambient temperature is returned as well. The units of
/// the IR data are not specified in the data sheet, while the ambient temperature is in degrees
/// Celsius.
///
/// The slices `pixel_data` and `destination` cover all of the camera pixels, but only the pixels
/// belonging to `subpage` are calculated. Determining which pixels belong to which subpage is done
/// with the `valid_pixels` iterator (see [`MelexisCamera::pixels_in_subpage`] for more details).
///
/// The datasheets suggest that stopping at this step (instead of continuing on with calculating
/// the temperatures of each pixel) can be appropriate if an "image" is all that is required (with
/// an example use case of machine vision).
#[allow(clippy::too_many_arguments)]
pub fn raw_pixels_to_ir_data<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    resolution_correction: f32,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    access_pattern: AccessPattern,
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
            calibration.access_pattern_compensation_cp(subpage, access_pattern),
        );
        // Premultiplying by the TGC here
        tgc * compensation_pixel_offset
    });
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    let access_mode_compensation = calibration
        .access_pattern_compensation_pixels(access_pattern)
        .map(|o| o.copied());
    destination
        .iter_mut()
        // Chunk into two byte segments, for each 16-bit value
        .zip(pixel_data.chunks_exact(2))
        // Zip up the corresponding values from the calibration data.
        // Skipping alpha (sensitivity) as that's more related to temperature calculations
        .zip(calibration.offset_reference_pixels(subpage))
        .zip(calibration.k_v_pixels(subpage))
        .zip(calibration.k_ta_pixels(subpage))
        .zip(access_mode_compensation)
        //.zip(calibration.access_pattern_compensation_pixels(access_pattern))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        // feeling a little lispy in here with all these parentheses
        .for_each(
            |(
                ((((output, pixel_slice), reference_offset), k_v), k_ta),
                access_mode_compensation,
            )| {
                // Safe to unwrap as this is from chunks_exact(2)
                let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
                let pixel_data = i16::from_be_bytes(pixel_bytes);
                let mut pixel_offset = per_pixel_v_ir(
                    pixel_data,
                    &common,
                    *reference_offset,
                    *k_v,
                    *k_ta,
                    access_mode_compensation,
                );
                // I hope the branch predictor/compiler is smart enough to realize there's
                // almost always going to be only one hot branch here
                if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                    pixel_offset -= compensation_pixel_offset;
                }
                *output = pixel_offset;
            },
        );
    common.t_a
}

/// Calculate $T_{a - r}$
///
/// $$
/// T_{aK4} = (T\_a + 273.15)^4 \newline
/// T_{rK4} = (T\_r + 273.15)^4 \newline
/// T_{a - r} = T_{rK4} - \frac{T_{rK4} - T_{aK4}}{\varepsilon}
/// $$
///
/// This is the part of the process for calculating $T_o$ (the temperature detected by a pixel)
/// that compensates for IR radiation that is *reflected* by the object being measured as
/// opposed to being emitted by the object. To do this the temperature of the surrounding space
/// (`t_r`) as well as the emissivity ($\varepsilon$) of the object are combined with the ambient
/// temperature of the sensor itself (`t_a`). This calculation is described in section 11.2.2.9 in
/// both datasheets.
#[doc = include_str!("katex.html")]
#[inline]
pub fn t_ar(t_a: f32, t_r: f32, emissivity: f32) -> f32 {
    // Again, start with the steps common to all pixels
    let t_a_k4 = (t_a + KELVINS_TO_CELSIUS).powi(4);
    // If the emissivity of an object is 1, it also absorbs all infrared radiation (see Kirchoff's
    // law of thermal radiation). In that case, there is no reflected radiation, so we can ignore
    // t_r.
    // Disabling the clippy list here as the emissivity value usually uses is going to be very
    // close to 1 anyways, so that comparing with a margin of error would be inaccurate. The exact
    // value "1.0" is also used as a default value.
    #[allow(clippy::float_cmp)]
    if emissivity == 1.0 {
        t_a_k4
    } else {
        let t_r_k4 = (t_r + KELVINS_TO_CELSIUS).powi(4);
        t_r_k4 - ((t_r_k4 - t_a_k4) / emissivity)
    }
}

/// Calculate the sensitivity correction coefficient
///
/// The sensitivity correction coefficient isn't explicitly named in the datasheet, but it's a
/// common factor used when calculating $T\_o$. In section 11.2.2.8 of the datasheet,
/// $1 + K\_{S\_{T\_a}} \* (T\_a - T\_{a\_0})$ is common to all pixel sensitivity calculation, so
/// calculating it once is done to improve performance.
#[doc = include_str!("katex.html")]
pub fn sensitivity_correction_coefficient<'a, Clb>(calibration: &'a Clb, t_a: f32) -> f32
where
    Clb: CalibrationData<'a>,
{
    let k_s_ta = calibration.k_s_ta();
    let t_a0 = 25f32;
    // Little bit of optimization; this factor is shared by all pixels
    1f32 + k_s_ta * (t_a - t_a0)
}

/// Calculate the temperature measured by a pixel from its raw IR measurement.
///
/// \begin{align*}
/// \alpha\_{\textit{comp}(i, j)} &= \begin{cases}
/// \alpha\_{(i, j)} \* \alpha\_{\textit{corr}} &\text{if not TGC} \newline
/// (\alpha\_{(i, j)} - TGC \* \alpha\_{CP}) \* \alpha\_{\textit{corr}} &\text{if TGC} \newline
/// \end{cases} \newline
/// S\_{x(i, j)} &= K\_{S\_{T\_o}} \* \sqrt\[4\]{{\alpha\_{\textit{comp}(i, j)}}^4 \* V\_{IR(i, j)} + {\alpha\_{(i,
/// j)}}^4 \* T\_{a - r}} \newline
/// T\_{o(i, j)} &= \sqrt\[4\]{\frac{V\_{IR(i, j)}}{\alpha\_{\textit{comp}(i, j)} \* (1 - K\_{S\_{T\_o}} \* 273.15) +
/// S\_{x(i, j)})} + T\_{a - r}} - 273.15
/// \end{align*}
///
/// This function takes the output of [`per_pixel_v_ir`] in `v_ir` ($V\_{IR}$) along with the
/// output of [`t_ar`] ($T\_{a - r}$), the pre-compensated sensitivity for the pixel (`alpha`,
/// $\alpha\_{\textit{comp}(i, j)}$), and the [sensitivity slope] for the [basic temperature
/// range][basic-range] (`k_s_to`, $K\_{S\_{T\_o}}$). The latter two formulas are described in
/// section 11.2.2.9 in both datasheets.
///
/// [sensitivity slope]: CalibrationData::k_s_to
/// [basic-range]: MelexisCamera::BASIC_TEMPERATURE_RANGE
///
/// $\alpha\_{(i, j)}$ is the [calibrated sensitivity][alpha-pixel] for the pixel. If the camera
/// supports a [temperature gradient coefficient][tgc] (TGC), it is multiplied by the [compensation
/// pixel's sensitivity][alpha-cp] ($\alpha\_{CP}$) for the current subpage and then subtracted
/// from the calibrated sensitivity. $\alpha\_{\textit{corr}}$ is the
/// [`sensitivity_correction_coefficient`] common to all pixels. This function does *not* perform
/// the TGC compensation, and expects `alpha` to be $\alpha\_{\textit{comp}(i, j)}$. This formula
/// is described in section 11.2.2.8 in both datasheets.
///
/// [alpha-pixel]: CalibrationData::alpha_pixels
/// [tgc]: CalibrationData::temperature_gradient_coefficient
/// [alpha-cp]: CalibrationData::alpha_cp
///
/// This function calculates the temperature for the basic temperature range only; extended
/// temperature range calculations are not implemented yet and will probably be done in a separate
/// function.
#[doc = include_str!("katex.html")]
#[inline]
pub fn per_pixel_temperature(v_ir: f32, alpha: f32, t_ar: f32, k_s_to: f32) -> f32 {
    // This function is a mess of raising floats to the third and fourth powers, doing some
    // operations, then taking the fourth root of everything.
    let s_x = k_s_to * (alpha.powi(3) * v_ir + alpha.powi(4) * t_ar).powf(0.25);
    let t_o_root = (v_ir / (alpha * (1f32 - k_s_to * KELVINS_TO_CELSIUS) + s_x) + t_ar).powf(0.25);
    t_o_root - KELVINS_TO_CELSIUS
}

/// Calculate the temperature for all pixels, starting from the raw IR data
///
/// This function applies [`per_pixel_temperature`] to all pixels in the given subpage. It modifies
/// the `destination` array in-place, replacing the $V\_{IR}$ data with the temperatures. The
/// ambient temperature (`t_a`, $T\_a$) needs to be given from the same frame that produced the IR
/// data.
#[doc = include_str!("katex.html")]
pub fn raw_ir_to_temperatures<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    t_a: f32,
    t_r: Option<f32>,
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
    let k_s_to_basic = calibration.k_s_to()[Clb::Camera::BASIC_TEMPERATURE_RANGE];
    let alpha_coefficient = sensitivity_correction_coefficient(calibration, t_a);
    let t_r = t_r.unwrap_or_else(|| t_a - Clb::Camera::SELF_HEATING);
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

/// Calculate the temperature from all pixels, starting with the raw data from the camera
///
/// This function combines [`raw_pixels_to_ir_data`] and [`raw_ir_to_temperatures`], performing all
/// per-pixel operations in a single pass.
#[allow(clippy::too_many_arguments)]
pub fn raw_pixels_to_temperatures<'a, Clb, Px>(
    calibration: &'a Clb,
    emissivity: f32,
    t_r: Option<f32>,
    resolution_correction: f32,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    access_pattern: AccessPattern,
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
            calibration.access_pattern_compensation_cp(subpage, access_pattern),
        );
        // Premultiplying by the TGC here
        tgc * compensation_pixel_offset
    });
    // TODO: this step could probably be optimized a little bit (if needed) by pushing this
    // calculation to the calibration loading step.
    let alpha_compensation_pixel = calibration
        .temperature_gradient_coefficient()
        .map(|tgc| calibration.alpha_cp(subpage) * tgc);
    let k_s_to_basic = calibration.k_s_to()[Clb::Camera::BASIC_TEMPERATURE_RANGE];
    let alpha_coefficient = sensitivity_correction_coefficient(calibration, common.t_a);
    let t_r = t_r.unwrap_or_else(|| common.t_a - Clb::Camera::SELF_HEATING);
    let t_ar = t_ar(common.t_a, t_r, emissivity);
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    let access_mode_compensation = calibration
        .access_pattern_compensation_pixels(access_pattern)
        .map(|o| o.copied());
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
        .zip(access_mode_compensation)
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        // feeling a little lispy in here with all these parentheses
        .for_each(
            |(
                (((((output, pixel_slice), reference_offset), k_v), k_ta), alpha),
                access_mode_compensation,
            )| {
                // Safe to unwrap as this is from chunks_exact(2)
                let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
                let pixel_data = i16::from_be_bytes(pixel_bytes);
                let mut v_ir = per_pixel_v_ir(
                    pixel_data,
                    &common,
                    *reference_offset,
                    *k_v,
                    *k_ta,
                    access_mode_compensation,
                );
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
#[allow(clippy::excessive_precision)]
mod test {
    use float_cmp::assert_approx_eq;

    use crate::test::{mlx90640_datasheet_eeprom, mlx90641_datasheet_eeprom};
    use crate::{mlx90640, mlx90641, CalibrationData, MelexisCamera, Subpage};

    fn mlx90640_calibration() -> mlx90640::Mlx90640Calibration {
        let eeprom_data = mlx90640_datasheet_eeprom();
        mlx90640::Mlx90640Calibration::from_data(&eeprom_data)
            .expect("Mlx90640Calibration should be able to be created from the example data")
    }

    fn mlx90641_calibration() -> mlx90641::Mlx90641Calibration {
        let eeprom_data = mlx90641_datasheet_eeprom();
        mlx90641::Mlx90641Calibration::from_data(&eeprom_data)
            .expect("Mlx90641Calibration should be able to be created from the example data")
    }

    // The super bare, single function tests with magic number are using values from the worked
    // example in the MLX90640 or MLX90641 datasheets.

    #[test]
    fn delta_v() {
        let clb_640 = mlx90640_calibration();
        let clb_641 = mlx90641_calibration();
        // Input argument is v_dd_pixel
        // The value from the 640 datasheet is rounded, but can be represented exactly by the
        // faction 59 / 3168.
        assert_eq!(super::delta_v(&clb_640, -13115), 59.0 / 3168.0);
        // Same deal as the value above, just the fraction this time is 138 / -3136
        assert_eq!(super::delta_v(&clb_641, -13430), 138.0 / -3136.0);
    }

    #[test]
    fn v_dd() {
        let clb_640 = mlx90640_calibration();
        let clb_641 = mlx90641_calibration();
        // The resolution correction is 1 in both datasheets
        let resolution_correction = 1.0;
        // MLX90640 datasheet has ≈3.319
        assert_approx_eq!(
            f32,
            super::v_dd(&clb_640, resolution_correction, 0.018623737),
            3.319,
            epsilon = 0.001
        );
        // MLX90641 datasheet has ≈ 3.25599
        assert_approx_eq!(
            f32,
            super::v_dd(&clb_641, resolution_correction, -0.0440051),
            3.25599,
            epsilon = 0.00001
        );
    }

    #[test]
    fn v_ptat_art() {
        let clb_640 = mlx90640_calibration();
        let clb_641 = mlx90641_calibration();
        // Inputs are t_a_ptat (aka v_ptat) and t_a_v_be
        assert_approx_eq!(
            f32,
            super::v_ptat_art(&clb_640, 1711, 19442),
            12873.57952,
            epsilon = 0.00001
        );
        assert_approx_eq!(
            f32,
            super::v_ptat_art(&clb_641, 1752, 19540),
            13007.71,
            epsilon = 0.01
        );
    }

    #[test]
    fn t_a() {
        let clb_640 = mlx90640_calibration();
        let clb_641 = mlx90641_calibration();
        let delta_v_640 = super::delta_v(&clb_640, -13115);
        let delta_v_641 = super::delta_v(&clb_641, -13430);
        let v_ptat_art_640 = super::v_ptat_art(&clb_640, 1711, 19442);
        let v_ptat_art_641 = super::v_ptat_art(&clb_641, 1752, 19540);
        // Both datasheets round the final value, but have extended precision on the same step, so
        // the expected value is using that extended precision.
        assert_approx_eq!(
            f32,
            super::ambient_temperature(&clb_640, v_ptat_art_640, delta_v_640),
            // Using the same numbers as the dataasheet, with an arbitrary precision calculator,
            // yields this value instead of the datasheet's value of 39.18440152
            39.18442383,
            epsilon = 0.00000001
        );
        assert_approx_eq!(
            f32,
            super::ambient_temperature(&clb_641, v_ptat_art_641, delta_v_641),
            // Same deal as above, the datasheet calculates 42.022 as T_a, but using the same
            // numbers they do, a calculator (that is *not* using floating point) yields this
            // value instead.
            42.09766048,
            epsilon = 0.001
        );
    }

    #[test]
    // also called "pixel offset" in a few places
    fn v_ir_640() {
        let clb = mlx90640_calibration();
        // The worked example uses pixel(12, 16) (and we use 0-indexing, so subtract one) and
        // subpage 1
        let pixel_index = 11 * mlx90640::Mlx90640::WIDTH + 15;
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
        let v_ir = super::per_pixel_v_ir(raw_pixel, &common, *offset, *k_v, *k_ta, None);
        // Same deal as t_a, performing the same calculations as the datasheet (in an arbitrary
        // precision calculator) results in a different value than is in the datasheet's example.
        // In this case the datasheet has 700.882495690866
        assert_approx_eq!(f32, v_ir, 700.8974671440075625);
    }

    #[test]
    // also called "pixel offset" in a few places
    fn v_ir_641() {
        let clb = mlx90641_calibration();
        // The worked example uses pixel(6, 9) (and we use 0-indexing, so subtract one) and
        // subpage 0
        let pixel_index = 5 * mlx90641::Mlx90641::WIDTH + 8;
        let offset = clb
            .offset_reference_pixels(Subpage::Zero)
            .nth(pixel_index)
            .unwrap();
        let k_v = clb.k_v_pixels(Subpage::Zero).nth(pixel_index).unwrap();
        let k_ta = clb.k_ta_pixels(Subpage::Zero).nth(pixel_index).unwrap();
        // Taken from the worked example
        let common = super::CommonIrData {
            gain: 1.02445038,
            v_dd: 3.25599,
            emissivity: 0.949218,
            t_a: 42.022,
        };
        let raw_pixel: i16 = 972;
        let v_ir = super::per_pixel_v_ir(raw_pixel, &common, *offset, *k_v, *k_ta, None);
        // 641 datasheet (in section 11.2.2.7) rounds 1784.78049 to 1785
        assert_approx_eq!(f32, v_ir, 1784.78049, epsilon = 0.01);
    }

    #[test]
    fn t_ar_640() {
        // Using the values from the datasheet's worked example as the inputs.
        let t_a = 39.184;
        let t_r = 31.0;
        let emissivity = 1.0;
        let t_ar = super::t_ar(t_a, t_r, emissivity);
        assert_approx_eq!(f32, t_ar, 9516495632.56);
    }

    #[test]
    fn pixel_temperature_640() {
        let clb = mlx90640_calibration();
        let basic_range =
            <mlx90640::Mlx90640Calibration as CalibrationData>::Camera::BASIC_TEMPERATURE_RANGE;
        let k_s_to = clb.k_s_to()[basic_range];
        // The worked example is using TGC, which is done before per_pixel_temperature() is called,
        // so these values are hard-coded from the datasheet.
        let alpha = 1.1876487360496E-7;
        // Pile of values from previous steps.
        let v_ir = 679.250909123826;
        let t_ar = 9516495632.56;
        let t_o = super::per_pixel_temperature(v_ir, alpha, t_ar, k_s_to);
        assert_eq!(t_o, 80.36331);
    }

    #[test]
    fn pixel_temperature_641() {
        let clb = mlx90641_calibration();
        let basic_range =
            <mlx90641::Mlx90641Calibration as CalibrationData>::Camera::BASIC_TEMPERATURE_RANGE;
        let k_s_to = clb.k_s_to()[basic_range];
        let alpha = 3.32641806639731E-7;
        // Pile of values from previous steps.
        let v_ir = 1785f32;
        let t_ar = 9899175739.92;
        let t_o = super::per_pixel_temperature(v_ir, alpha, t_ar, k_s_to);
        // Extended precision from earlier in the datasheet calculations
        assert_approx_eq!(f32, t_o, 80.129812);
    }
}
