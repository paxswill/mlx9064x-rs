// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

//! Calculations for turning sensor output into temperatures
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

use core::convert::TryInto;

use embedded_hal::blocking::i2c;

use crate::common::{Address, CalibrationData, MelexisCamera};
use crate::register::Subpage;
use crate::util::Num;

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
pub fn delta_v<'a, Clb, F>(calibration: &'a Clb, v_dd_pixel: i16) -> F
where
    Clb: CalibrationData<'a, F>,
    F: 'a + Num,
{
    let numer = F::coerce(v_dd_pixel - calibration.v_dd_25());
    let denom = F::coerce(calibration.k_v_dd());
    numer / denom
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
pub fn v_dd<'a, Clb, F>(calibration: &'a Clb, resolution_correction: F, delta_v: F) -> F
where
    Clb: CalibrationData<'a, F>,
    F: 'a + Num,
{
    // According to the documentation for num_traits::NumCast, loss of precision (like from an f64
    // to f32) is allowable. In this case, we're usually going to be going from f32 -> f32 or
    // f32 -> f64 anyways, so it's safe to unwrap.
    let v_dd_0 = calibration.v_dd_0();
    delta_v * resolution_correction + v_dd_0
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
pub fn v_ptat_art<'a, Clb, F>(calibration: &'a Clb, t_a_ptat: F, t_a_v_be: F) -> F
where
    Clb: CalibrationData<'a, F>,
    F: 'a + Num,
{
    let denom = t_a_ptat * calibration.alpha_ptat() + t_a_v_be;
    t_a_ptat / denom * F::TWO_RAISED_EIGHTEEN
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
pub fn ambient_temperature<'a, Clb, F>(calibration: &'a Clb, v_ptat_art: F, delta_v: F) -> F
where
    Clb: CalibrationData<'a, F>,
    F: 'a + Num,
{
    let v_ptat_25 = calibration.v_ptat_25();
    let k_v_ptat = calibration.k_v_ptat();
    let numerator = (v_ptat_art / (F::ONE + k_v_ptat * delta_v)) - v_ptat_25;
    let k_t_ptat = calibration.k_t_ptat();
    numerator / k_t_ptat + F::TWENTY_FIVE
}

/// The non-pixel values read from the camera's RAM for each frame.
///
/// This structure is the non-EEPROM, non-register input when [creating
/// `CommonIrData`][CommonIrData::new].
#[derive(Clone, Copy, Debug)]
pub struct RamData {
    /// $T_{a_{V_{BE}}}$
    ///
    /// This value is labelled as $T_{a_{V_{BE}}}$ on the EEPROM map, but is labelled $V_{BE}$
    /// elsewhere in the datasheet.
    pub t_a_v_be: i16,

    /// $T_{a_{PTAT}}$
    ///
    /// This value is labelled as $T_{a_{PTAT}}$ on the EEPROM map, but is labelled $V_{PTAT}
    /// elsewhere in the datasheet.
    pub t_a_ptat: i16,

    /// $V_{DD_{pix}}$
    pub v_dd_pixel: i16,

    /// The gain value for the current frame.
    pub gain: i16,

    /// The compensation pixel for the current frame.
    pub compensation_pixel: i16,
}

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
    pub fn from_i2c<I2C, Cam, F>(
        bus: &mut I2C,
        i2c_address: u8,
        subpage: Subpage,
    ) -> Result<Self, I2C::Error>
    where
        I2C: i2c::WriteRead,
        Cam: MelexisCamera<F>,
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
#[derive(Debug, PartialEq)]
pub struct CommonIrData<F> {
    /// The gain parameter ($K_{gain}$)
    ///
    /// The raw data from each pixel is first multipled by the "gain parameter" before further
    /// processing. The "gain parameter" is calculated by dividing the value read from RAM by the
    /// "gain coefficient" that is calculated from the calibration EEPROM. The parameter is
    /// documented in section 12.2.2.4 in both datasheets, while the coefficient is documented in
    /// section 11.1.7 in both datasheets.
    pub gain: F,

    /// The pixel supply voltage ($V_{dd}$)
    ///
    /// This is calculated using the [`delta_v`] and [`v_dd`] functions.
    pub v_dd: F,

    /// The [emissivity] ($\varepsilon$) to use when calculating the pixel temperatures.
    ///
    /// The MLX90641 can store an emissivity value in EEPROM, but the MLX90640 does not and a value
    /// must be provided. The high-level API defaults to the value in EEPROM, or 1.0 if that is not
    /// available.
    ///
    /// [emissivity]: https://en.wikipedia.org/wiki/Emissivity
    pub emissivity: F,

    /// The ambient temperature ($T_{a}$)
    ///
    /// The name of this value is slightly inaccurate as it's not so much the ambient temperature
    /// of the air surrounding the camera, but the temperature of the sensor itself, which is
    /// typically a few degrees warmer than the surrounding air. This value is calculated using
    /// [`ambient_temperature`].
    pub t_a: F,
}

impl<F> CommonIrData<F>
where
    F: Num,
{
    /// Create a new `CommonIrData`.
    ///
    /// The three base sources of data for the fields in this structure are the calibration EEPROM
    /// (`calibration`, `emissivity` in some cases, and half of `resolution_correction`), the
    /// device's control register (the other half of `resolution_correction`), and the current
    /// frame's RAM.
    pub fn new<'a, Clb>(
        resolution_correction: F,
        emissivity: F,
        calibration: &'a Clb,
        ram: &RamData,
    ) -> Self
    where
        Clb: CalibrationData<'a, F>,
        F: 'a,
    {
        let delta_v = delta_v(calibration, ram.v_dd_pixel);
        let v_dd = v_dd(calibration, resolution_correction, delta_v);
        // Labelled V_PTAT in the formulas, but T_a_PTAT in the memory map.
        let v_ptat_art = v_ptat_art(
            calibration,
            F::coerce(ram.t_a_ptat),
            F::coerce(ram.t_a_v_be),
        );
        let t_a = ambient_temperature(calibration, v_ptat_art, delta_v);
        let current_gain = F::coerce(ram.gain);
        let gain = calibration.gain() / current_gain;
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
/// section 11.2.2.5 in the both datasheets. If just an "image" is required but not the actual
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
/// ([`k_ta_pixels`]) values, while $K_{gain}$, $V_{dd}$, $T_{a}$, and emissivity ($\varepsilon$)
/// are provided by [`CommonIrData`].
///
/// [offset]: CalibrationData::offset_reference_pixels
/// [`k_v_pixels`]: CalibrationData::k_v_pixels
/// [`k_ta_pixels`]: CalibrationData::k_ta_pixels
///
/// This function is also used for calculating the compensation pixel values when using thermal
/// gradient compensation.
#[inline]
pub fn per_pixel_v_ir<F>(
    pixel_data: F,
    common: &CommonIrData<F>,
    reference_offset: F,
    k_v: F,
    k_ta: F,
) -> F
where
    F: Num,
{
    let pixel_gain = pixel_data * common.gain;
    let pixel_offset = pixel_gain
        - reference_offset
            * (F::ONE + k_ta * (common.t_a - F::TWENTY_FIVE))
            * (F::ONE + k_v * (common.v_dd - F::THREE_POINT_THREE));
    pixel_offset / common.emissivity
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
pub fn raw_pixels_to_ir_data<'a, Clb, Px, F>(
    calibration: &'a Clb,
    emissivity: F,
    resolution_correction: F,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [F],
) -> F
where
    Clb: CalibrationData<'a, F>,
    Px: Iterator<Item = bool>,
    F: 'a + Num,
{
    // Knock out the values common to all pixels first.
    let common = CommonIrData::new(resolution_correction, emissivity, calibration, &ram);
    // Compensation pixels are only used if temperature gradient compensation is being used.
    let compensation_pixel_offset = calibration.temperature_gradient_coefficient().map(|tgc| {
        // TODO: There's a note in the datasheet advising a moving average filter (length >=
        // 16) on the compensation pixel gain.
        let compensation_pixel = F::coerce(ram.compensation_pixel);
        let compensation_pixel_offset = per_pixel_v_ir(
            compensation_pixel,
            &common,
            F::coerce(calibration.offset_reference_cp(subpage)),
            calibration.k_v_cp(subpage),
            calibration.k_ta_cp(subpage),
        );
        // Premultiplying by the TGC here
        tgc * compensation_pixel_offset
    });
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    let offset_reference_pixels = calibration
        .offset_reference_pixels(subpage)
        .copied()
        .map(F::coerce);
    destination
        .iter_mut()
        // Chunk into two byte segments, for each 16-bit value
        .zip(pixel_data.chunks_exact(2))
        // Zip up the corresponding values from the calibration data.
        // Skipping alpha (sensitivity) as that's more related to temperature calculations
        .zip(offset_reference_pixels)
        .zip(calibration.k_v_pixels(subpage))
        .zip(calibration.k_ta_pixels(subpage))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        // feeling a little lispy in here with all these parentheses
        .for_each(|((((output, pixel_slice), reference_offset), k_v), k_ta)| {
            // Safe to unwrap as this is from chunks_exact(2)
            let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
            let pixel_data = F::coerce(i16::from_be_bytes(pixel_bytes));
            let mut pixel_offset =
                per_pixel_v_ir(pixel_data, &common, reference_offset, *k_v, *k_ta);
            // I hope the branch predictor/compiler is smart enough to realize there's
            // almost always going to be only one hot branch here
            if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                pixel_offset -= compensation_pixel_offset;
            }
            *output = pixel_offset;
        });
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
#[inline]
pub fn t_ar<F>(t_a: F, t_r: F, emissivity: F) -> F
where
    F: Num,
{
    // Again, start with the steps common to all pixels
    let t_a_k4 = (t_a + F::KELVINS_TO_CELSIUS).powi(4);
    // If the emissivity of an object is 1, it also absorbs all infrared radiation (see Kirchoff's
    // law of thermal radiation). In that case, there is no reflected radiation, so we can ignore
    // t_r.
    if emissivity == F::ONE {
        t_a_k4
    } else {
        let t_r_k4 = (t_r + F::KELVINS_TO_CELSIUS).powi(4);
        t_r_k4 - ((t_r_k4 - t_a_k4) / emissivity)
    }
}

/// Calculate the sensitivity correction coefficient
///
/// The sensitivity correction coefficient isn't explicitly named in the datasheet, but it's a
/// common factor used when calculating $T\_o$. In section 11.2.2.8 of the datasheet,
/// $1 + K\_{S\_{T\_a}} \* (T\_a - T\_{a\_0})$ is common to all pixel sensitivity calculation, so
/// calculating it once is done to improve performance.
pub fn sensitivity_correction_coefficient<'a, Clb, F>(calibration: &'a Clb, t_a: F) -> F
where
    Clb: CalibrationData<'a, F>,
    F: 'a + Num,
{
    let k_s_ta = calibration.k_s_ta();
    let t_a0 = F::TWENTY_FIVE;
    // Little bit of optimization; this factor is shared by all pixels
    F::ONE + k_s_ta * (t_a - t_a0)
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
#[inline]
pub fn per_pixel_temperature<F>(v_ir: F, alpha: F, t_ar: F, k_s_to: F) -> F
where
    F: Num,
{
    // This function is a mess of raising floats to the third and fourth powers, doing some
    // operations, then taking the fourth root of everything.
    let s_x = k_s_to * (alpha.powi(3) * v_ir + alpha.powi(4) * t_ar).fourth_root();
    let t_o_root =
        (v_ir / (alpha * (F::ONE - k_s_to * F::KELVINS_TO_CELSIUS) + s_x) + t_ar).fourth_root();
    t_o_root - F::KELVINS_TO_CELSIUS
}

/// Calculate the temperature for all pixels, starting from the raw IR data
///
/// This function applies [`per_pixel_temperature`] to all pixels in the given subpage. It modifies
/// the `destination` array in-place, replacing the $V\_{IR}$ data with the temperatures. The
/// ambient temperature (`t_a`, $T\_a$) needs to be given from the same frame that produced the IR
/// data.
pub fn raw_ir_to_temperatures<'a, Clb, Px, F>(
    calibration: &'a Clb,
    emissivity: F,
    t_a: F,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [F],
) where
    Clb: CalibrationData<'a, F>,
    Px: Iterator<Item = bool>,
    F: 'a + Num,
{
    // TODO: this step could probably be optimized a little bit (if needed) by pushing this
    // calculation to the calibration loading step.
    let alpha_compensation_pixel = calibration
        .temperature_gradient_coefficient()
        .map(|tgc| calibration.alpha_cp(subpage) * tgc);
    let k_s_to_basic = calibration.k_s_to()[Clb::Camera::BASIC_TEMPERATURE_RANGE];
    let alpha_coefficient = sensitivity_correction_coefficient(calibration, t_a);
    // TODO: design a way to provide T-r, the reflected temperature. Basically, the temperature
    // of the surrounding environment (but not T_a, which is basically the temperature of the
    // sensor itself). For now hard-coding this to 8 degrees lower than T_a.
    let t_r = t_a - F::coerce(8u8);
    let t_ar = t_ar(t_a, t_r, emissivity);

    destination
        .iter_mut()
        .zip(calibration.alpha_pixels(subpage))
        // filter out the pixels that aren't part of this subpage
        .filter(|_| valid_pixels.next().unwrap_or_default())
        .for_each(|(output, alpha)| {
            let v_ir = *output;
            let compensated_alpha = match alpha_compensation_pixel {
                Some(alpha_compensation_pixel) => *alpha - alpha_compensation_pixel,
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
pub fn raw_pixels_to_temperatures<'a, Clb, Px, F>(
    calibration: &'a Clb,
    emissivity: F,
    resolution_correction: F,
    pixel_data: &[u8],
    ram: RamData,
    subpage: Subpage,
    valid_pixels: &mut Px,
    destination: &mut [F],
) -> F
where
    Clb: CalibrationData<'a, F>,
    Px: Iterator<Item = bool>,
    F: 'a + Num,
{
    // Knock out the values common to all pixels first.
    let common = CommonIrData::new(resolution_correction, emissivity, calibration, &ram);
    // Compensation pixels are only used if temperature gradient compensation is being used.
    let compensation_pixel_offset = calibration.temperature_gradient_coefficient().map(|tgc| {
        // TODO: There's a note in the datasheet advising a moving average filter (length >=
        // 16) on the compensation pixel gain.
        let compensation_pixel = F::coerce(ram.compensation_pixel);
        let compensation_pixel_offset = per_pixel_v_ir(
            compensation_pixel,
            &common,
            F::coerce(calibration.offset_reference_cp(subpage)),
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
    let k_s_to_basic = calibration.k_s_to()[Clb::Camera::BASIC_TEMPERATURE_RANGE];
    let alpha_coefficient = sensitivity_correction_coefficient(calibration, common.t_a);
    // TODO: design a way to provide T-r, the reflected temperature. Basically, the temperature
    // of the surrounding environment (but not T_a, which is basically the temperature of the
    // sensor itself). For now hard-coding this to 8 degrees lower than T_a.
    let t_r = common.t_a - F::coerce(8u8);
    let t_ar = t_ar(common.t_a, t_r, emissivity);
    // At this point, we're now going to start calculating and copying over the pixel data. It
    // will *not* be actual temperatures, but it can be used for some imaging purposes.
    let offset_reference_pixels = calibration
        .offset_reference_pixels(subpage)
        .copied()
        .map(F::coerce);
    destination
        .iter_mut()
        // Chunk into two byte segments, for each 16-bit value
        .zip(pixel_data.chunks_exact(2))
        .zip(offset_reference_pixels)
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
                let pixel_data = F::coerce(i16::from_be_bytes(pixel_bytes));
                let mut v_ir = per_pixel_v_ir(pixel_data, &common, reference_offset, *k_v, *k_ta);
                // I hope the branch predictor/compiler is smart enough to realize there's
                // almost always going to be only one hot branch here
                if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                    v_ir -= compensation_pixel_offset;
                }
                let compensated_alpha = match alpha_compensation_pixel {
                    Some(alpha_compensation_pixel) => *alpha - alpha_compensation_pixel,
                    None => *alpha,
                } * alpha_coefficient;
                *output = per_pixel_temperature(v_ir, compensated_alpha, t_ar, k_s_to_basic);
            },
        );
    common.t_a
}

#[cfg(test)]
#[generic_tests::define]
mod test {
    use core::fmt::Debug;

    use float_cmp::{assert_approx_eq, ApproxEq};

    use crate::test::{mlx90640_datasheet_eeprom, mlx90641_datasheet_eeprom};
    use crate::util::Num;
    use crate::{mlx90640, mlx90641, CalibrationData, MelexisCamera, Subpage};

    fn mlx90640_calibration<F>() -> mlx90640::Mlx90640Calibration<F>
    where
        F: Debug + ApproxEq + Num,
    {
        let eeprom_data = mlx90640_datasheet_eeprom();
        mlx90640::Mlx90640Calibration::from_data(&eeprom_data)
            .expect("Mlx90640Calibration should be able to be created from the example data")
    }

    fn mlx90641_calibration<F>() -> mlx90641::Mlx90641Calibration<F>
    where
        F: Debug + ApproxEq + Num,
    {
        let eeprom_data = mlx90641_datasheet_eeprom();
        mlx90641::Mlx90641Calibration::from_data(&eeprom_data)
            .expect("Mlx90641Calibration should be able to be created from the example data")
    }

    // The super bare, single function tests with magic number are using values from the worked
    // example in the MLX90640 or MLX90641 datasheets.

    #[test]
    fn delta_v<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb_640 = mlx90640_calibration::<F>();
        let clb_641 = mlx90641_calibration::<F>();
        // The value from the 640 datasheet terminally repeats, so expressing it as a fraction here
        // (the actual fraction is \frac{-59}{-3168}, but cannot be simplified further as 59 is
        // prime).
        let expected_640 = F::coerce(59u16) / F::coerce(3168u16);
        // The 641 value = 138 / (-3136)
        let expected_641 = F::coerce(-0.04400510204081632653061224489796);
        // Input argument is v_dd_pixel
        assert_approx_eq!(F, super::delta_v(&clb_640, -13115), expected_640);
        assert_approx_eq!(F, super::delta_v(&clb_641, -13430), expected_641);
    }

    #[test]
    fn v_dd<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb_640 = mlx90640_calibration::<F>();
        let clb_641 = mlx90641_calibration::<F>();
        let resolution_correction = F::ONE;
        // MLX90640 datasheet has ≈3.319. Recalculated below for greater precision.
        // Because this uses delta_v, the same issue presents here (terminally repeating).
        let expected_640 = F::coerce(59u16) / F::coerce(3168u16) + F::THREE_POINT_THREE;
        assert_approx_eq!(
            F,
            super::v_dd(&clb_640, resolution_correction, F::coerce(0.018623737)),
            expected_640
        );
        // MLX90641 datasheet has ≈ 3.25599. Recalculated value below for more precision.
        let expected_641 = F::coerce(3.255994897959183673469387755102);
        assert_approx_eq!(
            F,
            super::v_dd(&clb_641, resolution_correction, F::coerce(-0.0440051)),
            expected_641
        );
    }

    #[test]
    fn v_ptat_art<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb_640 = mlx90640_calibration::<F>();
        let clb_641 = mlx90641_calibration::<F>();
        // Inputs are t_a_ptat (aka v_ptat) and t_a_v_be
        assert_approx_eq!(
            F,
            super::v_ptat_art(&clb_640, F::coerce(1711.0), F::coerce(19442.0)),
            F::coerce(12873.57952)
        );
        // difference from datasheet: precision added (2)
        assert_approx_eq!(
            F,
            super::v_ptat_art(&clb_641, F::coerce(1752.0), F::coerce(19540.0)),
            F::coerce(13007.712)
        );
    }

    #[test]
    fn t_a<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb_640 = mlx90640_calibration::<F>();
        let clb_641 = mlx90641_calibration::<F>();
        let delta_v_640 = super::delta_v(&clb_640, -13115);
        let delta_v_641 = super::delta_v(&clb_641, -13430);
        let v_ptat_art_640 = super::v_ptat_art(&clb_640, F::coerce(1711u16), F::coerce(19442u16));
        let v_ptat_art_641 = super::v_ptat_art(&clb_641, F::coerce(1752u16), F::coerce(19540u16));
        // The datasheet is a bit more precise than I can get with f32, so approx_eq here
        assert_approx_eq!(
            F,
            super::ambient_temperature(&clb_640, v_ptat_art_640, delta_v_640),
            F::coerce(39.18440152)
        );
        assert_approx_eq!(
            F,
            super::ambient_temperature(&clb_641, v_ptat_art_641, delta_v_641),
            F::coerce(42.022)
        );
    }

    #[test]
    // also called "pixel offset" in a few places
    fn v_ir_640<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb = mlx90640_calibration::<F>();
        // The worked example uses pixel(12, 16) (and we use 0-indexing, so subtract one) and
        // subpage 1
        let pixel_index = 11 * mlx90640::WIDTH + 15;
        let offset = clb
            .offset_reference_pixels(Subpage::One)
            .nth(pixel_index)
            .copied()
            .map(F::coerce)
            .unwrap();
        let k_v = clb.k_v_pixels(Subpage::One).nth(pixel_index).unwrap();
        let k_ta = clb.k_ta_pixels(Subpage::One).nth(pixel_index).unwrap();
        // Taken from the worked example
        let common = super::CommonIrData {
            gain: F::coerce(1.01753546947234),
            v_dd: F::coerce(3.319),
            emissivity: F::ONE,
            t_a: F::coerce(39.18440152),
        };
        let raw_pixel = F::coerce(609u16);
        let v_ir = super::per_pixel_v_ir(raw_pixel, &common, offset, *k_v, *k_ta);
        // I'm getting 700.89, which is close enough considering how many places to lose precision
        // there are in this step.
        assert_approx_eq!(F, v_ir, F::coerce(700.882495690866));
    }

    #[test]
    // also called "pixel offset" in a few places
    fn v_ir_641<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb = mlx90641_calibration::<F>();
        // The worked example uses pixel(6, 9) (and we use 0-indexing, so subtract one) and
        // subpage 0
        let pixel_index = 5 * mlx90641::WIDTH + 8;
        let offset = clb
            .offset_reference_pixels(Subpage::Zero)
            .nth(pixel_index)
            .copied()
            .map(F::coerce)
            .unwrap();
        let k_v = clb.k_v_pixels(Subpage::Zero).nth(pixel_index).unwrap();
        let k_ta = clb.k_ta_pixels(Subpage::Zero).nth(pixel_index).unwrap();
        // Taken from the worked example
        let common = super::CommonIrData {
            gain: F::coerce(1.02445038),
            v_dd: F::coerce(3.25599),
            emissivity: F::coerce(0.949218),
            t_a: F::coerce(42.022),
        };
        let raw_pixel = F::coerce(972u16);
        let v_ir = super::per_pixel_v_ir(raw_pixel, &common, offset, *k_v, *k_ta);
        // I'm getting 700.89, which is close enough considering how many places to lose precision
        // there are in this step.
        assert_approx_eq!(F, v_ir, F::coerce(1785.0));
    }

    #[test]
    fn pixel_temperature_640<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb = mlx90640_calibration::<F>();
        let basic_range =
            <<mlx90640::Mlx90640Calibration<F> as CalibrationData<F>>::Camera as MelexisCamera<
                F,
            >>::BASIC_TEMPERATURE_RANGE;
        let k_s_to = clb.k_s_to()[basic_range];
        // The worked example is using TGC, which is done before per_pixel_temperature() is called,
        // so these values are hard-coded from the datasheet.
        let alpha = F::coerce(1.1876487360496E-7);
        // Pile of values from previous steps.
        let v_ir = F::coerce(679.250909123826);
        let t_ar = F::coerce(9516495632.56);
        let t_o = super::per_pixel_temperature(v_ir, alpha, t_ar, k_s_to);
        assert_approx_eq!(F, t_o, F::coerce(80.36331));
    }

    #[test]
    fn pixel_temperature_641<F>()
    where
        F: Debug + ApproxEq + Num,
    {
        let clb = mlx90641_calibration::<F>();
        let basic_range =
            <<mlx90641::Mlx90641Calibration<F> as CalibrationData<F>>::Camera as MelexisCamera<
                F,
            >>::BASIC_TEMPERATURE_RANGE;
        let k_s_to = clb.k_s_to()[basic_range];
        let alpha = F::coerce(3.32641806639731E-7);
        // Pile of values from previous steps.
        let v_ir = F::coerce(1785.0);
        let t_ar = F::coerce(9899175739.92);
        let t_o = super::per_pixel_temperature(v_ir, alpha, t_ar, k_s_to);
        // Extended precision from earlier in the datasheet calculations
        assert_approx_eq!(F, t_o, F::coerce(80.129812));
    }

    #[instantiate_tests(<f32>)]
    mod f32 {}

    #[instantiate_tests(<f64>)]
    mod f64 {}
}
