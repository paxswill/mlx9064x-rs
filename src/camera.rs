//! Common functionality between MLX90640 and MLX90641 cameras.
//!
//! At first glance, the datasheets for these modules can be pretty intimidating, with nearly half
//! the document taken up by pages of formulas. After reading through them though, it becomes
//! pretty clear that the formulas are primarily performing manual conversion to signed integers
//! and other bit twiddling that can be written much more clearly. Here's a bit of a decoder ring
//! for some of the patterns:
//!
//! * A line similar to "If K<sub>Foo</sub> &gt; *(2<sup>n</sup> - 1)* → K<sub>Foo</sub> = K<sub>Foo</sub>
//!   - *(2<sup>n + 1</sup>" is converting an unsigned integer to a signed one. The *italicized*
//!   portions in the quote are typically expanded, so instead of *2<sup>7</sup> - 1* you'll see
//!   *127*, and so on.
//! * Masking a value off with a logical AND, followed by a division by a power of 2. The division
//!   can be easier to read as a right-shift by whatever power of two.
//! * The datasheets are inconsistent between each other and within themselves as to use "α" or
//!   *Alpha*, or whether to write something as "Pix<sub>Foo</sub>(i, j)" or "Foo<sub>pix</sub>(i,
//!   j)".
//!
//! And a glossary:
//! * α, alpha: Sensitivity coefficient
//! * CP: Compensation pixel
//! * ε, emissivity: This is a bit beyond a glossary entry, but basically how much IR radiation a
//!   surface emits relative to it's temperature.
//! * K: Prefix for constants.
//! * PTAT: Proportional to ambient temperature
//! * T<sub>a</sub>: Ambient temperature
//! * T<sub>a<sub>0</sub></sub>: Ambient temperature reference, 25.0 ℃
//! * T<sub>o</sub>: Object temperature (meaning the temperature an individual pixel has detected
//!   for an object.
//! * V<sub>DD</sub>: Pixel supply voltage
//! * V<sub>DD<sub>25</sub></sub>: Pixel supply voltage reference at 25.0 ℃

use arrayvec::ArrayVec;

/// The MLX9064\* modules store a large amount of calibration data in a built-in EEPROM. This trait
/// defines methods exposing the values needed for generating thermal images from the camera.
/// Without context, the method names are almost useless, but they make much more sense when read
/// in context of the MLX90640 and MLX90641 datasheets, and using more descritptive names would
/// make comparing the implementation to the datasheet more difficult.
///
/// A couple of conventions:
///
/// * Method names will use underscores in changes of sub-script-ness.
/// * Some methods will take suppage arguments when a device doesn't vary a value depending on the
///   subpage selected. In that case just ignore that argument.
/// * Slices covering all pixels are laid out in row-major order, with the X-axis increasing from
///   left to right, and the Y-axis increasing from top to bottom.
pub trait MelexisEeprom {
    /// K<sub>V<sub>DD</sub></sub>
    fn k_v_dd(&self) -> i16;

    /// V<sub>DD<sub>25</sub></sub>
    ///
    /// Pixel supply reference voltage at 25.0 ℃
    fn v_dd_25(&self) -> i16;

    /// Resolution<sub>EE</sub>
    fn resolution(&self) -> u8;

    /// V<sub>DD<sub>0</sub></sub>.
    ///
    /// The voltage supplied to the device. It *should* be as close to 3.3 as possible.
    fn v_dd_0(&self) -> f32 {
        3.3f32
    }

    /// K<sub>V<sub>PTAT</sub></sub>
    fn k_v_ptat(&self) -> i16;

    /// K<sub>T<sub>PTAT</sub></sub>
    fn k_t_ptat(&self) -> i16;

    /// V<sub>PTAT<sub>25</sub></sub>
    fn v_ptat_25(&self) -> i16;

    /// α<sub>PTAT</sub>
    fn alpha_ptat(&self) -> u16;

    /// The gain constant. Usually written as <var>GAIN</var> in the datasheets.
    fn gain(&self) -> i16;

    /// K<sub>S<sub>T<sub>a</sub></sub></sub>
    fn k_s_ta(&self) -> f32;

    /// A slice of the "corner temperatures". These define temperature ranges with different
    /// sensitivity characteristics. They are indexed in the datasheet starting from 1, so be aware
    /// fo the difference in indexing.
    fn corner_temperatures(&self) -> &[i16];

    /// K<sub>s<sub>T<sub>o</sub>(n)</sub></sub>, where <var>n</var> as a corresponding corner
    /// temperature index. The length of this index and the one from [corner_temperatures] must be
    /// identical.
    fn k_s_to(&self) -> &[f32];

    /// The α<sub>correction</sub>(r), where <var>r</var> is the *temperature range* index. This
    /// range is the region between two successive corner temperatures.
    fn alpha_correction(&self) -> &[f32];

    /// The index of the "native" temperature range. Temperature ranges (delimited by the control
    /// temperatures) outside of the "native" range are "extended temperature ranges" and require
    /// extra processing for accuracy. The datasheets do not use the term "native range, but for
    /// this library it is defined as the index of the range (0-indexed) that has
    /// α<sub>correction</sub>(r) = 1. Again note that the datasheets use 1-indexing.
    /// Also note that this is returning [usize], to make it easier to use when indexing into slices.
    fn native_range(&self) -> usize;

    /// The emissivity stored on the device. Not all devices support storing the emissivity, in
    /// which case they should return [None] (which is what the provided implementation does).
    fn emissivity(&self) -> Option<f32> {
        None
    }

    /// Offset<sub>reference<sub>pixel</sub></sub>, for all pixels.
    ///
    /// The returned slice covers all pixels.
    fn offset_reference_pixels(&self, subpage: Subpage) -> &[i16];

    // Offset<sub>reference<sub>CP</sub></sub>
    fn offset_reference_cp(&self, subpage: Subpage) -> i16;

    /// α<sub>pixel</sub>
    ///
    /// The returned slice covers all pixels.
    fn alpha_pixels(&self, subpage: Subpage) -> &[f32];

    /// α<sub>CP</sub>
    fn alpha_cp(&self, subpage: Subpage) -> f32;

    /// K<sub>V<sub>pixel</sub></sub>
    ///
    /// The returned slice covers all pixels.
    fn k_v_pixels(&self, subpage: Subpage) -> &[f32];

    /// K<sub>V<sub>CP</sub></sub>
    fn k_v_cp(&self, subpage: Subpage) -> f32;

    /// K<sub>T<sub>a</sub>pixel</sub>
    ///
    /// The returned slice covers all pixels.
    fn k_ta_pixels(&self, subpage: Subpage) -> &[f32];

    /// K<sub>T<sub>a</sub>CP</sub>
    fn k_ta_cp(&self, subpage: Subpage) -> f32;

    /// Temperature Gradient Coefficient
    ///
    /// Some devices do not support a TGC (it can also be disabled manually on other devices).
    fn temperature_gradient_coefficient(&self) -> Option<f32>;
}

/// Identify which subpage to access.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
#[repr(usize)]
pub enum Subpage {
    Zero = 0,
    One = 1,
}

/// A helper function for calculating the sensitivity correction coefficients
/// (Alpha<sub>corr<sub>range<sub>n</sub></sub></sub>) for the different temperature ranges.
///
/// This function will `panic` if the passed in slices both do not have exactly `N` elements.
pub(crate) fn alpha_correction_coefficients<const NUM_RANGES: usize>(
    native_range: usize,
    corner_temperatures: &[i16],
    k_s_to: &[f32],
) -> [f32; NUM_RANGES] {
    // This is the actual calculation. The values are built up recursively from the base case of
    // the native range (which doesn't need correcting, so it's 1).
    // Memoizing would be nice here, but these calculations are done only once, at start up, so the
    // impact isn't that big.
    let results: ArrayVec<f32, NUM_RANGES> = (0..NUM_RANGES)
        .map(|n| alpha_corr_n(n, native_range, corner_temperatures, k_s_to))
        .collect();
    results
        .into_inner()
        .expect("The Rust-range 0..NUM_RANGES should fill an array of NUM_RANGES elements")
}

/// The actual calculations for [alpha_correction_coefficients] as a recursive function. Memoizing
/// would be nice, but these calculations are only performed once, at start up.
fn alpha_corr_n(n: usize, native_range: usize, ct: &[i16], k_s_to: &[f32]) -> f32 {
    if n < native_range {
        (1f32 + k_s_to[n] * f32::from(ct[n + 1] - ct[n])).recip()
            * alpha_corr_n(n + 1, native_range, ct, k_s_to)
    } else if n > native_range {
        (1f32 + k_s_to[n - 1] * f32::from(ct[n] - ct[n - 1]))
            * alpha_corr_n(n - 1, native_range, ct, k_s_to)
    } else {
        // The base case, when n == native_range
        1f32
    }
}
