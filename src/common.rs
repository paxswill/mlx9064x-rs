// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
//! Common functionality between MLX90640 and MLX90641 cameras.
//!
//! At first glance, the datasheets for these modules can be pretty intimidating, with nearly half
//! the document taken up by pages of formulas. After reading through them though, it becomes
//! pretty clear that the formulas are primarily performing manual conversion to signed integers
//! and other bit twiddling that can be written much more clearly. Here's a bit of a decoder ring
//! for some of the patterns:
//!
//! * If you see a line similar to
//!   >  If K<sub>Foo</sub> &gt; *(2<sup>n</sup> - 1)* → K<sub>Foo</sub> = K<sub>Foo</sub> - *2<sup>n + 1</sup>*
//!
//!   It is converting an unsigned integer to a signed one. The *italicized* portions in the quote
//!   are typically expanded, so instead of *2<sup>7</sup>&nbsp;-&nbsp;1* and *2<sup>8</sup* you'll
//!   see 127 and 256.
//! * Masking a value off with a logical AND, followed by a division by a power of 2. The division
//!   can be easier to read as a right-shift by whatever power of two.
//! * The datasheets are inconsistent between each other and within themselves whether to write
//!   something as "Pix<sub>Foo</sub>(i, j)" or "Foo<sub>pix</sub>(i, j)".
//!
//! And a glossary:
//! # Glossary
//! <dl>
//! <dt>
//! α, alpha
//! </dt><dd>
//! Sensitivity coefficient
//! </dd>
//! <dt>
//! CP
//! </dt><dd>
//! Compensation pixel
//! </dd>
//! <dt>
//! ε, emissivity
//! </dt><dd>
//! This is a bit beyond a glossary entry, but basically how much IR radiation a surface emits
//! relative to its temperature.
//! </dd>
//! <dt>
//! K
//! </dt><dd>
//! Prefix for constants.
//! </dd>
//! <dt>
//! PTAT
//! </dt><dd>
//! Proportional to ambient temperature
//! </dd>
//! <dt>
//! T<sub>a</sub>
//! </dt><dd>
//! Ambient temperature
//! </dd>
//! <dt>
//! T<sub>a<sub>0</sub></sub>
//! </dt><dd>
//! Ambient temperature reference, 25.0 ℃. If it looks like 0, it's probably "o" as this value is
//! really only used in one place.
//! </dd>
//! <dt>
//! T<sub>o</sub>
//! </dt><dd>
//! Object temperature, meaning the temperature an individual pixel has detected for an object.
//! </dd>
//! <dt>
//! V<sub>DD</sub>
//! </dt><dd>
//! Pixel supply voltage
//! </dd>
//! <dt>
//! V<sub>DD<sub>25</sub></sub>
//! </dt><dd>
//! Pixel supply voltage reference at 25.0 ℃
//! </dd>
//! </dl>
use core::fmt;

use arrayvec::ArrayVec;

use crate::register::{AccessPattern, Subpage};

pub trait FromI2C<I2C> {
    type Error;
    type Ok;

    /// Create an instance of a type using data retrieved over I²C.
    fn from_i2c(bus: &mut I2C, i2c_address: u8) -> Result<Self::Ok, Self::Error>;
}

/// This trait provides access to the module-specific calibration data.
///
/// Each MLX9064\* camera has calibration data from the factory stored on its EEPROM. The
/// factory-provided data is then used as the input to generate the constants needed to convert the
/// raw output of the camera into concrete temperatures (or even just raw infrared radition
/// measurements). The naming scheme for the methods in this trait is taken from the names of the
/// variables used in the formulas in the datasheet. Most users of this library can make use fo the
/// provided implementations, but if you're trying to minimize memory usage or tweak performance
/// for a specific use case, this might be a way to do it.
pub trait CalibrationData<'a> {
    /// Pixel supply voltage constant (K<sub>V<sub>DD</sub></sub>).
    fn k_v_dd(&self) -> i16;

    /// Constant for pixel supply voltage at 25℃ (V<sub>DD<sub>25</sub></sub>).
    fn v_dd_25(&self) -> i16;

    /// ADC resolution this camera was calibrated at.
    // TODO: Should this return `Resolution`?
    fn resolution(&self) -> u8;

    /// Pixel supply voltage (V<sub>DD<sub>0</sub></sub>).
    ///
    /// This is the voltage supplied to the device, and should be 3.3V for the MLX90640 and
    /// MLX90641. The default implementation is hardcoded to return `3.3f32`, but if there's a
    /// reason it needs to be overridden, it's possible.
    fn v_dd_0(&self) -> f32 {
        3.3f32
    }

    /// Voltage proportional to ambient temperature constant (K<sub>V<sub>PTAT</sub></sub>).
    fn k_v_ptat(&self) -> f32;

    /// Temperature proportional to ambient temperature constant (K<sub>T<sub>PTAT</sub></sub>).
    fn k_t_ptat(&self) -> f32;

    /// Voltage proportional to ambient temperature at 25℃ (V<sub>PTAT<sub>25</sub></sub>).
    fn v_ptat_25(&self) -> f32;

    /// Sensitivity proportional to ambient temperature (α<sub>PTAT</sub>).
    fn alpha_ptat(&self) -> f32;

    /// The gain constant. Usually written as <var>GAIN</var> in the datasheets.
    fn gain(&self) -> f32;

    /// Sensitivity constant for ambient temperature (K<sub>S<sub>T<sub>a</sub></sub></sub>).
    fn k_s_ta(&self) -> f32;

    /// A slice of the "corner temperatures".
    ///
    /// These define temperature ranges with different sensitivity characteristics. They are
    /// indexed in the datasheet starting from 1 but everything in this library is 0-indexed, so be
    /// aware of the difference.
    fn corner_temperatures(&self) -> &[i16];

    /// Constant for the object temperature sensitivity (K<sub>s<sub>T<sub>o</sub>(n)</sub></sub>)
    /// depending on the temperature range.
    ///
    /// This is a slight variance from the datasheet's nomenclature. In the symbol above,
    /// <var>n</sub> is the index of the temperature range, which the datasheet normally just
    /// writes out (ex: K<sub>S<sub>T<sub>o</sub>1</sub></sub> through how every many temperature
    /// ranges the camera has).
    ///
    /// This method returns a slice of values equal in length to
    /// [`corner_temperatures`](CalibrationData::corner_temperatures).
    fn k_s_to(&self) -> &[f32];

    /// Temperature range sensitivity correction (α<sub>correction</sub>(n))
    ///
    /// Like [`k_s_to`], the name of this method is slightly different that the naming in the
    /// datasheet. Also like `k_s_to`, this method returns a slice of values with a length equal to
    /// the length of the slice returned by
    /// [`corner_temperatures`](CalibrationData::corner_temperatures),
    ///
    /// [`k_s_to`]: CalibrationData::k_s_to
    fn alpha_correction(&self) -> &[f32];

    /// The index of the basic temperature range.
    ///
    /// Temperature ranges (delimited by the control temperatures) outside of the basic range
    /// are "extended temperature ranges" and require extra processing for accuracy. The datasheets
    /// don't give a generic definition of the basic range, but for this library it is defined as
    /// the temperature range with α<sub>correction</sub>(r) = 1. Also note that this library uses
    /// 0-indexing as opposed to the datasheets that use 1-indexing.
    fn basic_range(&self) -> usize;

    /// The emissivity stored on the device.
    ///
    /// Not all devices support storing the emissivity, in which case they should return [None]
    /// (which is what the provided implementation does).
    fn emissivity(&self) -> Option<f32> {
        None
    }

    type OffsetReferenceIterator: Iterator<Item = &'a i16>;

    /// An iterator over the per-pixel offset reference values for the given subpage
    /// (Offset<sub>reference</sub>(i, j)).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn offset_reference_pixels(&'a self, subpage: Subpage) -> Self::OffsetReferenceIterator;

    /// The offset reference value for the compensation pixel corresponding to the given subpage
    /// (Offset<sub>reference<sub>CP</sub></sub>).
    fn offset_reference_cp(&self, subpage: Subpage) -> i16;

    type AlphaIterator: Iterator<Item = &'a f32>;

    /// An iterator over the per-pixel sensitivity calibration values (α<sub>pixel</sub>(i, j)).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn alpha_pixels(&'a self, subpage: Subpage) -> Self::AlphaIterator;

    /// The sensitivity calibration value for the compensation pixel for the given subpage
    /// (α<sub>CP</sub>).
    fn alpha_cp(&self, subpage: Subpage) -> f32;

    type KvIterator: Iterator<Item = &'a f32>;

    /// An iterator over the per-pixel voltage calibration constants (K<sub>V<sub>pixel</sub></sub>).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn k_v_pixels(&'a self, subpage: Subpage) -> Self::KvIterator;

    /// The voltage calibration constant for the compensation pixel for the given subpage
    /// (K<sub>V<sub>CP</sub></sub>).
    fn k_v_cp(&self, subpage: Subpage) -> f32;

    type KtaIterator: Iterator<Item = &'a f32>;

    /// The per pixel ambient temperature calibration constants (K<sub>T<sub>a</sub>pixel</sub>).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn k_ta_pixels(&'a self, subpage: Subpage) -> Self::KtaIterator;

    /// The ambient temperature calibration constant for the compensation pixel for the given
    /// subpage (K<sub>T<sub>a</sub>CP</sub>).
    fn k_ta_cp(&self, subpage: Subpage) -> f32;

    /// Temperature gradient coefficient (TGC).
    ///
    /// Some devices do not support a TGC (it can also be disabled manually on other devices).
    fn temperature_gradient_coefficient(&self) -> Option<f32>;
}
/// Marker newtype for addresses accessible over I<sup>2</sup>C.
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub struct Address(u16);

impl Address {
    /// Wrap the given address in an `Address`.
    ///
    /// This function is intended to be used in const contexts, in other cases the
    /// [`From`][core::convert::From]/[`TryFrom`][core::convert::TryFrom] implementations are
    /// probably easier to use.
    pub const fn new(address: u16) -> Self {
        Self(address)
    }

    pub(crate) fn as_bytes(&self) -> [u8; 2] {
        self.0.to_be_bytes()
    }
}

impl fmt::Debug for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Address({:#X})", self.0)
    }
}

impl From<u16> for Address {
    fn from(raw_address: u16) -> Self {
        Self::new(raw_address)
    }
}

impl From<Address> for u16 {
    fn from(address: Address) -> Self {
        address.0
    }
}

impl From<Address> for usize {
    fn from(address: Address) -> Self {
        address.0 as usize
    }
}

/// Define common addresses accessible within the camera's RAM.
pub trait MelexisCamera {
    type PixelRangeIterator: IntoIterator<Item = PixelAddressRange>;

    type PixelsInSubpageIterator: IntoIterator<Item = bool>;

    /// Ranges of memory that should be read to load a subpage's data from RAM.
    ///
    /// Different cameras with different [access patterns][crate::AccessPattern] have different optimal
    /// ways of loading data from RAM. In some cases loading by row and then ignoring half the data
    /// may be appropriate, in other loading individual pixels may be best.
    fn pixel_ranges(subpage: Subpage, access_pattern: AccessPattern) -> Self::PixelRangeIterator;

    /// Returns an iterator of booleans for whether or not a pixel should be considered for a
    /// subpage.
    ///
    /// This is a complement to [`pixel_ranges`][MelexisCamera::pixel_ranges], in that it lets an
    /// implementation load extra memory when it's more efficient but then ignore the pixels for
    /// later computations.
    ///
    /// The iterator should return true when the pixel is part of this subpage, and false when it is
    /// not. The ordering is rows, then columns. The iterator must not be infinite; it should only
    /// yield as many values as there are pixels.
    fn pixels_in_subpage(
        subpage: Subpage,
        access_pattern: AccessPattern,
    ) -> Self::PixelsInSubpageIterator;

    /// The address for T<sub>a<sub>V<sub>BE</sub></sub></sub>.
    fn t_a_v_be() -> Address;

    /// The address for T<sub>a<sub>PTAT</sub></sub>
    fn t_a_ptat() -> Address;

    /// The address of the compensation pixel for the given subpage.
    fn compensation_pixel(subpage: Subpage) -> Address;

    /// The address of the current gain.
    fn gain() -> Address;

    /// The address for V<sub>DD<sub>pixel</sub></sub>.
    fn v_dd_pixel() -> Address;

    /// Calculate the ADC resolution correction factor
    fn resolution_correction(calibrated_resolution: u8, current_resolution: u8) -> f32;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PixelAddressRange {
    pub(crate) start_address: Address,
    pub(crate) buffer_offset: usize,
    pub(crate) length: usize,
}

/// A helper function for calculating the sensitivity correction coefficients
/// (Alpha<sub>corr<sub>range<sub>n</sub></sub></sub>) for the different temperature ranges.
///
/// This function will `panic` if the passed in slices both do not have exactly `N` elements.
pub(crate) fn alpha_correction_coefficients<const NUM_RANGES: usize>(
    basic_range: usize,
    corner_temperatures: &[i16],
    k_s_to: &[f32],
) -> [f32; NUM_RANGES] {
    // This is the actual calculation. The values are built up recursively from the base case of
    // the basic range (which doesn't need correcting, so it's 1).
    // Memoizing would be nice here, but these calculations are done only once, at start up, so the
    // impact isn't that big.
    let results: ArrayVec<f32, NUM_RANGES> = (0..NUM_RANGES)
        .map(|n| alpha_corr_n(n, basic_range, corner_temperatures, k_s_to))
        .collect();
    results
        .into_inner()
        .expect("The Rust-range 0..NUM_RANGES should fill an array of NUM_RANGES elements")
}

/// The actual calculations for [alpha_correction_coefficients] as a recursive function. Memoizing
/// would be nice, but these calculations are only performed once, at start up.
fn alpha_corr_n(n: usize, basic_range: usize, ct: &[i16], k_s_to: &[f32]) -> f32 {
    match n.cmp(&basic_range) {
        core::cmp::Ordering::Equal => 1f32,
        core::cmp::Ordering::Less => {
            (1f32 + k_s_to[n] * f32::from(ct[n + 1] - ct[n])).recip()
                * alpha_corr_n(n + 1, basic_range, ct, k_s_to)
        }
        core::cmp::Ordering::Greater => {
            (1f32 + k_s_to[n - 1] * f32::from(ct[n] - ct[n - 1]))
                * alpha_corr_n(n - 1, basic_range, ct, k_s_to)
        }
    }
}
