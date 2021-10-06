// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
//! Common functionality between MLX90640 and MLX90641 cameras.
use core::fmt;

use arrayvec::ArrayVec;

use crate::register::{AccessPattern, Subpage};
use crate::util::{self, Sealed};

/// A trait for types that can be created by reading data from an I²C device.
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
#[doc = include_str!("katex.html")]
pub trait CalibrationData<'a> {
    /// The camera model this caliberation data is for.
    type Camera: MelexisCamera;

    /// Pixel supply voltage constant ($K_{V_{DD}}$).
    fn k_v_dd(&self) -> i16;

    /// Constant for pixel supply voltage at 25℃ ($K_{V_{DD_{25}}}$).
    fn v_dd_25(&self) -> i16;

    /// ADC resolution this camera was calibrated at.
    // TODO: Should this return `Resolution`?
    fn resolution(&self) -> u8;

    /// Pixel supply voltage ($K_{DD_0}$).
    ///
    /// This is the voltage supplied to the device, and should be 3.3V for the MLX90640 and
    /// MLX90641. The default implementation is hardcoded to return `3.3f32`, but if there's a
    /// reason it needs to be overridden, it's possible.
    fn v_dd_0(&self) -> f32 {
        3.3f32
    }

    /// Voltage proportional to ambient temperature constant ($K_{V_{PTAT}}$).
    fn k_v_ptat(&self) -> f32;

    /// Temperature proportional to ambient temperature constant ($K_{T_{PTAT}}$).
    fn k_t_ptat(&self) -> f32;

    /// Voltage proportional to ambient temperature at 25℃ ($V_{PTAT_{25}}$).
    fn v_ptat_25(&self) -> f32;

    /// Sensitivity proportional to ambient temperature ($\alpha_{PTAT}$).
    fn alpha_ptat(&self) -> f32;

    /// The gain constant. Usually written as <var>GAIN</var> in the datasheets.
    fn gain(&self) -> f32;

    /// Sensitivity constant for ambient temperature ($K_{S_{T_{a}}}$).
    fn k_s_ta(&self) -> f32;

    /// A slice of the "corner temperatures".
    ///
    /// These define temperature ranges with different sensitivity characteristics. They are
    /// indexed in the datasheet starting from 1 but everything in this library is 0-indexed, so be
    /// aware of the difference.
    fn corner_temperatures(&self) -> &[i16];

    /// Constant for the object temperature sensitivity ($K_{s_{T_{o}N}}$)
    /// depending on the temperature range.
    ///
    /// This is a slight variance from the datasheet's nomenclature. In the symbol above,
    /// <var>N</var> is the index of the temperature range, which the datasheet normally just
    /// writes out (ex: $K_{S_{T_{o}1}}$ through how every many temperature
    /// ranges the camera has).
    ///
    /// This method returns a slice of values equal in length to
    /// [`corner_temperatures`](CalibrationData::corner_temperatures).
    fn k_s_to(&self) -> &[f32];

    /// Temperature range sensitivity correction ($\alpha_{\text{correction}_{N}}$)
    ///
    /// Like [`k_s_to`], the name of this method is slightly different that the naming in the
    /// datasheet. Also like `k_s_to`, this method returns a slice of values with a length equal to
    /// the length of the slice returned by
    /// [`corner_temperatures`](CalibrationData::corner_temperatures),
    ///
    /// [`k_s_to`]: CalibrationData::k_s_to
    fn alpha_correction(&self) -> &[f32];

    /// The emissivity stored on the device.
    ///
    /// Not all devices support storing the emissivity, in which case they should return [None]
    /// (which is what the provided implementation does).
    fn emissivity(&self) -> Option<f32> {
        None
    }

    type OffsetReferenceIterator: Iterator<Item = &'a i16>;

    /// An iterator over the per-pixel offset reference values for the given subpage
    /// ($\text{Offset}_\text{reference}(i, j)$).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn offset_reference_pixels(&'a self, subpage: Subpage) -> Self::OffsetReferenceIterator;

    /// The offset reference value for the compensation pixel corresponding to the given subpage
    /// ($\text{Offset}\_{\text{reference}\_{CP}}$).
    fn offset_reference_cp(&self, subpage: Subpage) -> i16;

    type AlphaIterator: Iterator<Item = &'a f32>;

    /// An iterator over the per-pixel sensitivity calibration values ($\alpha_{pixel}(i, j)$).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn alpha_pixels(&'a self, subpage: Subpage) -> Self::AlphaIterator;

    /// The sensitivity calibration value for the compensation pixel for the given subpage
    /// ($\alpha_{CP}$).
    fn alpha_cp(&self, subpage: Subpage) -> f32;

    type KvIterator: Iterator<Item = &'a f32>;

    /// An iterator over the per-pixel voltage calibration constants ($K_{V_{pixel}}$).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn k_v_pixels(&'a self, subpage: Subpage) -> Self::KvIterator;

    /// The voltage calibration constant for the compensation pixel for the given subpage
    /// ($K_{V_{CP}}$).
    fn k_v_cp(&self, subpage: Subpage) -> f32;

    type KtaIterator: Iterator<Item = &'a f32>;

    /// The per pixel ambient temperature calibration constants ($K_{T_{a}pixel}$).
    ///
    /// The iterator must yield pixels by row, then by columns, with the rows increasing from left
    /// to right and the columns increasing from top to bottom. The iterator must yield *all*
    /// pixels, even if they would not normally be present in the given subpage.
    fn k_ta_pixels(&'a self, subpage: Subpage) -> Self::KtaIterator;

    /// The ambient temperature calibration constant for the compensation pixel for the given
    /// subpage ($K_{T_{a}CP}$).
    fn k_ta_cp(&self, subpage: Subpage) -> f32;

    /// Temperature gradient coefficient (<var>TGC</var>).
    ///
    /// Some devices do not support a TGC (it can also be disabled manually on other devices).
    fn temperature_gradient_coefficient(&self) -> Option<f32>;

    type AccessPatternCompensation: Iterator<Item = Option<&'a f32>>;

    /// A sequence of per-pixel correction values that are added to the pixel gain value.
    ///
    /// The MLX90640 can be used in interleaved mode, but for optimal performance a correction
    /// needs to be applied. This value is summed with the pixel gain value and reference offset
    /// (the reference offset being scaled relative to the temperature difference).
    fn access_pattern_compensation_pixels(
        &'a self,
        access_pattern: AccessPattern,
    ) -> Self::AccessPatternCompensation;

    /// Equivalent to [`Self::access_pattern_compensation_pixels`] for compensation pixels.
    fn access_pattern_compensation_cp(
        &self,
        subpage: Subpage,
        access_pattern: AccessPattern,
    ) -> Option<f32>;
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

/// Define common constants specific to a camera model.
///
/// The values from this trait are common between all cameras of a single model, and do not depend
/// on the calibration values from a specific camera.
///
/// This is a sealed trait, and can only be implemented by types defined within this crate.
#[doc = include_str!("katex.html")]
pub trait MelexisCamera: Sealed {
    type PixelRangeIterator: IntoIterator<Item = PixelAddressRange>;

    /// Ranges of memory that should be read to load a subpage's data from RAM.
    ///
    /// Different cameras with different [access patterns][crate::AccessPattern] have different optimal
    /// ways of loading data from RAM. In some cases loading by row and then ignoring half the data
    /// may be appropriate, in other loading individual pixels may be more efficient.
    ///
    /// The returned iterator will yield at most [`Self::HEIGHT`] items.
    fn pixel_ranges(subpage: Subpage, access_pattern: AccessPattern) -> Self::PixelRangeIterator;

    /// Filter the given iterator, skipping items that are not present in this subpage.
    ///
    /// The elements must be in row-major order, and the iterator must not be infinite. Items are
    /// skipped using [`Iterator::nth`].
    fn filter_by_subpage<I: IntoIterator>(
        iter: I,
        subpage: Subpage,
        access_pattern: AccessPattern,
    ) -> AccessPatternFilter<I::IntoIter>;

    /// The address for $T_{a_{V_{BE}}}$.
    const T_A_V_BE: Address;

    /// The address for $T_{a_{PTAT}}$.
    const T_A_PTAT: Address;

    /// The address of the compensation pixel for the given subpage.
    fn compensation_pixel(subpage: Subpage) -> Address;

    /// The address of the current gain.
    const GAIN: Address;

    /// The address for $V_{DD_{pixel}}$.
    const V_DD_PIXEL: Address;

    /// Calculate the ADC resolution correction factor
    fn resolution_correction(calibrated_resolution: u8, current_resolution: u8) -> f32;

    /// The index of the basic temperature range.
    ///
    /// Temperature ranges (delimited by the control temperatures) outside of the basic range
    /// are "extended temperature ranges" and require extra processing for accuracy. The datasheets
    /// don't give a generic definition of the basic range, but for this library it is defined as
    /// the temperature range with α<sub>correction</sub>(r) = 1. Also note that this library uses
    /// 0-indexing as opposed to the datasheets that use 1-indexing.
    const BASIC_TEMPERATURE_RANGE: usize;

    /// The expected amount of self-heating for this camera.
    ///
    /// In normal operation the camera generates some heat. If $T_r$ is not available, it
    /// can be calculated by subtracting this value from $T_a$.
    const SELF_HEATING: f32;

    /// The height of the thermal image in pixels.
    const HEIGHT: usize;

    /// The width of the thermal image in pixels.
    const WIDTH: usize;

    /// The total number of pixels in the thermal image.
    const NUM_PIXELS: usize;
}

/// A range of camera memory.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PixelAddressRange {
    /// The address of memory to start reading from.
    pub(crate) start_address: Address,
    /// The offset of this range of pixels in the larger image.
    pub(crate) buffer_offset: usize,
    /// The number of bytes in this range of pixels.
    ///
    /// Remember that each pixel is *two* bytes.
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

/// An iterator adapter that skips pixels that are not in a given subpage.
///
/// This type is created using the [`MelexisCamera::filter_by_subpage`] function.
#[derive(Clone, Debug)]
pub enum AccessPatternFilter<I: Iterator> {
    #[doc(hidden)]
    Chess(util::ByDiagonals<I>),
    #[doc(hidden)]
    Interlaced(util::Interlaced<I, 2>),
    #[doc(hidden)]
    Unfiltered(I),
}

impl<I: Iterator> AccessPatternFilter<I> {
    pub(crate) fn new<S>(
        source: S,
        subpage: Subpage,
        access_pattern: AccessPattern,
        width: usize,
    ) -> Self
    where
        S: IntoIterator<IntoIter = I>,
    {
        let source = source.into_iter();
        match access_pattern {
            AccessPattern::Chess => Self::Chess(util::ByDiagonals::new(
                source,
                subpage == Subpage::Zero,
                width,
            )),
            AccessPattern::Interleave => {
                Self::Interlaced(util::Interlaced::new(source, subpage as usize, width))
            }
        }
    }

    pub(crate) fn unfiltered<S>(source: S) -> Self
    where
        S: IntoIterator<IntoIter = I>,
    {
        Self::Unfiltered(source.into_iter())
    }

    fn inner_mut(&mut self) -> &mut dyn Iterator<Item = I::Item> {
        match self {
            AccessPatternFilter::Chess(inner) => inner,
            AccessPatternFilter::Interlaced(inner) => inner,
            AccessPatternFilter::Unfiltered(inner) => inner,
        }
    }
}

impl<I: Iterator> Iterator for AccessPatternFilter<I> {
    type Item = I::Item;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner_mut().next()
    }
}
