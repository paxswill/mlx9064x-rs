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
//!   surface emits relative to its temperature.
//! * K: Prefix for constants.
//! * PTAT: Proportional to ambient temperature
//! * T<sub>a</sub>: Ambient temperature
//! * T<sub>a<sub>0</sub></sub>: Ambient temperature reference, 25.0 ℃. If it looks like 0, it's
//!   probably "o" as this value is really only used in one place.
//! * T<sub>o</sub>: Object temperature, meaning the temperature an individual pixel has detected
//!   for an object.
//! * V<sub>DD</sub>: Pixel supply voltage
//! * V<sub>DD<sub>25</sub></sub>: Pixel supply voltage reference at 25.0 ℃

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;

use crate::error::Error;
use crate::register::{ControlRegister, Subpage};

/// This trait provides access to the module-specific calibration data.
///
/// Each MLX9064\* camera has calibration data from the factory stored on its EEPROM. The
/// factory-provided data is then used as the input to generate the constants needed to convert the
/// raw output of the camera into concrete temperatures (or even just raw infrared radition
/// measurements). The naming scheme for the methods in this trait is taken from the names of the
/// variables used in the formulas in the datasheet. Most users of this library can make use fo the
/// provided implementations, but if you're trying to minimize memory usage or tweak performance
/// for a specific use case, this might be a way to do it.
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
    fn k_v_ptat(&self) -> i16;

    /// Temperature proportional to ambient temperature constant (K<sub>T<sub>PTAT</sub></sub>).
    fn k_t_ptat(&self) -> i16;

    /// Voltage proportional to ambient temperature at 25℃ (V<sub>PTAT<sub>25</sub></sub>).
    fn v_ptat_25(&self) -> i16;

    /// Sensitivity proportional to ambient temperature (α<sub>PTAT</sub>).
    fn alpha_ptat(&self) -> u16;

    /// The gain constant. Usually written as <var>GAIN</var> in the datasheets.
    fn gain(&self) -> i16;

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
    /// This method returns a slice of values equal in length to [`corner_temperatures`].
    fn k_s_to(&self) -> &[f32];

    /// Temperature range sensitivity correction (α<sub>correction</sub>(n))
    ///
    /// Like [`k_s_to`], the name of this method is slightly different that the naming in the
    /// datasheet. Also like `k_s_to`, this method returns a slice of values with a length equal to
    /// the length of the slice returned by  [`corner_temperatures`],
    fn alpha_correction(&self) -> &[f32];

    /// The index of the "native" temperature range.
    ///
    /// Temperature ranges (delimited by the control temperatures) outside of the "native" range
    /// are "extended temperature ranges" and require extra processing for accuracy. The datasheets
    /// do not use the term "native range", but for this library it is defined as the index of the
    /// range (0-indexed) that has α<sub>correction</sub>(r) = 1. Also note that this library uses
    /// 0-indexing as opposed to the datasheets that use 1-indexing.
    fn native_range(&self) -> usize;

    /// The emissivity stored on the device.
    ///
    /// Not all devices support storing the emissivity, in which case they should return [None]
    /// (which is what the provided implementation does).
    fn emissivity(&self) -> Option<f32> {
        None
    }

    /// Offset<sub>reference<sub>pixel</sub></sub>, for all pixels.
    ///
    /// The returned slice covers all pixels.
    fn offset_reference_pixels(&self, subpage: Subpage) -> &[i16];

    /// The offset reference value for the compensation pixel corresponding to the given subpage
    /// (Offset<sub>reference<sub>CP</sub></sub>).
    fn offset_reference_cp(&self, subpage: Subpage) -> i16;

    /// α<sub>pixel</sub>
    ///
    /// The returned slice covers all pixels.
    fn alpha_pixels(&self, subpage: Subpage) -> &[f32];

    /// The sensitivity calibration value for the compensation pixel for the given subpage
    /// (α<sub>CP</sub>).
    fn alpha_cp(&self, subpage: Subpage) -> f32;

    /// K<sub>V<sub>pixel</sub></sub>
    ///
    /// The returned slice covers all pixels.
    fn k_v_pixels(&self, subpage: Subpage) -> &[f32];

    /// The voltage calibration constant for the compensation pixel for the given subpage
    /// (K<sub>V<sub>CP</sub></sub>).
    fn k_v_cp(&self, subpage: Subpage) -> f32;

    /// K<sub>T<sub>a</sub>pixel</sub>
    ///
    /// The returned slice covers all pixels.
    fn k_ta_pixels(&self, subpage: Subpage) -> &[f32];

    /// The ambient temperature calibration constant for the compensation pixel for the given
    /// subpage (K<sub>T<sub>a</sub>CP</sub>).
    fn k_ta_cp(&self, subpage: Subpage) -> f32;

    /// Temperature gradient coefficient (TGC).
    ///
    /// Some devices do not support a TGC (it can also be disabled manually on other devices).
    fn temperature_gradient_coefficient(&self) -> Option<f32>;
}
/// Marker newtype for addresses accessible over I<sup>2</sup>C.
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
pub struct Address(u16);

impl Address {
    pub(crate) fn as_bytes(&self) -> [u8; 2] {
        self.0.to_be_bytes()
    }
}

impl From<u16> for Address {
    fn from(raw_address: u16) -> Self {
        Self(raw_address)
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
pub trait MelexisCamera: Sized {
    type PixelRangeIterator: IntoIterator<Item = PixelAddressRange>;

    type PixelsInSubpageIterator: Iterator<Item = bool>;

    /// Create a new camera with the current control register and a dump of the EEPROM.
    fn new<I2C>(register: ControlRegister, eeprom: &[u8]) -> Result<Self, Error<I2C>>
    where
        I2C: i2c::WriteRead + i2c::Write;

    /// Ranges of memory that should be read to load a subpage's data from RAM.
    ///
    /// Different cameras with different [access patterns][crate::AccessPattern] have different optimal
    /// ways of loading data from RAM. In some cases loading by row and then ignoring half the data
    /// may be appropriate, in other loading individual pixels may be best.
    fn pixel_ranges(&self, subpage: Subpage) -> Self::PixelRangeIterator;

    /// Returns an iterator of booleans for whether or not a pixel should be considered for a
    /// subpage.
    ///
    /// This is a complement to [`pixel_ranges`][MelexisCamera::pixel_ranges], in that it lets an
    /// implementation load extra memory when it's more efficient but then ignore the pixels for
    /// later computations.
    ///
    /// The iterator should return tru when the pixel is part of this subpage, and false when it is
    /// not. The ordering is rows, then columns. The iterator must not be infinite; it should only
    /// yield as many values as there are pixels.
    fn pixels_in_subpage(&self, subpage: Subpage) -> Self::PixelsInSubpageIterator;

    /// The address for T<sub>a<sub>V<sub>BE</sub></sub></sub>.
    fn t_a_v_be(&self) -> Address;

    /// The address for T<sub>a<sub>PTAT</sub></sub>
    fn t_a_ptat(&self) -> Address;

    /// The address of the compensation pixel for the given subpage.
    fn compensation_pixel(&self, subpage: Subpage) -> Address;

    /// The address of the current gain.
    fn gain(&self) -> Address;

    /// The address for V<sub>DD<sub>pixel</sub></sub>.
    fn v_dd_pixel(&self) -> Address;

    fn calibration(&self) -> &dyn CalibrationData;

    fn update_control_register(&mut self, register: ControlRegister);

    /// Calculate the ADC resolution correction factor
    fn resolution_correction(&self) -> f32;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PixelAddressRange {
    pub(crate) start_address: Address,
    pub(crate) length: usize,
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
    match n.cmp(&native_range) {
        core::cmp::Ordering::Equal => 1f32,
        core::cmp::Ordering::Less => {
            (1f32 + k_s_to[n] * f32::from(ct[n + 1] - ct[n])).recip()
                * alpha_corr_n(n + 1, native_range, ct, k_s_to)
        }
        core::cmp::Ordering::Greater => {
            (1f32 + k_s_to[n - 1] * f32::from(ct[n] - ct[n - 1]))
                * alpha_corr_n(n - 1, native_range, ct, k_s_to)
        }
    }
}
