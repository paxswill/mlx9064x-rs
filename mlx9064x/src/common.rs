// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
//! Common functionality between MLX90640 and MLX90641 cameras.
use core::fmt;

use arrayvec::ArrayVec;
use bitvec::order::BitOrder;
use bitvec::slice::{BitSlice, IterOnes};
use bitvec::store::BitStore;
use embedded_hal::blocking::i2c;

use crate::calculations::RamData;
use crate::error::Error;
use crate::register::{AccessPattern, Resolution, Subpage};
use crate::util::Sealed;

/// A trait for types that can be created by reading data from an I²C device.
pub trait FromI2C<I2C> {
    type Error;
    type Ok;

    /// Create an instance of a type using data retrieved over I²C.
    fn from_i2c(bus: &mut I2C, i2c_address: u8) -> Result<Self::Ok, Self::Error>;
}

/// A trait for types that can be written to an I²C device.
pub trait ToI2C<I2C> {
    type Error;

    /// Write the value of this type to the specified I²C device.
    fn to_i2c(&self, bus: &mut I2C, i2c_address: u8) -> Result<(), Self::Error>;
}

/// A trait for flagging individual pixels.
pub trait FlaggedPixels {
    /// Check if any pixels are flagged.
    fn any(&self) -> bool;

    type FlaggedIterator: Iterator<Item = usize>;

    /// Iterate over the indexes of flagged pixels.
    fn iter_flagged(&self) -> Self::FlaggedIterator;
}

impl<'a, T, O> FlaggedPixels for &'a BitSlice<T, O>
where
    T: BitStore,
    O: BitOrder,
{
    fn any(&self) -> bool {
        BitSlice::any(self)
    }

    type FlaggedIterator = IterOnes<'a, T, O>;

    fn iter_flagged(&self) -> Self::FlaggedIterator {
        self.iter_ones()
    }
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
    fn resolution(&self) -> Resolution;

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

    type FailedPixels: FlaggedPixels;

    /// Any pixels that have completely failed.
    ///
    /// These pixels should be skipped during temperature calculations.
    fn failed_pixels(&'a self) -> Self::FailedPixels;

    type OutlierPixels: FlaggedPixels;

    /// Pixels that are outside specification during calibration.
    ///
    /// Pixels that might also drift over a long period of time are also flagged in this manner.
    fn outlier_pixels(&'a self) -> Self::OutlierPixels;
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

    type PixelsInSubpageIterator: IntoIterator<Item = bool>;

    /// Ranges of memory that should be read to load a subpage's data from RAM.
    ///
    /// Different cameras with different [access patterns][crate::AccessPattern] have different optimal
    /// ways of loading data from RAM. In some cases loading by row and then ignoring half the data
    /// may be appropriate, in other loading individual pixels may be more efficient.
    ///
    /// The returned iterator will yield at most [`Self::HEIGHT`] items.
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
    fn resolution_correction(
        calibrated_resolution: Resolution,
        current_resolution: Resolution,
    ) -> f32;

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

/// Read a frame of data from the camera's memory.
pub fn read_ram<Cam, I2C, const HEIGHT: usize>(
    bus: &mut I2C,
    i2c_address: u8,
    access_pattern: AccessPattern,
    subpage: Subpage,
    pixel_data_buffer: &mut [u8],
) -> Result<RamData, Error<I2C>>
where
    Cam: MelexisCamera,
    I2C: i2c::WriteRead + i2c::Write,
{
    // Pick a maximum size of HEIGHT, as the worst access pattern is still by rows
    let pixel_ranges: ArrayVec<PixelAddressRange, HEIGHT> =
        Cam::pixel_ranges(subpage, access_pattern)
            .into_iter()
            .collect();
    for range in pixel_ranges.iter() {
        let offset = range.buffer_offset;
        let address_bytes = range.start_address.as_bytes();
        bus.write_read(
            i2c_address,
            &address_bytes[..],
            &mut pixel_data_buffer[offset..(offset + range.length)],
        )
        .map_err(Error::I2cWriteReadError)?;
    }
    // And now to read the non-pixel information out
    RamData::from_i2c::<I2C, Cam>(bus, i2c_address, subpage).map_err(Error::I2cWriteReadError)
}
