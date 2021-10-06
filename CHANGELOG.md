# Unreleased (v0.3.0)

## In `MelexisCamera`, `pixels_in_subpage()` is replaced by `filter_by_subpage()`
Instead of an interator of boolean values which can then be used with
`Iterator::filter()`, `filter_by_subpage()` wraps a provided iterator with a new
iterator adapter, `AccessPatternFilter`. This adapter allows more efficient
access by using `Iterator::nth()` to (potentially) skip elements without
computing them in `Iterator::next()`. To support this change the other
iterators in this crate now provide their own implementations of `nth()`,
`count()`, `size_hint()`, and `last()` and implement `ExactSizedIterator` and
`FusedIterator` when possible.

This change also impacts a number of functions in the `calculations` module:

* `raw_pixels_to_ir_data()`
* `raw_ir_to_temperatures()`
* `raw_pixels_to_temperatures()`

Where before these functions took a `valid_pixels` argument, they now use
`MelexisCamera::filter_by_subpage()` (via `CalibrationData::Camera`) to only
process pixels in the current subpage. This should also help reduce code size
with the removal of the `Px` generic parameter to these functions.

# v0.2.1
* Fix for two bugs preventing interleave mode from working with the MLX90640.
# v0.2.0

* Unintended dependency on `alloc` has been removed.
* T_r can now be specified to account for reflected radiation when emissivity is
  <1.
* The estimated self heating method for determining T_r is more accurate on 641s
  (they self-heat less, ~5 degrees compared to ~8 for the 640).
* Multiple functions on `MelexisCamera` are now associated constants.
* Image size constants (`HEIGHT`, `WIDTH`, `NUM_PIXELS`) are now on the
  `MelexisCamera` trait instead of free constants in the specific camera module.
* The `address_enum_ops` and `expose_member` macros are no longer part of the
  documented API.
* The `MelexisCamera` trait is now sealed.
* Interleave mode compensation for the MLX90640 is now implemented.

# v0.1.1

* Register access methods have been exposed for the low-level API.
* Implemented `From` for `FrameRate` to `Duration`.
* Reduced the number of I2C operations performed for common tasks.

# v0.1.0

* Improved/fixed MLX90641 support.

# v0.0.4

* Bug fixes to 90640 EEPROM handling
* Better 90640 testing with full example data.

# v0.0.3

* Added MLX90641 support.
* `Camera` renamed to `CameraDriver`, root level `-Camera` type aliases renamed
  to `-Driver`, and `camera` module renamed to `driver`.

# v0.0.2

* Added ambient temperature and image dimension access methods to base camera
  type.
* Error type was split to make internal use easier.
* FrameRate and Resolution's From implementations were rewritten to be useful to
  external users of the crate instead of an internal implementation detail.
* Documentation much improved.
* Improved testing with full device-level mocks.
* Stop sending 0-length read requests over I2C.
* High and low level APIs defined.

# v0.0.1

*Initial Release
