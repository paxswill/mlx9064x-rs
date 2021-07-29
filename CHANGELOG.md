# Unreleased

* Added MLX90641 support.

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
