[![Crates.io](https://img.shields.io/crates/v/mlx9064x)](https://crates.io/crates/mlx9064x)
[![docs.rs](https://img.shields.io/docsrs/mlx9064x?label=docs.rs)](https://docs.rs/mlx9064x/)
[![builds.sr.ht status](https://builds.sr.ht/~paxswill/mlx9064x.svg)](https://builds.sr.ht/~paxswill/mlx9064x?)

# mlx9064x

`mlx9064x` is a library for using the MLX90640 and MLX90641 thermal cameras from
Melexis. It's `no_std`, but these cameras require a fair bit of memory and
floating point processing power to use to their fullest extent.

## Roadmap

There's a few remaining features/tasks left before I consider this crate "done":

 - [ ] Implement and expose dead pixels. The calibration data marks dead or out
       of spec pixels. The manufacturer provided library also provides some
       basic interpolation functions to fill in dead pixels, but I feel that's
       better handled by a separate crate.
 - [x] Simplify access pattern iterators. I feel there's some extra performance
       as well as a simpler way to handle the "which pixels need to be updated"
       task that is currently handled by `MelexisCamera::pixels_in_subpage` (not
       to mention the naming of that function and `pixel_ranges` can be
       confisuing as to what they're doing).
 - [ ] Implement extended temperature range calculations. Outside of the basic
       temperature range (between 0 and a configurable upper limit, defaulting
       to 160 for the MLX90640 and 80 for the MLX90641), extra calculations need
       to be performed to account for non-linear sensitivity of the camera
       sensors. Right now only the basic temperature calculation is performed.
