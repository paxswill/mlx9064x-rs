# Branch Note
This branch refactors the specific floating point type used in the calculations
out so you could use f64 or f32 (or some other floating point type). My initial
intention was to allow the use of fixed point types, but I feel the complexity
it's adding to the codebase is too much for the small benefit (I'm not sure
there is a case where the greater precision of doubles makes sense for this
application). I'm leaving this branch up in case there ever is a use case for
using doubles.

[![Crates.io](https://img.shields.io/crates/v/mlx9064x)](https://crates.io/crates/mlx9064x)
[![docs.rs](https://img.shields.io/docsrs/mlx9064x?label=docs.rs)](https://docs.rs/mlx9064x/)
[![builds.sr.ht status](https://builds.sr.ht/~paxswill/mlx9064x.svg)](https://builds.sr.ht/~paxswill/mlx9064x?)

# mlx9064x

`mlx9064x` is a library for using the MLX90640 and MLX90641 thermal cameras from
Melexis. It's `no_std`, but these cameras require a fair bit of memory and
processing power to use fully.

This library is still under development, but the final API will be pretty close
to the current one.
