// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

use core::cmp;
use core::ops;

/// A trait for numbers that can be converted into a number used in the [`calculations`] module.
///
/// This is a blend of [`num_traits::NumCast`] in that the conversions to *not* need to be precise,
/// and [`From`] in that the conversions are infallible. Another reason for this trait (and its
/// choice in naming) is to be easier to use than `T::from(value)`, which needs to be disambiguated
/// each time because of the previously mentioned traits.
pub trait Coerce<T> {
    /// Convert
    fn coerce(value: T) -> Self;
}

impl<T> Coerce<T> for T {
    fn coerce(value: T) -> Self {
        value
    }
}

/// Implement [`Coerce`] for the given type.
///
/// There are two variants available, one that uses [`From`], the other using `as` to cast
/// primitives.
#[macro_export]
macro_rules! impl_coerce {
    ($dest:ty, from $source:ty ) => {
        impl Coerce<$source> for $dest {
            fn coerce(value: $source) -> Self {
                $dest::from(value)
            }
        }
    };
    ($dest:ty, from $($source:ty),+ ) => {
        $(impl_coerce!($dest, from $dest););+
    };
    (as $dest:ty, $source:ty) => {
        impl Coerce<$source> for $dest {
            fn coerce(value: $source) -> Self {
                value as $dest
            }
        }
    };
    (as $dest:ty, $($source:ty),+) => {
        $(impl_coerce!(as $dest, $source);)+
    };
}
impl_coerce!(as f32, u8, i8, u16, i16, f64);
impl_coerce!(as f64, u8, i8, u16, i16, f32);

pub trait Num:
    Copy
    + cmp::PartialEq
    + cmp::PartialOrd
    + num_traits::NumOps
    + ops::AddAssign
    + ops::DivAssign
    + ops::SubAssign
    + Coerce<f64>
    + Coerce<i16>
    + Coerce<u16>
    + Coerce<i8>
    + Coerce<u8>
{
    /// Positive zero.
    const ZERO: Self;

    /// The number 1.
    const ONE: Self;

    /// The freezing point of water in kelvins (273.15).
    const KELVINS_TO_CELSIUS: Self;

    /// The number 1/4.
    const ONE_QUARTER: Self;

    /// The number 3.3.
    const THREE_POINT_THREE: Self;

    /// The number 25.0.
    const TWENTY_FIVE: Self;

    /// The number 2^18.
    ///
    /// This is a scaling factor used when calculating [v_ptat_art].
    const TWO_RAISED_EIGHTEEN: Self;

    /// A wrapper around [`Coerce`] to allow using a turbofish to disambiguate types.
    fn coerce_from<T>(value: T) -> Self
    where
        Self: Coerce<T>,
    {
        <Self as Coerce<T>>::coerce(value)
    }

    /// Calculate the reciprocal (inverse) of a number.
    fn recip(self) -> Self;

    /// Raise `self` to the given (integer) power.
    fn powi(self, n: i32) -> Self;

    /// Calculate 2 raised to `self`.
    fn exp2(self) -> Self;

    /// Calculate the 4th root of `self`.
    fn fourth_root(self) -> Self;
}

/// Implements [`Num`] for the given builtin float type.
#[macro_export]
macro_rules! impl_num {
    ($typ:ty) => {
        impl Num for $typ {
            const ZERO: Self = 0.0;

            const ONE: Self = 1.0;

            const KELVINS_TO_CELSIUS: Self = 273.15;

            const ONE_QUARTER: Self = 0.25;

            const THREE_POINT_THREE: Self = 3.3;

            const TWENTY_FIVE: Self = 25.0;

            const TWO_RAISED_EIGHTEEN: Self = 2u32.pow(18) as Self;

            fn recip(self) -> Self {
                <Self as num_traits::Float>::recip(self)
            }

            fn powi(self, n: i32) -> Self {
                <Self as num_traits::Float>::powi(self, n)
            }

            fn exp2(self) -> Self {
                <Self as num_traits::Float>::exp2(self)
            }

            fn fourth_root(self) -> Self {
                <Self as num_traits::Float>::powf(self, 0.25)
            }
        }
    };
}
impl_num!(f32);
impl_num!(f64);
