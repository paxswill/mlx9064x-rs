// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

/// Define addition and subtraction for address enumerations.
#[macro_export]
macro_rules! address_enum_ops {
    ($typ:ident) => {
        // Add
        impl ::core::ops::Add<$typ> for $typ {
            type Output = u16;
            fn add(self, other: $typ) -> Self::Output {
                self as u16 + other as u16
            }
        }
        impl ::core::ops::Add<&$typ> for $typ {
            type Output = u16;
            fn add(self, other: &$typ) -> Self::Output {
                self as u16 + *other as u16
            }
        }
        impl ::core::ops::Add<$typ> for &$typ {
            type Output = u16;
            fn add(self, other: $typ) -> Self::Output {
                *self as u16 + other as u16
            }
        }
        impl ::core::ops::Add<&$typ> for &$typ {
            type Output = u16;
            fn add(self, other: &$typ) -> Self::Output {
                *self as u16 + *other as u16
            }
        }
        // Sub
        impl ::core::ops::Sub<$typ> for $typ {
            type Output = u16;
            fn sub(self, other: $typ) -> Self::Output {
                self as u16 - other as u16
            }
        }
        impl ::core::ops::Sub<&$typ> for $typ {
            type Output = u16;
            fn sub(self, other: &$typ) -> Self::Output {
                self as u16 - *other as u16
            }
        }
        impl ::core::ops::Sub<$typ> for &$typ {
            type Output = u16;
            fn sub(self, other: $typ) -> Self::Output {
                *self as u16 - other as u16
            }
        }
        impl ::core::ops::Sub<&$typ> for &$typ {
            type Output = u16;
            fn sub(self, other: &$typ) -> Self::Output {
                *self as u16 - *other as u16
            }
        }
    };
}

#[macro_export]
macro_rules! expose_member {
    ($name:ident, $typ:ty) => {
        fn $name(&self) -> $typ {
            self.$name
        }
    };
    (&$name:ident, $typ:ty) => {
        fn $name(&self) -> &$typ {
            &self.$name
        }
    };
}
