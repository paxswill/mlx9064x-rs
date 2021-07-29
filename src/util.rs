// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

use bytes::Buf;

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

#[cfg(test)]
#[macro_export]
macro_rules! datasheet_test {
    ($name:ident, $value:literal) => {
        #[test]
        fn $name() {
            assert_eq!(eeprom().$name(), $value);
        }
    };
}

/// Check if the n-th bit is set.
///
/// Bits are 0-indexed, from the LSB.
pub(crate) fn is_bit_set<B>(value: B, index: usize) -> bool
where
    B: num_traits::PrimInt + num_traits::Unsigned,
{
    (value & (B::one() << index)) > B::zero()
}

/// Create a i16 from some bytes representing a `num_bits`-bit signed integer.
pub(crate) fn i16_from_bits(mut bytes: &[u8], num_bits: u8) -> i16 {
    let num = match bytes.remaining() {
        1 => i16::from(bytes.get_i8()),
        _ => bytes.get_i16(),
    };
    let shift_amount = 16 - num_bits;
    (num << shift_amount) >> shift_amount
}

#[cfg(test)]
mod test {
    #[test]
    fn is_bit_set() {
        for n in 0..16 {
            let value: u16 = 1 << n;
            assert!(
                super::is_bit_set(value, n),
                "is_bit_set was incorrect for bit {}",
                n
            );
        }
    }

    #[test]
    fn i16_from_bits() {
        assert_eq!(super::i16_from_bits(b"\x00\xff", 8), -1);
        assert_eq!(super::i16_from_bits(b"\x03\xff", 10), -1);
        // Now check that upper bits get ignored properly
        assert_eq!(super::i16_from_bits(b"\xf0\xff", 8), -1);
        assert_eq!(super::i16_from_bits(b"\xf3\xff", 10), -1);
    }
}
