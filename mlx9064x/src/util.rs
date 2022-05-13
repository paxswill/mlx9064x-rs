// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

/// The word size of the camera in terms of 8-bit bytes.
pub(crate) const WORD_SIZE: usize = (u16::BITS / u8::BITS) as usize;

/// Define addition and subtraction for address enumerations.
#[doc(hidden)]
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

#[doc(hidden)]
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

/// This is a very small reimplementation of [bytes::Buf] with just the parts needed for this
/// crate.
///
/// If/when `bytes` removes their dependency on `alloc`, this can be removed.
///
/// [bytes::Buf]: https://docs.rs/bytes/*/bytes/trait.Buf.html
pub(crate) trait Buffer {
    fn advance(&mut self, cnt: usize);
    fn get_u8(&mut self) -> u8;
    fn get_i8(&mut self) -> i8;
    fn get_u16(&mut self) -> u16;
    fn get_i16(&mut self) -> i16;
}

impl Buffer for &[u8] {
    fn advance(&mut self, cnt: usize) {
        *self = &self[cnt..];
    }

    fn get_u8(&mut self) -> u8 {
        let (byte, rest) = self.split_at(1);
        *self = rest;
        u8::from_be_bytes([byte[0]])
    }

    fn get_i8(&mut self) -> i8 {
        let (byte, rest) = self.split_at(1);
        *self = rest;
        i8::from_be_bytes([byte[0]])
    }

    fn get_u16(&mut self) -> u16 {
        let (bytes, rest) = self.split_at(2);
        *self = rest;
        u16::from_be_bytes([bytes[0], bytes[1]])
    }

    fn get_i16(&mut self) -> i16 {
        let (bytes, rest) = self.split_at(2);
        *self = rest;
        i16::from_be_bytes([bytes[0], bytes[1]])
    }
}

/// A hidden public trait to mark certain traits as only implementable within this crate.
pub trait Sealed {}

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
    let num = match bytes.len() {
        1 => i16::from(bytes.get_i8()),
        _ => bytes.get_i16(),
    };
    let shift_amount = 16 - num_bits;
    (num << shift_amount) >> shift_amount
}

#[cfg(test)]
mod test {
    use super::Buffer;

    #[test]
    fn buffer_advance() {
        let data = b"\xde\xad\xbe\xef";
        let mut buf = &data[..];
        assert_eq!(buf, &data[..]);
        buf.advance(1);
        assert_eq!(buf, &data[1..]);
        buf.advance(2);
        assert_eq!(buf, &data[3..]);
    }

    #[test]
    fn buffer_get_u8() {
        let data = b"\xde\xad\xbe\xef";
        let mut buf = &data[..];
        let v = buf.get_u8();
        assert_eq!(v, 0xde);
        assert_eq!(buf, &data[1..]);
    }

    #[test]
    fn buffer_get_i8() {
        let data = b"\xde\xad\xbe\xef";
        let mut buf = &data[..];
        let v = buf.get_i8();
        assert_eq!(v, -34);
        assert_eq!(buf.len(), 3);
        assert_eq!(buf, &data[1..]);
    }

    #[test]
    fn buffer_get_u16() {
        let data = b"\xde\xad\xbe\xef";
        let mut buf = &data[..];
        let v = buf.get_u16();
        assert_eq!(v, 0xdead);
        assert_eq!(buf.len(), 2);
        assert_eq!(buf, &data[2..]);
    }

    #[test]
    fn buffer_get_i16() {
        let data = b"\xde\xad\xbe\xef";
        let mut buf = &data[..];
        let v = buf.get_i16();
        assert_eq!(v, -8531);
        assert_eq!(buf.len(), 2);
        assert_eq!(buf, &data[2..]);
    }

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
