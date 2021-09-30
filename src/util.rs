// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use crate::common::PixelAddressRange;
use crate::register::Subpage;

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

/// An iterator that yields [`common::PixelAddressRange`] for interleaved data.
///
/// # Generic parameters
/// * `STRIDE_LENGTH` is the length of the stride in number of addresses.
/// * `NUM_STRIDES` is the number of strides.
/// * `PIXEL_START_ADDRESS` is the address of the very first pixel in the pixel range.
#[derive(Clone, Copy, Debug)]
pub struct SubpageInterleave<
    const STRIDE_LENGTH: u16,
    const NUM_STRIDES: u16,
    const PIXEL_START_ADDRESS: u16,
> {
    stride_count: u16,
    base_address: u16,
}

impl<const STRIDE_LENGTH: u16, const NUM_STRIDES: u16, const PIXEL_START_ADDRESS: u16>
    SubpageInterleave<STRIDE_LENGTH, NUM_STRIDES, PIXEL_START_ADDRESS>
{
    pub(crate) fn new(subpage: Subpage) -> Self {
        let starting_address: u16 = match subpage {
            Subpage::Zero => PIXEL_START_ADDRESS,
            Subpage::One => PIXEL_START_ADDRESS + STRIDE_LENGTH,
        };
        Self {
            stride_count: 0,
            base_address: starting_address,
        }
    }
}

impl<const STRIDE_LENGTH: u16, const NUM_STRIDES: u16, const PIXEL_START_ADDRESS: u16>
    core::iter::Iterator for SubpageInterleave<STRIDE_LENGTH, NUM_STRIDES, PIXEL_START_ADDRESS>
{
    type Item = PixelAddressRange;

    fn next(&mut self) -> Option<Self::Item> {
        match self.stride_count.cmp(&NUM_STRIDES) {
            core::cmp::Ordering::Less => {
                let next_value = PixelAddressRange {
                    start_address: (self.base_address + self.stride_count * STRIDE_LENGTH).into(),
                    // buffer_offset and length are both counting *bytes*, not addresses so we need
                    // to multiply them by 2
                    buffer_offset: (self.stride_count * STRIDE_LENGTH * 2) as usize,
                    length: (STRIDE_LENGTH * 2) as usize,
                };
                self.stride_count += 1;
                Some(next_value)
            }
            _ => None,
        }
    }
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
