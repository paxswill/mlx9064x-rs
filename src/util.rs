// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
use core::iter;

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

/// An iterator adapter that only yields elements in a chess board pattern.
// This is hidden as it's an implementation detail for `AccessPatternFilter`.
#[doc(hidden)]
#[derive(Clone, Debug, PartialEq)]
pub struct ByDiagonals<I: Iterator> {
    index: usize,
    width: usize,
    next_nth: usize,
    source: I,
}

impl<I: Iterator> ByDiagonals<I> {
    /// Create an iterator that yields elements in a chess board pattern.
    ///
    /// `is_even` controls if the elements yielded are the black spaces (`is_even == true`) or the
    /// white spaces (`is_even == false`).
    pub(crate) fn new<S>(source: S, is_even: bool, width: usize) -> Self
    where
        S: IntoIterator<IntoIter = I>,
    {
        let index = (!is_even) as usize;
        Self {
            index,
            width,
            next_nth: index,
            source: source.into_iter(),
        }
    }

    fn update_next_nth(&mut self) {
        let column = self.index % self.width;
        // The last element of each row is also the first element of the next row, which means
        // from the second to last element we need to jump over *two* elements. Also a reminder
        // that nth() is 0-indexed.
        self.next_nth = match self.width.saturating_sub(column) {
            // Handle the case where self.width is less than 2 (with 0)
            0 | 1 => 0,
            2 => 2,
            _ => 1,
        };
        self.index += self.next_nth + 1;
    }
}

impl<I: Iterator> Iterator for ByDiagonals<I> {
    type Item = I::Item;

    fn next(&mut self) -> Option<Self::Item> {
        let n = self.next_nth;
        self.update_next_nth();
        self.source.nth(n)
    }
}

impl<I> iter::FusedIterator for ByDiagonals<I> where I: iter::FusedIterator {}

/// An iterator adapter that only yields items that are in every n-th row.
///
/// The number of rows that are interlaced is controlled by the `NUM_INTERLACED` parameter.
// This is hidden as it's an implementation detail for `AccessPatternFilter`.
#[doc(hidden)]
#[derive(Clone, Debug)]
pub struct Interlaced<I: Iterator, const NUM_INTERLACED: usize> {
    index: usize,
    width: usize,
    source: iter::Skip<I>,
    interlaced_row_index: usize,
}

impl<I: Iterator, const NUM_INTERLACED: usize> Interlaced<I, NUM_INTERLACED> {
    /// Create an iterator that yields elements by row but only every `n`th row.
    pub(crate) fn new<S>(source: S, n: usize, width: usize) -> Self
    where
        S: IntoIterator<IntoIter = I>,
    {
        let index = n * width;
        Self {
            index,
            width,
            // Skip the first few elements if needed.
            source: source.into_iter().skip(index),
            interlaced_row_index: n,
        }
    }
}

impl<I: Iterator, const NUM_INTERLACED: usize> Iterator for Interlaced<I, NUM_INTERLACED> {
    type Item = I::Item;

    fn next(&mut self) -> Option<Self::Item> {
        let row = self.index / self.width;
        let n = if row % NUM_INTERLACED == self.interlaced_row_index {
            0
        } else {
            self.width * (NUM_INTERLACED - 1)
        };
        self.index += n + 1;
        self.source.nth(n)
    }
}

impl<I: Iterator, const NUM_INTERLACED: usize> iter::FusedIterator for Interlaced<I, NUM_INTERLACED> where
    iter::Skip<I>: iter::FusedIterator
{
}

#[cfg(test)]
mod test {
    use super::{Buffer, ByDiagonals, Interlaced};

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

    #[test]
    fn by_diagonal() {
        #[rustfmt::skip]
        let source = [
            0, 1, 0, 1, 0, 1,
            1, 0, 1, 0, 1, 0,
            0, 1, 0, 1, 0, 1,
            1, 0, 1, 0, 1, 0,
        ];
        let diagonal0 = ByDiagonals::new(source.iter().copied(), true, 6);
        let diagonal1 = ByDiagonals::new(source.iter().copied(), false, 6);
        let mut count = 0;
        for (index, val) in diagonal0.enumerate() {
            assert_eq!(val, 0, "index: {}", index);
            count += 1;
        }
        assert_eq!(count, source.len() / 2);
        let mut count = 0;
        for (index, val) in diagonal1.enumerate() {
            assert_eq!(val, 1, "index: {}", index);
            count += 1;
        }
        assert_eq!(count, source.len() / 2);
    }

    #[test]
    fn interlaced_2_wide_2_rows() {
        #[rustfmt::skip]
        let source = [
            0, 0,
            1, 1,
            0, 0,
            1, 1,
            0, 0,
            1, 1,
            0, 0,
            1, 1,
        ];
        for n in 0..2 {
            let interlaced = Interlaced::<_, 2>::new(source.iter(), n, 2).copied();
            let mut count = 0;
            for (index, val) in interlaced.enumerate() {
                assert_eq!(val, n, "index: {}", index);
                count += 1;
            }
            assert_eq!(count, source.len() / 2);
        }
    }

    #[test]
    fn interlaced_2_wide_3_rows() {
        #[rustfmt::skip]
        let source = [
            0, 0,
            1, 1,
            2, 2,
            0, 0,
            1, 1,
            2, 2,
            0, 0,
            1, 1,
            2, 2,
            0, 0,
            1, 1,
            2, 2,
        ];
        for n in 0..3 {
            let interlaced = Interlaced::<_, 3>::new(source.iter(), n, 2).copied();
            let mut count = 0;
            for (index, val) in interlaced.enumerate() {
                assert_eq!(val, n, "index: {}", index);
                count += 1;
            }
            assert_eq!(count, source.len() / 3);
        }
    }
}
