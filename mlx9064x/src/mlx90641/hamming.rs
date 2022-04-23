// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

//! MLX90641 EEPROM Hamming code
//!
//! The MLX90641 encodes data with a (16, 11) [Hamming code] ((15, 11) with an extra
//! parity bit) and all parity bits moved to the most significant bits.
//!
//! [Hamming code]: https://en.wikipedia.org/wiki/Hamming_code
//!
//! <table align="center">
//!   <caption>Bits covered by each parity bit</caption>
//!   <colgroup>
//!     <col span="1">
//!   </colgroup>
//!   <thead>
//!     <tr>
//!       <th scope="row">Bit</th>
//!       <th scope="col">15</th>
//!       <th scope="col">14</th>
//!       <th scope="col">13</th>
//!       <th scope="col">12</th>
//!       <th scope="col">11</th>
//!       <th scope="col">10</th>
//!       <th scope="col">9</th>
//!       <th scope="col">8</th>
//!       <th scope="col">7</th>
//!       <th scope="col">6</th>
//!       <th scope="col">5</th>
//!       <th scope="col">4</th>
//!       <th scope="col">3</th>
//!       <th scope="col">2</th>
//!       <th scope="col">1</th>
//!       <th scope="col">0</th>
//!     </tr>
//!     <tr>
//!       <th scope="row">Role</th>
//!       <th scope="col">P4</th>
//!       <th scope="col">P3</th>
//!       <th scope="col">P2</th>
//!       <th scope="col">P1</th>
//!       <th scope="col">P0</th>
//!       <th scope="col">D10</th>
//!       <th scope="col">D9</th>
//!       <th scope="col">D8</th>
//!       <th scope="col">D7</th>
//!       <th scope="col">D6</th>
//!       <th scope="col">D5</th>
//!       <th scope="col">D4</th>
//!       <th scope="col">D3</th>
//!       <th scope="col">D2</th>
//!       <th scope="col">D1</th>
//!       <th scope="col">D0</th>
//!     </tr>
//!   </thead>
//!   <tbody>
//!     <tr>
//!       <th scope="row">P0</th>
//!       <td></td><td></td><td></td><td></td>
//!       <td></td><td>x</td><td></td><td>x</td>
//!       <td></td><td>x</td><td></td><td>x</td>
//!       <td>x</td><td></td><td>x</td><td>x</td>
//!     </tr>
//!     <tr>
//!       <th scope="row">P1</th><td></td><td></td><td></td>
//!       <td></td><td></td><td>x</td><td>x</td>
//!       <td></td><td></td><td>x</td><td>x</td>
//!       <td></td><td>x</td><td>x</td><td></td><td>x</td>
//!     </tr>
//!     <tr>
//!       <th scope="row">P2</th><td></td><td></td><td></td><td></td>
//!       <td></td><td>x</td><td>x</td><td>x</td>
//!       <td>x</td><td></td><td></td><td></td>
//!       <td>x</td><td>x</td><td>x</td><td></td>
//!     </tr>
//!     <tr>
//!       <th scope="row">P3</th><td></td><td></td><td></td><td></td>
//!       <td></td><td>x</td><td>x</td><td>x</td>
//!       <td>x</td><td>x</td><td>x</td><td>x</td>
//!       <td></td><td></td><td></td><td></td>
//!     </tr>
//!     <tr>
//!       <th scope="row">P4</th><td></td><td>x</td><td>x</td><td>x</td>
//!       <td>x</td><td>x</td><td>x</td><td>x</td>
//!       <td>x</td><td>x</td><td>x</td><td>x</td>
//!       <td>x</td><td>x</td><td>x</td><td>x</td>
//!     </tr>
//!   </tbody>
//! </table>

use crate::error::LibraryError;
use crate::util::is_bit_set;

const DATA_MASK: u16 = 0x07FF;

const PARITY_BITS: usize = 5;

const PARITY_MASKS: [u16; PARITY_BITS] = [
    0b0000_0101_0101_1011,
    0b0000_0110_0110_1101,
    0b0000_0111_1000_1110,
    0b0000_0111_1111_0000,
    0b0111_1111_1111_1111,
];

/// Compute and add the Hamming code to a word
///
/// A Hamming code covering the lower eleven bits, with the code in the upper five bits is
/// returned. The bits that will be used for the code must be 0 when given, otherwise an error is
/// returned.
pub fn add_checksum(word: u16) -> Result<u16, LibraryError> {
    // TODO: rewrite to be closer to a "standard" Hamming code implementation
    if word & !DATA_MASK != 0 {
        Err(LibraryError::Other(
            "Value given to add_checksum already has a checksum",
        ))
    } else {
        let mut calculated_word = word;
        for (mask_index, parity_mask) in PARITY_MASKS.iter().enumerate() {
            let parity_index = DATA_MASK.count_ones() as usize + mask_index;
            let parity = ((calculated_word & parity_mask).count_ones() % 2) as u16;
            calculated_word |= parity << parity_index;
        }
        Ok(calculated_word)
    }
}

/// Validate the checksum used for the MLX90641 EEPROM
///
/// One-bit errors can be corrected. If uncorrectable errors are found, a [`LibraryError::Checksum]
/// is returned with the failing value.
pub fn validate_checksum(word: u16) -> Result<u16, LibraryError> {
    // TODO: Rewrite to be closer to a standard Hamming code implementation
    let calculated_word = add_checksum(word & DATA_MASK)?;
    if calculated_word != word {
        let given_parity = word & (!DATA_MASK);
        let calculated_parity = calculated_word & (!DATA_MASK);
        const PARITY_OFFSET: u16 = u16::BITS as u16 - PARITY_BITS as u16;
        let affected_parity = (given_parity ^ calculated_parity) >> PARITY_OFFSET;
        let mut error_bits = 0xFFFF;
        for (mask_index, mask) in PARITY_MASKS[..(PARITY_MASKS.len() - 1)].iter().enumerate() {
            // For finding errors, the mask affects its own parity bit
            let mask = mask | (1 << (mask_index as u16 + PARITY_OFFSET));
            if is_bit_set(affected_parity, mask_index) {
                error_bits &= mask;
            } else {
                error_bits &= !mask;
            }
        }
        let corrected_word = word ^ error_bits;
        if add_checksum(corrected_word & DATA_MASK) == Ok(corrected_word) {
            // Flip that bit, and double check that it's good
            Ok(corrected_word & DATA_MASK)
        } else {
            Err(LibraryError::Checksum(word))
        }
    } else {
        Ok(word & DATA_MASK)
    }
}

#[cfg(test)]
mod test {
    use mlx9064x_test_data::{mlx90641_datasheet_eeprom, EEPROM_LENGTH};

    use crate::error::LibraryError;

    // Check against all of the example values
    fn eeprom_values() -> [u16; EEPROM_LENGTH] {
        let mut dest = [0u16; EEPROM_LENGTH];
        let raw = mlx90641_datasheet_eeprom();
        raw.chunks_exact(2)
            .map(|chunk| {
                let bytes: [u8; 2] = chunk.try_into().unwrap();
                u16::from_be_bytes(bytes)
            })
            .zip(dest.iter_mut())
            .for_each(|(source, dest)| *dest = source);
        dest
    }

    #[test]
    fn checksum_no_errors() {
        for value in eeprom_values().iter() {
            let checked = super::validate_checksum(*value);
            assert_eq!(
                checked,
                Ok(*value & super::DATA_MASK),
                "{:#06X} did not pass checksum validation",
                *value
            );
        }
    }

    #[test]
    fn checksum_one_error() {
        for value in eeprom_values().iter() {
            for bit_index in 0..16 {
                let bad_value = *value ^ (1 << bit_index);
                let checked = super::validate_checksum(bad_value);
                // Only errors in the non-checksum bits are correctable
                assert_eq!(
                    checked,
                    Ok(*value & super::DATA_MASK),
                    "Unable to correct {:#06X} to {:#06X}",
                    bad_value,
                    *value
                );
            }
        }
    }

    #[test]
    fn checksum_two_errors() {
        for value in eeprom_values().iter() {
            for bit_index1 in 0..16 {
                for bit_index2 in 0..16 {
                    if bit_index1 == bit_index2 {
                        continue;
                    }
                    let bad_value = *value ^ (1 << bit_index1) ^ (1 << bit_index2);
                    let checked = super::validate_checksum(bad_value);
                    assert_eq!(
                        checked,
                        Err(LibraryError::Checksum(bad_value)),
                        "{:#06X} (originally {:#06X}) passed checksum validation unexpectedly",
                        bad_value,
                        *value,
                    );
                }
            }
        }
    }
}
