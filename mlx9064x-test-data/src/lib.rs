// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod eeprom_data;
mod i2c_mock;
pub mod mlx90640_example_data;

pub use eeprom_data::{mlx90640_datasheet_eeprom, mlx90641_datasheet_eeprom, EEPROM_LENGTH};
pub use i2c_mock::{
    datasheet_mlx90640_at_address, example_mlx90640_at_address, mock_mlx90641_at_address,
    MockCameraBus, MLX90640_RAM_LENGTH, MLX90641_RAM_LENGTH,
};
