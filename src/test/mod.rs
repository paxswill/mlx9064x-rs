// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross
mod eeprom_data;
mod i2c_mock;

pub(crate) use eeprom_data::{mlx90640_eeprom_data, mlx90641_eeprom_data, EEPROM_LENGTH};
pub(crate) use i2c_mock::{
    mock_mlx90640_at_address, mock_mlx90641_at_address, MockCameraBus, MLX90640_RAM_LENGTH,
    MLX90641_RAM_LENGTH,
};
