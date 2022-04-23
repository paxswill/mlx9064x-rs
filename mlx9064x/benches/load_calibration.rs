use criterion::{criterion_group, criterion_main, Criterion};

use mlx9064x::{mlx90640::Mlx90640Calibration, mlx90641::Mlx90641Calibration};
use mlx9064x_test_data::{mlx90640_datasheet_eeprom, mlx90641_datasheet_eeprom};

pub fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("Calibration Loading");

    group.bench_with_input("MLX90640", &mlx90640_datasheet_eeprom(), |b, eeprom| {
        b.iter(|| Mlx90640Calibration::from_data(eeprom))
    });
    group.bench_with_input("MLX90641", &mlx90641_datasheet_eeprom(), |b, eeprom| {
        b.iter(|| Mlx90641Calibration::from_data(eeprom))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
