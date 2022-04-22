use std::ops::DerefMut;
use std::sync::{Arc, Mutex};

use criterion::{criterion_group, criterion_main, BatchSize, Criterion};

use mlx9064x::calculations::{
    raw_ir_to_temperatures, raw_pixels_to_ir_data, raw_pixels_to_temperatures,
};
use mlx9064x::common::{read_ram, FromI2C, MelexisCamera};
use mlx9064x::mlx90640::{Mlx90640, Mlx90640Calibration};
use mlx9064x::{AccessPattern, CalibrationData, ControlRegister, StatusRegister, Subpage};
use mlx9064x_test_data::{example_mlx90640_at_address, mlx90640_example_data};

pub fn criterion_benchmark(c: &mut Criterion) {
    let address = 0x33;
    let mut mocked = example_mlx90640_at_address(address);

    let control = ControlRegister::from_i2c(&mut mocked, address).unwrap();
    let calibration = Mlx90640Calibration::from_i2c(&mut mocked, address).unwrap();
    let resolution_correction =
        Mlx90640::resolution_correction(calibration.resolution(), control.resolution());
    let emissivity = calibration.emissivity().unwrap_or(1f32);
    let access_pattern = AccessPattern::Chess;

    let mocked = Arc::new(Mutex::new(mocked));
    let mut group = c.benchmark_group("Pixels to Temperatures");

    let setup = || {
        let mut bus_guard = mocked.lock().unwrap();
        let bus = bus_guard.deref_mut();
        let status = StatusRegister::from_i2c(bus, address).unwrap();
        let subpage = status.last_updated_subpage();
        let mut pixel_bytes = [0u8; Mlx90640::NUM_PIXELS * 2];
        let temperatures = [0f32; Mlx90640::NUM_PIXELS];
        let valid_pixels = Mlx90640::pixels_in_subpage(subpage, access_pattern);

        let ram_data = read_ram::<Mlx90640, _, { Mlx90640::HEIGHT }>(
            bus,
            address,
            access_pattern,
            subpage,
            &mut pixel_bytes,
        )
        .unwrap();
        // Update for the next frame
        match subpage {
            Subpage::Zero => {
                bus.update_frame(
                    mlx90640_example_data::FRAME_1_DATA,
                    mlx90640_example_data::FRAME_1_STATUS_REGISTER,
                );
            }
            Subpage::One => {
                bus.update_frame(
                    mlx90640_example_data::FRAME_0_DATA,
                    mlx90640_example_data::FRAME_0_STATUS_REGISTER,
                );
            }
        };
        bus.set_data_available(true);
        (subpage, pixel_bytes, temperatures, valid_pixels, ram_data)
    };

    group.bench_function("split functions", |b| {
        b.iter_batched(
            setup,
            |(subpage, pixel_bytes, mut temperatures, mut valid_pixels, ram_data)| {
                let ambient_temperature = raw_pixels_to_ir_data(
                    &calibration,
                    emissivity,
                    resolution_correction,
                    &pixel_bytes,
                    ram_data,
                    subpage,
                    access_pattern,
                    &mut valid_pixels.clone(),
                    &mut temperatures,
                );
                raw_ir_to_temperatures(
                    &calibration,
                    emissivity,
                    ambient_temperature,
                    None,
                    subpage,
                    &mut valid_pixels,
                    &mut temperatures,
                );
            },
            BatchSize::SmallInput,
        )
    });

    group.bench_function("one pass", |b| {
        b.iter_batched(
            setup,
            |(subpage, pixel_bytes, mut temperatures, mut valid_pixels, ram_data)| {
                let _ambient_temperature = raw_pixels_to_temperatures(
                    &calibration,
                    emissivity,
                    None,
                    resolution_correction,
                    &pixel_bytes,
                    ram_data,
                    subpage,
                    access_pattern,
                    &mut valid_pixels,
                    &mut temperatures,
                );
            },
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
