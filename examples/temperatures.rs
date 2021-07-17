use std::env;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use linux_embedded_hal::I2cdev;

use mlx9064x::Mlx90640Camera;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        println!("Two arguments required: <I2C bus> <camera address>");
        return;
    }
    let address: u8 = if args[2].starts_with("0x") {
        let hex_digits = args[2].split_at(2).1;
        u8::from_str_radix(&hex_digits, 16)
            .expect("If the address starts with 0x, its a base-16 number")
    } else {
        args[2].parse().expect("The address to be an integer")
    };
    let bus_path = Path::new(&args[1]);
    let bus = I2cdev::new(bus_path).expect("The given path should work as an I2C device");
    let mut camera =
        Mlx90640Camera::new(bus, address).expect("An MLX90640 instance should've been created");
    let mut temperatures = vec![0f32; 768];
    let delay = Duration::from_millis(500);
    camera
        .generate_image_if_ready(&mut temperatures)
        .expect("accessing an image to work");
    sleep(delay);
    camera
        .generate_image_if_ready(&mut temperatures)
        .expect("accessing an image to work");

    let width = 32;
    print_temperatures(&temperatures, width);
    println!();
}

fn print_temperatures(temperatures: &[f32], width: usize) {
    for (count, temperature) in temperatures.iter().enumerate() {
        if count % width == 0 {
            println!();
        }
        print!("{:4.2}  ", temperature);
    }
}
