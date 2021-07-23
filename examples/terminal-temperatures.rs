use std::env;
use std::error::Error as StdError;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use linux_embedded_hal::I2cdev;
use mlx9064x::Mlx90640Camera;

fn main() -> Result<(), AnyError> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(AnyError::String(
            "Two arguments required: <I2C bus> <camera address>".to_string(),
        ));
    }
    let address: u8 = if args[2].starts_with("0x") {
        let hex_digits = args[2].split_at(2).1;
        u8::from_str_radix(&hex_digits, 16)?
    } else {
        args[2].parse()?
    };
    let bus_path = Path::new(&args[1]);
    let bus = I2cdev::new(bus_path)?;
    let mut camera = Mlx90640Camera::new(bus, address)?;
    let mut temperatures = vec![0f32; camera.height() * camera.width()];
    let delay = Duration::from_millis(500);
    camera.generate_image_if_ready(&mut temperatures)?;
    sleep(delay);
    camera.generate_image_if_ready(&mut temperatures)?;

    let width = 32;
    print_temperatures(&temperatures, width);
    println!();
    Ok(())
}

fn print_temperatures(temperatures: &[f32], width: usize) {
    for (count, temperature) in temperatures.iter().enumerate() {
        if count % width == 0 {
            println!();
        }
        print!("{:4.2}  ", temperature);
    }
}

// It's anyhow::Error, but less functional and less tested.
#[derive(Debug)]
enum AnyError {
    Wrapped(Box<dyn StdError + 'static>),
    String(String),
}

impl std::fmt::Display for AnyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AnyError::Wrapped(err) => write!(f, "{}", err),
            AnyError::String(s) => write!(f, "{}", s),
        }
    }
}

impl<E> From<E> for AnyError
where
    E: StdError + 'static,
{
    fn from(err: E) -> Self {
        AnyError::Wrapped(Box::new(err))
    }
}
