use std::env;
use std::error::Error as StdError;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use linux_embedded_hal::I2cdev;
use mlx9064x::{Mlx90640Driver, Mlx90641Driver};

fn main() -> Result<(), AnyError> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        return Err(AnyError::String(
            "Three arguments required: [640|641] <I2C bus> <camera address>".to_string(),
        ));
    }
    let address: u8 = if args[3].starts_with("0x") {
        let hex_digits = args[3].split_at(2).1;
        u8::from_str_radix(&hex_digits, 16)?
    } else {
        args[3].parse()?
    };
    let bus_path = Path::new(&args[2]);
    let bus = I2cdev::new(bus_path)?;
    let (temperatures, width) = match args[1].as_ref() {
        "640" => {
            let mut camera = Mlx90640Driver::new(bus, address)?;
            let mut temperatures = vec![0f32; camera.height() * camera.width()];
            let delay = Duration::from_millis(500);
            camera.generate_image_if_ready(&mut temperatures)?;
            sleep(delay);
            camera.generate_image_if_ready(&mut temperatures)?;
            (temperatures, camera.width())
        }
        "641" => {
            let mut camera = Mlx90641Driver::new(bus, address)?;
            let mut temperatures = vec![0f32; camera.height() * camera.width()];
            let delay = Duration::from_millis(500);
            camera.generate_image_if_ready(&mut temperatures)?;
            sleep(delay);
            camera.generate_image_if_ready(&mut temperatures)?;
            (temperatures, camera.width())
        }
        _ => {
            return Err(AnyError::String("The second argument must be either 640 or 641".to_string()));
        }
    };
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
