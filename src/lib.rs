#![no_std]

mod mlx90640;
mod mlx90641;
mod common;
mod error;
mod register;
mod util;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
