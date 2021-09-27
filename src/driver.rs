// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

use core::fmt::Debug;
use core::marker::PhantomData;

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;
use paste::paste;

use crate::calculations::*;
use crate::common::*;
use crate::error::Error;
use crate::register::*;
use crate::util::Num;

/// DRY macro for the set_* methods in `CameraDriver` that modify a register field.
///
/// Most of the fields are boolean values, so that's the default type. Otherwise, add the type in
/// before the docstring.
macro_rules! set_register_field {
    { $register_access:ident, $field:ident, $doc:literal } => {
        set_register_field! {
            $register_access,
            $field,
            bool,
            $doc
        }
    };
    { $register_access:ident, $field:ident, $typ:ty, $doc:literal } => {
    paste! {
        #[doc = $doc]
        pub fn [< set_ $field >](&mut self, new_value: $typ) -> Result<(), Error<I2C>> {
            let mut current = self.$register_access()?;
            if current.$field() != new_value {
                current.[< set_ $field >](new_value);
                self.[< set_ $register_access >](current)
            } else {
                Ok(())
            }
        }
    }};
}

/// The shared camera driver for the MLX90640 and MLX90641 thermopiles.
///
/// These cameras offer higher resolutions and faster refresh rates than other common low-cost
/// thermal cameras, but they come with some differences in operation, as well as much more
/// processing required to end up with a grid of temperatures.
///
/// The biggest impact to users of these modules is that one of the  `generate_image_*` functions
/// will need to be called twice (once for each subpage) before a full image is available.
#[derive(Clone, Debug)]
pub struct CameraDriver<
    Cam,
    Clb,
    I2C,
    F,
    const HEIGHT: usize,
    const WIDTH: usize,
    const NUM_BYTES: usize,
> {
    /// The I²C bus this camera is accessible on.
    bus: I2C,

    /// The I²C address this camera is accessible at.
    address: u8,

    /// The factory calibration data for a specific camera.
    calibration: Clb,

    /// Buffer for reading pixel data off of the camera.
    // I wish I could use const generics for computer parameters :/
    pixel_buffer: [u8; NUM_BYTES],

    /// ADC resolution correction factor.
    resolution_correction: F,

    /// The most recent observed ambient temperature.
    ///
    /// The ambient temperature is calculated during image processing step. Save it for those
    /// applications that want the ambient temperature so a full recalculation isn't necessary.
    ambient_temperature: Option<F>,

    /// The emissivity value to use when calculating pixel temperature.
    emissivity: F,

    /// The current access pattern the camera is using.
    access_pattern: AccessPattern,

    _camera: PhantomData<Cam>,
}

impl<'a, Cam, Clb, I2C, F, const HEIGHT: usize, const WIDTH: usize, const BUFFER_SIZE: usize>
    CameraDriver<Cam, Clb, I2C, F, HEIGHT, WIDTH, BUFFER_SIZE>
where
    Cam: MelexisCamera<F>,
    Clb: CalibrationData<'a, F>,
    F: 'a + Debug + Num,
    I2C: i2c::WriteRead + i2c::Write,
{
    /// Create a new `CameraDriver`, obtaining the calibration data from the camera over I²C.
    pub fn new(bus: I2C, address: u8) -> Result<Self, Error<I2C>>
    where
        Clb: FromI2C<I2C, Ok = Clb, Error = Error<I2C>>,
    {
        let mut bus = bus;
        let calibration = Clb::from_i2c(&mut bus, address)?;
        Self::new_with_calibration(bus, address, calibration)
    }

    /// Create a `CameraDriver` for accessing the camera at the given I²C address.
    ///
    /// MLX90964\*s can be configured to use any I²C address (except 0x00), but the default address
    /// is 0x33.
    pub fn new_with_calibration(
        bus: I2C,
        address: u8,
        calibration: Clb,
    ) -> Result<Self, Error<I2C>> {
        // We own the bus now, make it mutable.
        let mut bus = bus;
        // Grab the control register values first
        // Need to map from I2C::Error manually as it's an associated type without bounds, so we
        // can't implement From<I2C:Error>
        let control: ControlRegister = read_register(&mut bus, address)?;
        // Cache these values
        let resolution_correction = Cam::resolution_correction(
            calibration.resolution(),
            control.resolution().as_raw() as u8,
        );
        let access_pattern = control.access_pattern();
        // Choose an emissivity value to start with.
        let emissivity = calibration.emissivity().unwrap_or(F::ONE);
        Ok(Self {
            bus,
            address,
            calibration,
            pixel_buffer: [0u8; BUFFER_SIZE],
            resolution_correction,
            ambient_temperature: None,
            emissivity,
            access_pattern,
            _camera: PhantomData,
        })
    }

    fn status_register(&mut self) -> Result<StatusRegister, Error<I2C>> {
        let register = read_register(&mut self.bus, self.address)?;
        Ok(register)
    }

    fn set_status_register(&mut self, register: StatusRegister) -> Result<(), Error<I2C>> {
        write_register(&mut self.bus, self.address, register)
    }

    fn update_control_register(&mut self, register: &ControlRegister) {
        // Update the resolution as well
        let calibrated_resolution = self.calibration.resolution();
        self.resolution_correction =
            Cam::resolution_correction(calibrated_resolution, register.resolution().as_raw() as u8);
        self.access_pattern = register.access_pattern();
    }

    fn control_register(&mut self) -> Result<ControlRegister, Error<I2C>> {
        let register: ControlRegister = read_register(&mut self.bus, self.address)?;
        // Update the resolution as well
        self.update_control_register(&register);
        Ok(register)
    }

    fn set_control_register(&mut self, register: ControlRegister) -> Result<(), Error<I2C>> {
        self.update_control_register(&register);
        write_register(&mut self.bus, self.address, register)?;
        Ok(())
    }

    /// Get the last measured subpage.
    pub fn last_measured_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.status_register()?.last_updated_subpage())
    }

    /// Check if there is new data available, and if so, which subpage.
    pub fn data_available(&mut self) -> Result<Option<Subpage>, Error<I2C>> {
        let register = self.status_register()?;
        Ok(if register.new_data() {
            Some(register.last_updated_subpage())
        } else {
            None
        })
    }

    /// Clear the data available flag, signaling to the camera that the controller is ready for
    /// more data.
    ///
    /// This flag can only be reset by the controller.
    pub fn reset_data_available(&mut self) -> Result<(), Error<I2C>> {
        let mut current = self.status_register()?;
        current.reset_new_data();
        self.set_status_register(current)
    }

    /// Check if the overwrite enabled flag is set.
    ///
    /// This flag is only effective when `data_hold_enabled` is active.
    pub fn overwrite_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.status_register()?.overwrite_enabled())
    }

    set_register_field! {
        status_register,
        overwrite_enabled,
        "Enabled (or disable) overwriting of data in RAM with new data."
    }

    /// Check if the camera is using subpages.
    ///
    /// When disabled, only one page will be measured. The default is to use subpages.
    pub fn subpages_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.use_subpages())
    }

    set_register_field! {
        control_register,
        use_subpages,
        "Enabled (or disable) the use of subpages."
    }

    /// Check if the "Enable data hold" flag is set.
    ///
    /// When this flag (bit 2 on 0x800D) is set, data is not copied to RAM unless the
    /// `enable_overwrite` flag is set. The default is for this mode to be disabled.
    pub fn data_hold_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.data_hold())
    }

    set_register_field! {
        control_register,
        data_hold,
        "Enabled (or disable) data holding."
    }

    /// Check if the camera is in subpage repeat mode.
    ///
    /// This flag only has an effect if [subpages][CameraDriver::subpages_enabled] is enabled. In
    /// subpage repeat mode, only the subpage set in `selected_subpage` will be measured and
    /// updated. When disabled, the active subpage will alternate between the two. The default is
    /// disabled.
    pub fn subpage_repeat(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.subpage_repeat())
    }

    set_register_field! {
        control_register,
        subpage_repeat,
        "Enabled (or disable) subpage repeat mode."
    }

    /// Get the currently selected subpage when [subpage repeat] is enabled.
    ///
    /// This setting only has an effect when `subpage_repeat` is enabled. The default value is
    /// `Subpage::Zero`.
    ///
    /// [subpage repeat]: CameraDriver::subpage_repeat
    pub fn selected_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.control_register()?.subpage())
    }

    set_register_field! {
        control_register,
        subpage,
        Subpage,
        "Set the currently selected subpage when [subpage repeat][CameraDriver::subpage_repeat] is enabled."
    }

    /// Read the frame rate from the camera.
    ///
    /// The default frame rate is [2 FPS][FrameRate::Two].
    pub fn frame_rate(&mut self) -> Result<FrameRate, Error<I2C>> {
        Ok(self.control_register()?.frame_rate())
    }

    set_register_field! {
        control_register,
        frame_rate,
        FrameRate,
        "Set camera's frame rate."
    }
    // TODO: Add a special-use function for setting the frame rate in EEPROM.

    /// Get the current resolution of the ADC in the camera.
    ///
    /// The default resolution is [18 bits][Resolution::Eighteen].
    pub fn resolution(&mut self) -> Result<Resolution, Error<I2C>> {
        Ok(self.control_register()?.resolution())
    }

    set_register_field! {
        control_register,
        resolution,
        Resolution,
        "Set ADC resolution within the camera."
    }

    /// Get the current access pattern used by the camera when updating subpages.
    ///
    /// The default for the MLX90640 is the chess patterm while the default for the MLX90641 is the
    /// interleaved pattern.
    pub fn access_pattern(&mut self) -> Result<AccessPattern, Error<I2C>> {
        Ok(self.control_register()?.access_pattern())
    }

    set_register_field! {
        control_register,
        access_pattern,
        AccessPattern,
        "Set the access pattern used by the camera."
    }

    /// Get the emissivity value that is being used for calculations currently.
    ///
    /// The default emissivity is 1, unless a camera has a different value stored in EEPROM, in
    /// which case that value is used. The default can also be
    /// [overridden][CameraDriver::override_emissivity], but this change is not stored on the camera.
    pub fn effective_emissivity(&self) -> F {
        self.emissivity
    }

    /// Override the emissivity value used in temperature calculations.
    ///
    /// The default emissivity is 1, unless a camera has a different value stored in EEPROM, in
    /// which case that value is used. This method allows a new value to be used to compensate for
    /// emissivity.
    pub fn override_emissivity(&mut self, new_value: F) {
        self.emissivity = new_value;
    }

    /// Use the default emissivity value.
    ///
    /// This is the opposite to `override_emissivity`, as it uses the default emissivity from
    /// either the camera (if the camera has a value set) or 1.
    pub fn use_default_emissivity(&mut self) {
        let default_emissivity = self.calibration.emissivity();
        self.emissivity = default_emissivity.unwrap_or(F::ONE);
    }

    /// Get the most recent ambient temperature calculation.
    ///
    /// The ambient temperature is calculated as part of the overall image calculations. If that
    /// process hasn't been performed yet (by calling `generate_image_if_ready` or similar), this
    /// method will return `None`.
    pub fn ambient_temperature(&self) -> Option<F> {
        self.ambient_temperature
    }

    /// The height of the thermal image, in pixels.
    pub fn height(&self) -> usize {
        // const generics make this silly.
        HEIGHT
    }

    /// The width of the thermal image, in pixels.
    pub fn width(&self) -> usize {
        WIDTH
    }

    fn read_ram(&mut self, subpage: Subpage) -> Result<RamData, Error<I2C>> {
        read_ram::<Cam, I2C, F, HEIGHT>(
            &mut self.bus,
            self.address,
            self.access_pattern,
            subpage,
            &mut self.pixel_buffer,
        )
    }

    pub fn generate_raw_image_subpage_to(
        &'a mut self,
        subpage: Subpage,
        destination: &mut [F],
    ) -> Result<(), Error<I2C>> {
        let ram = self.read_ram(subpage)?;
        let mut valid_pixels = Cam::pixels_in_subpage(subpage, self.access_pattern).into_iter();
        let t_a = raw_pixels_to_ir_data(
            &self.calibration,
            self.emissivity,
            self.resolution_correction,
            &self.pixel_buffer,
            ram,
            subpage,
            &mut valid_pixels,
            destination,
        );
        self.ambient_temperature = Some(t_a);
        Ok(())
    }

    pub fn generate_image_subpage_to(
        &'a mut self,
        subpage: Subpage,
        destination: &mut [F],
    ) -> Result<(), Error<I2C>> {
        let ram = self.read_ram(subpage)?;
        let mut valid_pixels = Cam::pixels_in_subpage(subpage, self.access_pattern).into_iter();
        let t_a = raw_pixels_to_temperatures(
            &self.calibration,
            self.emissivity,
            self.resolution_correction,
            &self.pixel_buffer,
            ram,
            subpage,
            &mut valid_pixels,
            destination,
        );
        self.ambient_temperature = Some(t_a);
        Ok(())
    }

    /// Generate a thermal "image" from the camera's current data.
    ///
    /// This function does *not* check if there is new data, it just copies the
    pub fn generate_image_to<'b: 'a>(
        &'b mut self,
        destination: &mut [F],
    ) -> Result<(), Error<I2C>> {
        let subpage = self.last_measured_subpage()?;
        self.generate_image_subpage_to(subpage, destination)
    }

    /// Generate a thermal "image" from the camera's current data, if there's new data.
    ///
    /// This function first checks to see if there is new data available, and if there is it copies
    /// that data into the provided buffer. It will then clear the data ready flag afterwards,
    /// signaliing to the camera that we are ready for more data. The `Ok` value is a boolean for
    /// whether or not data was ready and copied.
    pub fn generate_image_if_ready(
        &'a mut self,
        destination: &mut [F],
    ) -> Result<bool, Error<I2C>> {
        // Not going through the helper methods on self to avoid infecting them with 'a
        let address = self.address;
        let bus = &mut self.bus;
        let pixel_buffer = &mut self.pixel_buffer;
        let mut status_register: StatusRegister = read_register(bus, address)?;
        if status_register.new_data() {
            let subpage = status_register.last_updated_subpage();
            let mut valid_pixels = Cam::pixels_in_subpage(subpage, self.access_pattern).into_iter();
            let ram = read_ram::<Cam, I2C, F, HEIGHT>(
                bus,
                address,
                self.access_pattern,
                subpage,
                pixel_buffer,
            )?;
            let ambient_temperature = raw_pixels_to_temperatures(
                &self.calibration,
                self.emissivity,
                self.resolution_correction,
                &self.pixel_buffer,
                ram,
                subpage,
                &mut valid_pixels,
                destination,
            );
            self.ambient_temperature = Some(ambient_temperature);
            status_register.reset_new_data();
            write_register(bus, address, status_register)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Synchronize with the camera's frame update timing
    ///
    /// This function ignores any new data, then forces a new measurement by the camera, only
    /// returning when that measurement is complete. This can be used to synchronize frame access
    /// form the controller to the update time of the camera.
    pub fn synchronize(&mut self) -> Result<(), Error<I2C>> {
        let mut status_register = self.status_register()?;
        status_register.reset_new_data();
        status_register.set_overwrite_enabled(true);
        status_register.set_start_measurement();
        write_register(&mut self.bus, self.address, status_register)?;
        // Spin while we wait for data
        while !status_register.new_data() {
            status_register = read_register(&mut self.bus, self.address)?;
            core::hint::spin_loop();
        }
        Ok(())
    }
}

fn read_ram<Cam, I2C, F, const HEIGHT: usize>(
    bus: &mut I2C,
    i2c_address: u8,
    access_pattern: AccessPattern,
    subpage: Subpage,
    pixel_data_buffer: &mut [u8],
) -> Result<RamData, Error<I2C>>
where
    Cam: MelexisCamera<F>,
    I2C: i2c::WriteRead + i2c::Write,
{
    // Pick a maximum size of HEIGHT, as the worst access pattern is still by rows
    let pixel_ranges: ArrayVec<PixelAddressRange, HEIGHT> =
        Cam::pixel_ranges(subpage, access_pattern)
            .into_iter()
            .collect();
    for range in pixel_ranges.iter() {
        let offset = range.buffer_offset;
        let address_bytes = range.start_address.as_bytes();
        bus.write_read(
            i2c_address,
            &address_bytes[..],
            &mut pixel_data_buffer[offset..(offset + range.length)],
        )
        .map_err(Error::I2cWriteReadError)?;
    }
    // And now to read the non-pixel information out
    RamData::from_i2c::<I2C, Cam, F>(bus, i2c_address, subpage).map_err(Error::I2cWriteReadError)
}

fn read_register<R, I2C>(bus: &mut I2C, address: u8) -> Result<R, Error<I2C>>
where
    I2C: i2c::WriteRead + i2c::Write,
    R: Register,
{
    // Inner function to reduce the impact of monomorphization for Register. It'll still get
    // duplicated, but it should just be duplicated on I2C, and there should only be one of those
    // in an application (usually).
    fn read_register<I2C: i2c::WriteRead>(
        bus: &mut I2C,
        i2c_address: u8,
        register_address: Address,
    ) -> Result<[u8; 2], I2C::Error> {
        let register_address_bytes = register_address.as_bytes();
        let mut register_bytes = [0u8; 2];
        bus.write_read(i2c_address, &register_address_bytes, &mut register_bytes)?;
        Ok(register_bytes)
    }

    let register_address = R::address();
    let register_value =
        read_register(bus, address, register_address).map_err(Error::I2cWriteReadError)?;
    let register = R::from(&register_value[..]);
    Ok(register)
}

fn write_register<R, I2C>(bus: &mut I2C, address: u8, register: R) -> Result<(), Error<I2C>>
where
    I2C: i2c::Write + i2c::WriteRead,
    R: Register,
{
    let register_address = R::address();
    let register_bytes: [u8; 2] = register.into();
    write_raw_register(bus, address, register_address.as_bytes(), register_bytes)
        .map_err(Error::I2cWriteError)?;
    Ok(())
}

fn write_raw_register<I2C: i2c::Write>(
    bus: &mut I2C,
    i2c_address: u8,
    register_address: [u8; 2],
    register_data: [u8; 2],
) -> Result<(), I2C::Error> {
    let combined: [u8; 4] = [
        register_address[0],
        register_address[1],
        register_data[0],
        register_data[1],
    ];
    bus.write(i2c_address, &combined)?;
    Ok(())
}

#[cfg(test)]
mod test {
    extern crate std;

    use core::fmt::Debug;

    use float_cmp::{approx_eq, assert_approx_eq};
    use num_traits::NumCast;

    use crate::test::*;
    use crate::util::Num;
    use crate::{mlx90640, mlx90641};
    use crate::{I2cRegister, Mlx90640Driver, Mlx90641Driver, StatusRegister};

    fn create_mlx90640<F>() -> Mlx90640Driver<MockCameraBus<MLX90640_RAM_LENGTH>, F>
    where
        F: Debug + Num,
    {
        // Specifically using a non-default address to make sure assumptions aren't being made
        // about the address.
        let address: u8 = 0x30;
        let mock_bus = datasheet_mlx90640_at_address(address);
        Mlx90640Driver::new(mock_bus, address)
            .expect("A MLX90640 camera should be created after loading its data")
    }

    fn create_mlx90641<F>() -> Mlx90641Driver<MockCameraBus<MLX90641_RAM_LENGTH>, F>
    where
        F: Debug + Num,
    {
        // Again, non default address, but different from the '640 mock as well
        let address: u8 = 0x28;
        let mock_bus = mock_mlx90641_at_address(address);
        Mlx90641Driver::new(mock_bus, address)
            .expect("A MLX90641 camera should be created after loading its data")
    }

    #[test]
    fn smoke_test() {
        create_mlx90640::<f32>();
        create_mlx90640::<f64>();
        create_mlx90641::<f32>();
        create_mlx90641::<f64>();
        // Test passes if we get this far.
    }

    #[test]
    fn read_register() {
        // Just picking addresses now
        let address = 0x10;
        let mut mock_bus = datasheet_mlx90640_at_address(address);
        mock_bus.clear_recent_operations();
        let register: I2cRegister = super::read_register(&mut mock_bus, address).unwrap();
        assert_eq!(register, I2cRegister::default());
        let ops = mock_bus.recent_operations();
        assert_eq!(
            ops.len(),
            1,
            "Only one operation should be performed to read a register"
        )
    }

    #[test]
    fn read_write_register() {
        let address = 0x42;
        // Using the status register for this test as it has read-only sections at both ends.
        let mut mock_bus = datasheet_mlx90640_at_address(address);
        mock_bus.clear_recent_operations();
        let mut status_register: StatusRegister =
            super::read_register(&mut mock_bus, address).unwrap();
        assert_eq!(mock_bus.recent_operations().len(), 1);
        assert!(!status_register.overwrite_enabled());
        status_register.set_overwrite_enabled(true);
        super::write_register(&mut mock_bus, address, status_register).unwrap();
        assert_eq!(mock_bus.recent_operations().len(), 2);
    }

    #[test]
    fn default_emissivity() {
        // The MLX90640 doesn't store emissivity in EEPROM, so it should *always* default to 1
        let mut cam = create_mlx90640();
        assert_eq!(cam.effective_emissivity(), 1.0);
        // When we override it, it should change. Let's use the value for limestone from Wikipedia.
        cam.override_emissivity(0.92);
        assert_eq!(cam.effective_emissivity(), 0.92);
        // And we can reset it back to 0
        cam.use_default_emissivity();
        assert_eq!(cam.effective_emissivity(), 1.0);
    }

    #[test]
    fn mlx90640_datasheet_integration() {
        let mut cam = create_mlx90640();
        let mut temperatures = [0.0; mlx90640::NUM_PIXELS];
        let res = cam.generate_image_if_ready(&mut temperatures);
        assert!(res.is_ok());
        assert!(res.unwrap());
        // Test pixel is (12, 16)
        const PIXEL_INDEX: usize = 11 * mlx90640::WIDTH + 15;
        assert_approx_eq!(f32, temperatures[PIXEL_INDEX], 80.36331, epsilon = 0.0001);
    }

    #[test]
    fn mlx90641_datasheet_integration() {
        let mut cam = create_mlx90641();
        let mut temperatures = [0.0; mlx90641::NUM_PIXELS];
        let res = cam.generate_image_if_ready(&mut temperatures);
        assert!(res.is_ok());
        assert!(res.unwrap());
        // Test pixel is (6, 9)
        const PIXEL_INDEX: usize = 5 * mlx90641::WIDTH + 8;
        assert_approx_eq!(f32, temperatures[PIXEL_INDEX], 80.129812, epsilon = 0.0001);
    }

    #[test]
    fn mlx90640_example_integration() {
        let i2c_address = 0x43;
        let mut mocked = example_mlx90640_at_address(i2c_address);
        mocked.set_data_available(false);
        let mut cam = Mlx90640Driver::new(mocked.clone(), i2c_address).unwrap();
        let mut temperatures = [f32::NAN; mlx90640::NUM_PIXELS];
        // Make sure that nothing happens if the camera doesn't have data ready
        let not_ready = cam.generate_image_if_ready(&mut temperatures);
        assert!(!not_ready.unwrap());
        for temperature in temperatures.iter() {
            assert!(temperature.is_nan());
        }
        mocked.set_data_available(true);
        let ready = cam.generate_image_if_ready(&mut temperatures);
        assert!(ready.unwrap());
        // Set the next frame of data
        mocked.update_frame(
            &mlx90640_example_data::FRAME_1_DATA[..],
            &mlx90640_example_data::FRAME_1_STATUS_REGISTER[..],
        );
        mocked.set_data_available(true);
        assert!(cam.generate_image_if_ready(&mut temperatures).unwrap());
        // Check the results
        let expected_pixels = mlx90640_example_data::TEMPERATURES
            .iter()
            .map(|n| <f32 as NumCast>::from(*n).unwrap());
        let paired = temperatures.iter().zip(expected_pixels);
        for (index, (actual, expected)) in paired.enumerate() {
            assert!(
                // The datasheet only goes out to three decimal places for the pixel temperatures.
                approx_eq!(f32, *actual, expected, epsilon = 0.001),
                "Pixel {}: Expected: {}, Actual: {}",
                index,
                expected,
                *actual
            );
        }
    }

    fn create_sentinel_buffer() -> [u8; mlx90641::NUM_PIXELS * 2] {
        let mut buf = [0u8; mlx90641::NUM_PIXELS * 2];
        // Initialize to 0xDEADBEEF to mark untouched memory
        buf.chunks_exact_mut(4).for_each(|chunk| {
            chunk[0] = 0xDE;
            chunk[1] = 0xAD;
            chunk[2] = 0xBE;
            chunk[3] = 0xEF;
        });
        buf
    }

    fn check_sentinel_buffer(buf: &[u8]) {
        assert!(buf.len() % 4 == 0);
        // If there's a block of two bytes that haven't been updated, fail
        for (index, chunk) in buf.chunks_exact(4).enumerate() {
            assert_ne!(
                chunk[0..2],
                [0xDE, 0xAD],
                "Failed at byte index {}",
                index * 4
            );
            assert_ne!(
                chunk[2..4],
                [0xBE, 0xEF],
                "Failed at byte index {}",
                index * 4 + 2
            );
        }
    }

    #[test]
    fn mlx90641_ram_access_subpage_0() {
        let mut buf = create_sentinel_buffer();
        let i2c_address = 0x47;
        let mut mock_bus = mock_mlx90641_at_address(i2c_address);
        let ram_data_result = super::read_ram::<mlx90641::Mlx90641, _, f32, { mlx90641::HEIGHT }>(
            &mut mock_bus,
            i2c_address,
            crate::AccessPattern::Interleave,
            crate::Subpage::Zero,
            &mut buf,
        );
        assert!(ram_data_result.is_ok());
        check_sentinel_buffer(&buf);
    }

    #[test]
    fn mlx90641_ram_access_subpage_1() {
        let mut buf = create_sentinel_buffer();
        let i2c_address = 0x49;
        let mut mock_bus = mock_mlx90641_at_address(i2c_address);
        let ram_data_result = super::read_ram::<mlx90641::Mlx90641, _, f32, { mlx90641::HEIGHT }>(
            &mut mock_bus,
            i2c_address,
            crate::AccessPattern::Interleave,
            crate::Subpage::One,
            &mut buf,
        );
        assert!(ram_data_result.is_ok());
        check_sentinel_buffer(&buf);
    }

    #[test]
    fn get_register_flag_minimal_operations() {
        let i2c_address = 0x49;
        let mut mocked = example_mlx90640_at_address(i2c_address);
        mocked.set_data_available(false);
        let mut cam = Mlx90640Driver::<_, f32>::new(mocked.clone(), i2c_address).unwrap();
        mocked.clear_recent_operations();
        cam.frame_rate().unwrap();
        let ops = mocked.recent_operations();
        assert_eq!(
            ops.len(),
            1,
            "There should only be one operation to check a register"
        );
    }

    #[test]
    fn set_register_flag_minimal_operations() {
        let i2c_address = 0x49;
        let mut mocked = example_mlx90640_at_address(i2c_address);
        mocked.set_data_available(false);
        let mut cam = Mlx90640Driver::<_, f32>::new(mocked.clone(), i2c_address).unwrap();
        mocked.clear_recent_operations();
        cam.set_frame_rate(crate::FrameRate::SixtyFour).unwrap();
        let ops = mocked.recent_operations();
        assert_eq!(
            ops.len(),
            2,
            "There should only be two operations to update a register"
        );
    }
}
