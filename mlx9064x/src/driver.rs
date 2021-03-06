// SPDX-License-Identifier: Apache-2.0
// Copyright © 2021 Will Ross

use embedded_hal::blocking::i2c;
use paste::paste;

use crate::calculations::*;
use crate::common::*;
use crate::error::Error;
use crate::register::*;

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

// HEIGHT and NUM_BYTES are const generics until generic_const_expr is stabilized (maybe). After
// that, point the associated constants on Clb::Camera (Clb::Camera::HEIGHT and {
// Clb::Camera::HEIGHT * Clb::Camera::WIDTH * 2 } ) can be used instead.
#[derive(Clone, Debug)]
pub struct CameraDriver<Clb, I2C, const HEIGHT: usize, const NUM_BYTES: usize> {
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
    resolution_correction: f32,

    /// The most recent observed ambient temperature.
    ///
    /// The ambient temperature is calculated during image processing step. Save it for those
    /// applications that want the ambient temperature so a full recalculation isn't necessary.
    ambient_temperature: Option<f32>,

    /// The emissivity value to use when calculating pixel temperature.
    emissivity: f32,

    /// The current access pattern the camera is using.
    access_pattern: AccessPattern,

    /// The temperature of the ambient environment.
    ///
    /// This value is used as the "reflected temperature" for converting the observed IR data to
    /// actual temperatures.
    reflected_temperature: Option<f32>,
}

impl<'a, Clb, I2C, const HEIGHT: usize, const BUFFER_SIZE: usize>
    CameraDriver<Clb, I2C, HEIGHT, BUFFER_SIZE>
where
    Clb: CalibrationData<'a>,
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
    /// MLX9064\*s can be configured to use any I²C address (except 0x00), but the default address
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
        let control = ControlRegister::from_i2c(&mut bus, address)?;
        // Cache these values
        let resolution_correction =
            Clb::Camera::resolution_correction(calibration.resolution(), control.resolution());
        let access_pattern = control.access_pattern();
        // Choose an emissivity value to start with.
        let emissivity = calibration.emissivity().unwrap_or(1f32);
        Ok(Self {
            bus,
            address,
            calibration,
            pixel_buffer: [0u8; BUFFER_SIZE],
            resolution_correction,
            ambient_temperature: None,
            emissivity,
            access_pattern,
            reflected_temperature: None,
        })
    }

    fn status_register(&mut self) -> Result<StatusRegister, Error<I2C>> {
        let register = StatusRegister::from_i2c(&mut self.bus, self.address)?;
        Ok(register)
    }

    fn set_status_register(&mut self, register: StatusRegister) -> Result<(), Error<I2C>> {
        register.to_i2c(&mut self.bus, self.address)
    }

    fn update_control_register(&mut self, register: &ControlRegister) {
        // Update the resolution as well
        let calibrated_resolution = self.calibration.resolution();
        self.resolution_correction =
            Clb::Camera::resolution_correction(calibrated_resolution, register.resolution());
        self.access_pattern = register.access_pattern();
    }

    fn control_register(&mut self) -> Result<ControlRegister, Error<I2C>> {
        let register = ControlRegister::from_i2c(&mut self.bus, self.address)?;
        // Update the resolution as well
        self.update_control_register(&register);
        Ok(register)
    }

    fn set_control_register(&mut self, register: ControlRegister) -> Result<(), Error<I2C>> {
        self.update_control_register(&register);
        register.to_i2c(&mut self.bus, self.address)?;
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
    pub fn effective_emissivity(&self) -> f32 {
        self.emissivity
    }

    /// Override the emissivity value used in temperature calculations.
    ///
    /// The default emissivity is 1, unless a camera has a different value stored in EEPROM, in
    /// which case that value is used. This method allows a new value to be used to compensate for
    /// emissivity.
    pub fn override_emissivity(&mut self, new_value: f32) {
        self.emissivity = new_value;
    }

    /// Use the default emissivity value.
    ///
    /// This is the opposite to `override_emissivity`, as it uses the default emissivity from
    /// either the camera (if the camera has a value set) or 1.
    pub fn use_default_emissivity(&mut self) {
        let default_emissivity = self.calibration.emissivity();
        self.emissivity = default_emissivity.unwrap_or(1f32);
    }

    /// Retrieve the current reflected temperature value.
    ///
    /// When the temperature of the ambient environment is not known, this function will return
    /// `None`. See [`set_reflected_temperature`][Self::set_reflected_temperature] for more
    /// information.
    pub fn reflected_temperature(&self) -> Option<f32> {
        self.reflected_temperature
    }

    /// Set the reflected temperature value.
    ///
    /// This value is used to compensate for infrared radiation not being emitted by an object
    /// itself, but being emitted by the ambient environment and reflected by an object being
    /// measured. This value is distinct from the one from [`ambient_temperature`], but if not
    /// explicitly known it can be estimated from that value.
    pub fn set_reflected_temperature(&mut self, new_value: Option<f32>) {
        self.reflected_temperature = new_value;
    }

    /// Get the most recent ambient temperature calculation.
    ///
    /// What the datasheets (and this crate) refer to as "ambient temperature" should be better
    /// understood as the ambient temperature of the camera itself, not of the area being imaged.
    /// These values will usually be different because the camera generates some heat itself, and
    /// the sensor used for this value is within the camera module. See
    /// [`MelexisCamera::SELF_HEATING`] for more details.
    ///
    /// This value is calculated as part of the overall image calculations. If that
    /// process hasn't been performed yet (by calling
    /// [`generate_image_if_ready`][Self::generate_image_if_ready] or similar), this method will
    /// return `None`.
    pub fn ambient_temperature(&self) -> Option<f32> {
        self.ambient_temperature
    }

    /// The height of the thermal image, in pixels.
    pub fn height(&self) -> usize {
        // const generics make this silly.
        Clb::Camera::HEIGHT
    }

    /// The width of the thermal image, in pixels.
    pub fn width(&self) -> usize {
        Clb::Camera::WIDTH
    }

    fn read_ram(&mut self, subpage: Subpage) -> Result<RamData, Error<I2C>> {
        read_ram::<Clb::Camera, I2C, HEIGHT>(
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
        destination: &mut [f32],
    ) -> Result<(), Error<I2C>> {
        let ram = self.read_ram(subpage)?;
        let mut valid_pixels =
            Clb::Camera::pixels_in_subpage(subpage, self.access_pattern).into_iter();
        let t_a = raw_pixels_to_ir_data(
            &self.calibration,
            self.emissivity,
            self.resolution_correction,
            &self.pixel_buffer,
            ram,
            subpage,
            self.access_pattern,
            &mut valid_pixels,
            destination,
        );
        self.ambient_temperature = Some(t_a);
        Ok(())
    }

    pub fn generate_image_subpage_to(
        &'a mut self,
        subpage: Subpage,
        destination: &mut [f32],
    ) -> Result<(), Error<I2C>> {
        let ram = self.read_ram(subpage)?;
        let mut valid_pixels =
            Clb::Camera::pixels_in_subpage(subpage, self.access_pattern).into_iter();
        let t_a = raw_pixels_to_temperatures(
            &self.calibration,
            self.emissivity,
            self.reflected_temperature,
            self.resolution_correction,
            &self.pixel_buffer,
            ram,
            subpage,
            self.access_pattern,
            &mut valid_pixels,
            destination,
        );
        self.ambient_temperature = Some(t_a);
        Ok(())
    }

    /// Generate a thermal "image" from the camera's current data.
    ///
    /// This function does *not* check if there is new data, it just copies the current frame of
    /// data.
    pub fn generate_image_to<'b: 'a>(
        &'b mut self,
        destination: &mut [f32],
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
        destination: &mut [f32],
    ) -> Result<bool, Error<I2C>> {
        // Not going through the helper methods on self to avoid infecting them with 'a
        let address = self.address;
        let bus = &mut self.bus;
        let pixel_buffer = &mut self.pixel_buffer;
        let mut status_register = StatusRegister::from_i2c(bus, address)?;
        if status_register.new_data() {
            let subpage = status_register.last_updated_subpage();
            let mut valid_pixels =
                Clb::Camera::pixels_in_subpage(subpage, self.access_pattern).into_iter();
            let ram = read_ram::<Clb::Camera, I2C, HEIGHT>(
                bus,
                address,
                self.access_pattern,
                subpage,
                pixel_buffer,
            )?;
            let ambient_temperature = raw_pixels_to_temperatures(
                &self.calibration,
                self.emissivity,
                self.reflected_temperature,
                self.resolution_correction,
                &self.pixel_buffer,
                ram,
                subpage,
                self.access_pattern,
                &mut valid_pixels,
                destination,
            );
            self.ambient_temperature = Some(ambient_temperature);
            status_register.reset_new_data();
            status_register.to_i2c(bus, address)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Synchronize with the camera's frame update timing
    ///
    /// This function ignores any new data, then forces a new measurement by the camera, only
    /// returning when that measurement is complete. This can be used to synchronize frame access
    /// from the controller to the update time of the camera.
    pub fn synchronize(&mut self) -> Result<(), Error<I2C>> {
        let mut status_register = self.status_register()?;
        status_register.reset_new_data();
        status_register.set_overwrite_enabled(true);
        status_register.set_start_measurement();
        status_register.to_i2c(&mut self.bus, self.address)?;
        // Spin while we wait for data
        while !status_register.new_data() {
            status_register = StatusRegister::from_i2c(&mut self.bus, self.address)?;
            core::hint::spin_loop();
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::excessive_precision)]
mod test {
    extern crate std;

    use float_cmp::{approx_eq, assert_approx_eq};

    use mlx9064x_test_data::*;

    use crate::common::{FromI2C, ToI2C};
    use crate::{mlx90640, mlx90641, Subpage};
    use crate::{I2cRegister, MelexisCamera, Mlx90640Driver, Mlx90641Driver, StatusRegister};

    fn create_mlx90640() -> Mlx90640Driver<MockCameraBus<MLX90640_RAM_LENGTH>> {
        // Specifically using a non-default address to make sure assumptions aren't being made
        // about the address.
        let address: u8 = 0x30;
        let mock_bus = datasheet_mlx90640_at_address(address);
        Mlx90640Driver::new(mock_bus, address)
            .expect("A MLX90640 camera should be created after loading its data")
    }

    fn create_mlx90641() -> Mlx90641Driver<MockCameraBus<MLX90641_RAM_LENGTH>> {
        // Again, non default address, but different from the '640 mock as well
        let address: u8 = 0x28;
        let mock_bus = mock_mlx90641_at_address(address);
        Mlx90641Driver::new(mock_bus, address)
            .expect("A MLX90641 camera should be created after loading its data")
    }

    #[test]
    fn smoke_test() {
        create_mlx90640();
        create_mlx90641();
        // Test passes if we get this far.
    }

    #[test]
    fn read_register() {
        // Just picking addresses now
        let address = 0x10;
        let mut mock_bus = datasheet_mlx90640_at_address(address);
        mock_bus.clear_recent_operations();
        let register = I2cRegister::from_i2c(&mut mock_bus, address).unwrap();
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
        let mut status_register = StatusRegister::from_i2c(&mut mock_bus, address).unwrap();
        assert_eq!(mock_bus.recent_operations().len(), 1);
        assert!(!status_register.overwrite_enabled());
        status_register.set_overwrite_enabled(true);
        status_register.to_i2c(&mut mock_bus, address).unwrap();
        assert_eq!(mock_bus.recent_operations().len(), 2);
    }

    #[test]
    fn default_emissivity() {
        // The MLX90640 doesn't store emissivity in EEPROM, so it should *always* default to 1
        let mut cam = create_mlx90640();
        assert_eq!(cam.effective_emissivity(), 1f32);
        // When we override it, it should change. Let's use the value for limestone from Wikipedia.
        cam.override_emissivity(0.92);
        assert_eq!(cam.effective_emissivity(), 0.92);
        // And we can reset it back to 0
        cam.use_default_emissivity();
        assert_eq!(cam.effective_emissivity(), 1f32);
    }

    #[test]
    fn set_reflected_temperature() {
        // The true temperature of the ambient environment is unknown initially, so it should be
        // none at first.
        let mut cam = create_mlx90640();
        assert!(cam.reflected_temperature().is_none());
        // We should be able to set the reflected temperature
        const T_R: f32 = 10.5;
        cam.set_reflected_temperature(Some(T_R));
        assert_eq!(cam.reflected_temperature(), Some(T_R));
        // And un-set it.
        cam.set_reflected_temperature(None);
        assert!(cam.reflected_temperature().is_none());
    }

    #[test]
    fn mlx90640_datasheet_integration() {
        let mut cam = create_mlx90640();
        let mut temperatures = [0f32; mlx90640::Mlx90640::NUM_PIXELS];
        // The pixel used in the datasheet is part of Subpage 0, but the mocked example is on
        // Subpage 1 currently so we can't use `generate_image_if_ready`.
        let res = cam.generate_image_subpage_to(Subpage::Zero, &mut temperatures);
        assert!(res.is_ok());
        // Test pixel is (12, 16)
        const PIXEL_INDEX: usize = 11 * mlx90640::Mlx90640::WIDTH + 15;
        // The final calculations for T_o are nearly a worst-case scenario for precision. The
        // datasheet examples use arbitrary precision, which we just can't match with f32 (or even
        // f64).
        assert_approx_eq!(f32, temperatures[PIXEL_INDEX], 80.36331, epsilon = 0.5);
    }

    #[test]
    fn mlx90641_datasheet_integration() {
        let mut cam = create_mlx90641();
        let mut temperatures = [0f32; mlx90641::Mlx90641::NUM_PIXELS];
        let res = cam.generate_image_if_ready(&mut temperatures);
        assert!(res.is_ok());
        assert!(res.unwrap());
        // Test pixel is (6, 9)
        const PIXEL_INDEX: usize = 5 * mlx90641::Mlx90641::WIDTH + 8;
        assert_approx_eq!(f32, temperatures[PIXEL_INDEX], 80.129812, epsilon = 0.5);
    }

    #[test]
    fn mlx90640_example_integration() {
        let i2c_address = 0x43;
        let mut mocked = example_mlx90640_at_address(i2c_address);
        mocked.set_data_available(false);
        let mut cam = Mlx90640Driver::new(mocked.clone(), i2c_address).unwrap();
        let mut temperatures = [f32::NAN; mlx90640::Mlx90640::NUM_PIXELS];
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
            mlx90640_example_data::FRAME_1_DATA,
            mlx90640_example_data::FRAME_1_STATUS_REGISTER,
        );
        mocked.set_data_available(true);
        assert!(cam.generate_image_if_ready(&mut temperatures).unwrap());
        // Check the results
        let paired = temperatures
            .iter()
            .zip(mlx90640_example_data::TEMPERATURES.iter());
        for (index, (actual, expected)) in paired.enumerate() {
            assert!(
                approx_eq!(f32, *actual, *expected, epsilon = 0.001),
                "[pixel {:?}]:\n{:>10}: `{:?}`,\n{:>10}: `{:?}`,",
                index,
                "expected",
                *expected,
                "actual",
                *actual
            );
        }
    }

    fn create_sentinel_buffer() -> [u8; mlx90641::Mlx90641::NUM_PIXELS * 2] {
        let mut buf = [0u8; mlx90641::Mlx90641::NUM_PIXELS * 2];
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
        let ram_data_result =
            super::read_ram::<mlx90641::Mlx90641, _, { mlx90641::Mlx90641::HEIGHT }>(
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
        let ram_data_result =
            super::read_ram::<mlx90641::Mlx90641, _, { mlx90641::Mlx90641::HEIGHT }>(
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
        let mut cam = Mlx90640Driver::new(mocked.clone(), i2c_address).unwrap();
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
        let mut cam = Mlx90640Driver::new(mocked.clone(), i2c_address).unwrap();
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
