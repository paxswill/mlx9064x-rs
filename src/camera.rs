use core::convert::TryInto;

use arrayvec::ArrayVec;
use embedded_hal::blocking::i2c;
use paste::paste;

use crate::common::*;
use crate::error::Error;
use crate::register::*;

/// Both the MLX90640 and MLX90641 use the same starting address for their EEPROM.
const EEPROM_BASE: [u8; 2] = 0x2400u16.to_be_bytes();

/// Both cameras (MLX90640 and MLX90641) have the same size EEPROM. 0x273F is the last address in
/// the EEPROM, so add one to that to include it, while 0x2400 is the first address. Each address
/// contains a 16-bit value, so we multiply by two to get the number of 8-bit bytes.
const EEPROM_LENGTH: usize = ((0x273F + 1) - 0x2400) * 2;

/// Constant needed a few times for the final pixel temperature calculations.
const KELVINS_TO_CELSIUS: f32 = 273.15;

/// DRY macro for the set_* methods in `Camera` that modify a register field.
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
            if current.$field != new_value {
                current.$field = new_value;
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
/// # Subpages and Access Patterns
/// One of the biggest differences you may encounter with these cameras is that not all of the
/// image is updated at once. The imaging area is divided into two [subpages][Subpage], each being
/// updated in turn. The pixels are split into subpages depending on the current [access
/// pattern][AccessPattern]. In chess board mode, the pixels alternate subpages in both the X and
/// Y axes, resulting in a chess or checker board-like pattern:
/// ```text
/// 0 1 0 1 0 1 0 1
/// 1 0 1 0 1 0 1 0
/// 0 1 0 1 0 1 0 1
/// 1 0 1 0 1 0 1 0
/// ```
/// The other access mode interleaves each row, so pixels will alternate subpages only on the Y
/// axis. This is also referred to as "TV" mode in the manufacturer's datasheet.
/// ```text
/// 0 0 0 0 0 0 0 0
/// 1 1 1 1 1 1 1 1
/// 0 0 0 0 0 0 0 0
/// 1 1 1 1 1 1 1 1
/// ```
/// The default mode is different between these two cameras, and the datasheet either strongly
/// advises against changing the access mode (90640), or doesn't mention the impact of changing the
/// access mode at all (90641).
///
/// The biggest impact to users of these modules is that one of the  `generate_image_*` functions
/// will need to be called twice (once for each subpage) before a full image is available.
#[derive(Clone, Debug)]
pub struct Camera<Cam, I2C, const HEIGHT: usize, const WIDTH: usize, const NUM_BYTES: usize> {
    /// The I²C bus this camera is accessible on.
    bus: I2C,

    /// The I²C address this camera is accessible at.
    address: u8,

    /// The camera-specific functionality.
    camera: Cam,

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
}

impl<Cam, I2C, const HEIGHT: usize, const WIDTH: usize, const NUM_PIXELS: usize>
    Camera<Cam, I2C, HEIGHT, WIDTH, NUM_PIXELS>
where
    Cam: MelexisCamera,
    I2C: i2c::WriteRead,
{
    /// Create a `Camera` for accessing the camera at the given I²C address.
    ///
    /// MLX90964\*s can be configured to use any I²C address (except 0x00), but the default address
    /// is 0x33.
    pub fn new(bus: I2C, address: u8) -> Result<Self, Error<I2C>> {
        // We own the bus now, make it mutable.
        let mut bus = bus;
        // Grab the control register values first
        // Need to map from I2C::Error manually as it's an associated type without bounds, so we
        // can't implement From<I2C:Error>
        let control: ControlRegister = read_register(&mut bus, address)?;
        // Dump the EEPROM. Both cameras use the same size and starting offset for their EEPROM.
        let mut eeprom_buf = [0u8; EEPROM_LENGTH];
        bus.write_read(address, &EEPROM_BASE, &mut eeprom_buf)
            .map_err(Error::I2cError)?;
        let camera = Cam::new(control, &eeprom_buf[..])?;
        // Cache this value
        let resolution_correction = camera.resolution_correction();
        // Choose an emissivity value to start with.
        let emissivity = camera.calibration().emissivity().unwrap_or(1f32);
        Ok(Self {
            bus,
            address,
            camera,
            pixel_buffer: [0u8; NUM_PIXELS],
            resolution_correction,
            ambient_temperature: None,
            emissivity,
        })
    }

    fn status_register(&mut self) -> Result<StatusRegister, Error<I2C>> {
        let register = read_register(&mut self.bus, self.address)?;
        Ok(register)
    }

    fn set_status_register(&mut self, register: StatusRegister) -> Result<(), Error<I2C>> {
        write_register(&mut self.bus, self.address, register)
    }

    fn control_register(&mut self) -> Result<ControlRegister, Error<I2C>> {
        let register = read_register(&mut self.bus, self.address)?;
        self.camera.update_control_register(register);
        // Update the resolution as well
        self.resolution_correction = self.camera.resolution_correction();
        Ok(register)
    }

    fn set_control_register(&mut self, register: ControlRegister) -> Result<(), Error<I2C>> {
        write_register(&mut self.bus, self.address, register)?;
        // Trigger yet another read to ensure we have the lastest value for the camera
        self.control_register()?;
        Ok(())
    }

    /// Get the last measured subpage.
    pub fn last_measured_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.status_register()?.last_updated_subpage)
    }

    /// Check if there is new data available, and if so, which subpage.
    pub fn data_available(&mut self) -> Result<Option<Subpage>, Error<I2C>> {
        let register = self.status_register()?;
        Ok(if register.new_data {
            Some(register.last_updated_subpage)
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
        current.new_data = false;
        self.set_status_register(current)
    }

    /// Check if the overwrite enabled flag is set.
    ///
    /// This flag is only effective when `data_hold_enabled` is active.
    pub fn overwrite_enabled(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.status_register()?.overwrite_enabled)
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
        Ok(self.control_register()?.use_subpages)
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
        Ok(self.control_register()?.data_hold)
    }

    set_register_field! {
        control_register,
        data_hold,
        "Enabled (or disable) data holding."
    }

    /// Check if the camera is in subpage repeat mode.
    ///
    /// This flag only has an effect if [subpages][Camera::subpages_enabled] is enabled. In subpage
    /// repeat mode, only the subpage set in `selected_subpage` will be measured and updated. When
    /// disabled, the active subpage will alternate between the two. The default is disabled.
    pub fn subpage_repeat(&mut self) -> Result<bool, Error<I2C>> {
        Ok(self.control_register()?.subpage_repeat)
    }

    set_register_field! {
        control_register,
        subpage_repeat,
        "Enabled (or disable) subpage repeat mode."
    }

    /// Get the currently selected subpage when [subpage repeat][Camera::subpage_repeat] is enabled.
    ///
    /// This setting only has an effect when `subpage_repeat` is enabled. The default value is
    /// `Subpage::Zero`.
    pub fn selected_subpage(&mut self) -> Result<Subpage, Error<I2C>> {
        Ok(self.control_register()?.subpage)
    }

    set_register_field! {
        control_register,
        subpage,
        Subpage,
        "Set the currently selected subpage when [subpage repeat][Camera::subpage_repeat] is enabled."
    }

    /// Read the frame rate from the camera.
    ///
    /// The default frame rate is [2 FPS][FrameRate::Two].
    pub fn frame_rate(&mut self) -> Result<FrameRate, Error<I2C>> {
        Ok(self.control_register()?.frame_rate)
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
        Ok(self.control_register()?.resolution)
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
        Ok(self.control_register()?.access_pattern)
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
    /// [overridden][Camera::override_emissivity], but this change is not stored on the camera.
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
        self.emissivity = self.camera.calibration().emissivity().unwrap_or(1f32);
    }

    /// Get the most recent ambient temperature calculation.
    ///
    /// The ambient temperature is calculated as part of the overall image calculations. If that
    /// process hasn't been performed yet (by calling `generate_image_if_ready` or similar), this
    /// method will return `None`.
    pub fn ambient_temperature(&self) -> Option<f32> {
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

    /// Read a value from the camera's RAM.
    ///
    /// All values in RAM are signed 16-bit integers, so this function also converts the raw values
    /// into `i16`.
    fn read_ram_value(&mut self, address: Address) -> Result<i16, Error<I2C>> {
        let address_bytes = address.as_bytes();
        let mut scratch = [0u8; 2];
        self.bus
            .write_read(self.address, &address_bytes[..], &mut scratch[..])
            .map_err(Error::I2cError)?;
        Ok(i16::from_be_bytes(scratch))
    }

    // The functions from here until the generate_* functions are all components of the temperature
    // calculation process. Splitting them out makes it easier to test them, and I hope the
    // compiler will inline them appropriately.

    fn delta_v(&self, v_dd_pixel: i16) -> f32 {
        f32::from(v_dd_pixel - self.camera.calibration().v_dd_25())
            / f32::from(self.camera.calibration().k_v_dd())
    }

    fn v_dd(&self, delta_v: f32) -> f32 {
        delta_v * self.resolution_correction + self.camera.calibration().v_dd_0()
    }

    fn v_ptat_art(&self, t_a_ptat: i16, t_a_v_be: i16) -> f32 {
        // Use i32 to prevent overflowing (which *will* happen if you stay as i16, these values
        // are large enough).
        let denom =
            t_a_ptat as i32 * self.camera.calibration().alpha_ptat() as i32 + t_a_v_be as i32;
        // Take the loss in precision when forcing a conversion to f32.
        f32::from(t_a_ptat) / denom as f32 * 18f32.exp2()
    }

    /// Calculate the ambient temperature, and cache it for later use.
    fn calculate_ambient_temperature(&mut self, v_ptat_art: f32, delta_v: f32) -> f32 {
        // Should probably change these in Calibration so they're already floats
        let k_v_ptat = f32::from(self.camera.calibration().k_v_ptat()) / 12f32.exp2();
        let v_ptat_25 = f32::from(self.camera.calibration().v_ptat_25());
        let numerator = (v_ptat_art / (1f32 + k_v_ptat * delta_v)) - v_ptat_25;
        let k_t_ptat = f32::from(self.camera.calibration().k_t_ptat()) / 3f32.exp2();
        self.ambient_temperature = Some(numerator / k_t_ptat + 25f32);
        self.ambient_temperature.unwrap()
    }

    /// Generate and copy a "raw" thermal image for the given subpage into the provided slice.
    ///
    /// This is one of the fundamental functions in this crate. It reads the raw data off of the
    /// camera, then copies it into the provided buffer (but only the pixels for the given
    /// subpage!). The copied data is the raw infrared measurement (in volts I think) from the
    /// camera, and has not been compensated for pixel sensitivity or ambient temperature. It *is*
    /// useful though for applications where just an "image" is needed but the actual temperatures
    /// are not.
    fn generate_raw_image_subpage_to(
        &mut self,
        subpage: Subpage,
        destination: &mut [f32],
    ) -> Result<(), Error<I2C>> {
        // This is going to be a monster function, as the results of earlier calculations are used
        // throughout the process.
        // Pick a maximum size of HEIGHT, as the worst access pattern is still by rows
        let pixel_ranges: ArrayVec<PixelAddressRange, HEIGHT> =
            self.camera.pixel_ranges(subpage).into_iter().collect();
        // Use the first pixel range's starting address as the first.
        // This is a weak assumption, but it holds so far with MLX90640.
        // TODO when '641 support is added, make sure this tested extensively.
        let base_address: usize = pixel_ranges[0].start_address.into();
        for range in pixel_ranges.iter() {
            let offset: usize = range.start_address.into();
            let offset = offset - base_address;
            let address_bytes = range.start_address.as_bytes();
            self.bus
                .write_read(
                    self.address,
                    &address_bytes[..],
                    &mut self.pixel_buffer[offset..(offset + range.length)],
                )
                .map_err(Error::I2cError)?;
        }
        // And now to read the non-pixel information out
        let t_a_v_be = self.read_ram_value(self.camera.t_a_v_be())?;
        let t_a_ptat = self.read_ram_value(self.camera.t_a_ptat())?;
        let v_dd_pixel = self.read_ram_value(self.camera.v_dd_pixel())?;
        let ram_gain = self.read_ram_value(self.camera.gain())?;
        let compensation_pixel = self.read_ram_value(self.camera.compensation_pixel(subpage))?;
        // Knock out the values common to all pixels first.
        let emissivity = self.emissivity;
        let delta_v = self.delta_v(v_dd_pixel);
        let v_dd = self.v_dd(delta_v);
        // Labelled V_PTAT in the formulas, but T_a_PTAT in the memory map.
        let v_ptat_art = self.v_ptat_art(t_a_ptat, t_a_v_be);
        let t_a = self.calculate_ambient_temperature(v_ptat_art, delta_v);
        let gain = f32::from(self.camera.calibration().gain()) / f32::from(ram_gain);
        // Compensation pixels are only used if temperature gradient compensation is being used.
        let compensation_pixel_offset = self
            .camera
            .calibration()
            .temperature_gradient_coefficient()
            .map(|tgc| {
                // TODO: There's a note in the datasheet advising a moving average filter (length >=
                // 16) on the compensation pixel gain.
                let compensation_pixel_offset = per_pixel_v_ir(
                    compensation_pixel,
                    gain,
                    self.camera.calibration().offset_reference_cp(subpage),
                    self.camera.calibration().k_v_cp(subpage),
                    self.camera.calibration().k_ta_cp(subpage),
                    t_a,
                    v_dd,
                    emissivity,
                );
                // Premultiplying by the TGC here
                tgc * compensation_pixel_offset
            });
        // At this point, we're now going to start calculating and copying over the pixel data. It
        // will *not* be actual temperatures, but it can be used for some imaging purposes.
        let mut pixel_subpage_sequence = self.camera.pixels_in_subpage(subpage);
        destination
            .iter_mut()
            // Chunk into two byte segments, for each 16-bit value
            .zip(self.pixel_buffer.chunks_exact(2))
            // Zip up the corresponding values from the calibration data.
            // Skipping alpha (sensitivity) as that's more related to temperature calculations
            .zip(self.camera.calibration().offset_reference_pixels(subpage))
            .zip(self.camera.calibration().k_v_pixels(subpage))
            .zip(self.camera.calibration().k_ta_pixels(subpage))
            // filter out the pixels that aren't part of this subpage
            .filter(|_| pixel_subpage_sequence.next().unwrap_or_default())
            // feeling a little lispy in here with all these parentheses
            .for_each(|((((output, pixel_slice), reference_offset), k_v), k_ta)| {
                // Safe to unwrap as this is from chunks_exact(2)
                let pixel_bytes: [u8; 2] = pixel_slice.try_into().unwrap();
                let pixel_data = i16::from_be_bytes(pixel_bytes);
                let mut pixel_offset = per_pixel_v_ir(
                    pixel_data,
                    gain,
                    *reference_offset,
                    *k_v,
                    *k_ta,
                    t_a,
                    v_dd,
                    emissivity,
                );
                // I hope the branch predictor/compiler is smart enough to realize there's
                // almost always going to be only one hot branch here
                if let Some(compensation_pixel_offset) = compensation_pixel_offset {
                    pixel_offset -= compensation_pixel_offset;
                }
                *output = pixel_offset;
            });
        Ok(())
    }

    fn generate_image_subpage_to(
        &mut self,
        subpage: Subpage,
        destination: &mut [f32],
    ) -> Result<(), Error<I2C>> {
        // Most of the heavy lifting is in this method
        self.generate_raw_image_subpage_to(subpage, destination)?;
        // TODO: this step could probably be optimized a little bit (if needed) by pushing this
        // calculation to the calibration loading step.
        let alpha_compensation_pixel = self
            .camera
            .calibration()
            .temperature_gradient_coefficient()
            .map(|tgc| self.camera.calibration().alpha_cp(subpage) * tgc);
        // Get the most recent ambient temperature (cached) and constants
        let t_a = self.ambient_temperature.expect(
            "The ambient temperature should be valid immediately after generating a raw image",
        );
        let native_index = self.camera.calibration().native_range();
        let k_s_to = self.camera.calibration().k_s_to();
        let k_s_to_native = k_s_to[native_index];
        let k_s_ta = self.camera.calibration().k_s_ta();
        let t_a0 = 25f32;
        // Little bit of optimization; this factor is shared by all pixels
        let alpha_coefficient = 1f32 + k_s_ta * (t_a - t_a0);
        // TODO: design a way to provide T-r, the reflected temperature. Basically, the temperature
        // of the surrounding environment (but not T_a, which is basically the temperature of the
        // sensor itself). For now hard-coding this to 8 degrees lower than T_a.
        let t_r = t_a - 8.0;
        // Again, start with the steps common to all pixels
        let t_a_k4 = (t_a + KELVINS_TO_CELSIUS).powi(4);
        let t_r_k4 = (t_r + KELVINS_TO_CELSIUS).powi(4);
        let t_ar = t_r_k4 - ((t_r_k4 - t_a_k4) / self.emissivity);

        let mut pixel_subpage_sequence = self.camera.pixels_in_subpage(subpage);
        destination
            .iter_mut()
            .zip(self.camera.calibration().alpha_pixels(subpage))
            // filter out the pixels that aren't part of this subpage
            .filter(|_| pixel_subpage_sequence.next().unwrap_or_default())
            .for_each(|(output, alpha)| {
                let v_ir = *output;
                let compensated_alpha = match alpha_compensation_pixel {
                    Some(alpha_compensation_pixel) => alpha - alpha_compensation_pixel,
                    None => *alpha,
                } * alpha_coefficient;
                *output = per_pixel_temparature(v_ir, compensated_alpha, t_ar, k_s_to_native);
            });
        Ok(())
    }

    /// Generate a thermal "image" from the camera's current data.
    ///
    /// This function does *not* check if there is new data, it just copies the
    pub fn generate_image_to(&mut self, destination: &mut [f32]) -> Result<(), Error<I2C>> {
        let subpage = self.last_measured_subpage()?;
        self.generate_image_subpage_to(subpage, destination)
    }

    /// Generate a thermal "image" from the camera's current data, if there's new data.
    ///
    // This function first checks to see if there is new data available, and if there is it copies
    // that data into the provided `ndarray::ArrayViewMut`. It will then clear the data ready flag
    // afterwards, signaliing to the camera that we are ready for more data.
    ///
    /// The `Ok` value is a boolean for whether or not data was ready and copied.
    pub fn generate_image_if_ready(&mut self, destination: &mut [f32]) -> Result<bool, Error<I2C>> {
        match self.data_available()? {
            Some(subpage) => {
                self.generate_image_subpage_to(subpage, destination)?;
                Ok(true)
            }
            None => Ok(false),
        }
    }
}

/// The per-pixel calculations to get a raw measurement of infrared radiation.
#[allow(clippy::too_many_arguments)]
fn per_pixel_v_ir(
    pixel_data: i16,
    gain: f32,
    reference_offset: i16,
    k_v: f32,
    k_ta: f32,
    t_a: f32,
    v_dd: f32,
    emissivity: f32,
) -> f32 {
    let pixel_gain = f32::from(pixel_data) * gain;
    let mut pixel_offset = pixel_gain
        - f32::from(reference_offset)
            * (1f32 + k_ta * (t_a - 25f32))
            * (1f32 + k_v * (v_dd - 3.3f32));
    pixel_offset /= emissivity;
    pixel_offset
}

/// The per-pixel calculations to go from a raw measurement to a temperature.
fn per_pixel_temparature(v_ir: f32, alpha: f32, t_ar: f32, k_s_to: f32) -> f32 {
    // This function is a mess of raising floats to the third and fourth powers, doing some
    // operations, then taking the fourth root of everything.
    let s_x = k_s_to * (alpha.powi(3) * v_ir + alpha.powi(4) * t_ar).powf(0.25);

    let t_o_root = (v_ir / (alpha * (1f32 - k_s_to * KELVINS_TO_CELSIUS) + s_x) + t_ar).powf(0.25);
    t_o_root - KELVINS_TO_CELSIUS
}

fn read_register<R, I2C>(bus: &mut I2C, address: u8) -> Result<R, Error<I2C>>
where
    I2C: i2c::WriteRead,
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
    let register_value = read_register(bus, address, register_address).map_err(Error::I2cError)?;
    let register = R::from(&register_value[..]);
    Ok(register)
}

fn write_register<R, I2C>(bus: &mut I2C, address: u8, register: R) -> Result<(), Error<I2C>>
where
    I2C: i2c::WriteRead,
    R: Register,
{
    fn write_register<I2C: i2c::WriteRead>(
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
        bus.write_read(i2c_address, &combined, &mut [])?;
        Ok(())
    }
    let register_address = R::address();
    let register_address_bytes = register_address.as_bytes();
    // Can't use read_register(), as it strips the unused bytes off
    let mut existing_value = [0u8; 2];
    bus.write_read(address, &register_address_bytes, &mut existing_value)
        .map_err(Error::I2cError)?;
    let mut new_bytes: [u8; 2] = register.into();
    new_bytes
        .iter_mut()
        .zip(R::write_mask())
        .zip(existing_value)
        .for_each(|((new_value, mask), old_value)| {
            *new_value &= mask;
            *new_value |= old_value & !mask;
        });
    write_register(bus, address, register_address_bytes, new_bytes).map_err(Error::I2cError)?;
    Ok(())
}

#[cfg(test)]
mod test {
    extern crate std;
    use core::iter::FromIterator;
    use std::vec;

    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction};
    use float_cmp::{approx_eq, F32Margin};

    use crate::{
        mlx90640, Camera, I2cRegister, MelexisCamera, Mlx90640Camera, StatusRegister, Subpage,
    };

    fn create_mlx90640() -> Mlx90640Camera<I2cMock> {
        // Specifically using a non-default address to make sure assumptions aren't being made
        // about the address.
        let address: u8 = 0x30;
        let eeprom_vec = vec::Vec::from_iter(mlx90640::eeprom_data());
        let expected = [
            // Control register. Using the value used in the datasheet for the worked example
            Transaction::write_read(address, vec![0x80, 0x0D], vec![0x09, 0x01]),
            // EEPROM
            Transaction::write_read(address, vec![0x24, 0x00], eeprom_vec),
        ];
        let mock_bus = I2cMock::new(&expected);
        Mlx90640Camera::new(mock_bus, address)
            .expect("A MLX90640 camera should be created after loading its data")
    }

    #[test]
    fn smoke_test() {
        create_mlx90640();
        // Test passes if we get this far.
    }

    #[test]
    fn read_register() {
        // Just picking addresses now
        let address = 0x10;
        let expected = [
            // Toggling all of the reserved bits on for kicks.
            Transaction::write_read(address, vec![0x80, 0x0F], vec![0xFF, 0xF0]),
        ];
        let mut mock_bus = I2cMock::new(&expected);
        let register: I2cRegister = super::read_register(&mut mock_bus, address).unwrap();
        assert_eq!(register, I2cRegister::default());
    }

    #[test]
    fn read_write_register() {
        let address = 0x42;
        // Using the status register for this test as it has read-only sections at both ends.
        let expected = [
            // Start with the read-only sections zeroed
            Transaction::write_read(address, vec![0x80, 0x00], vec![0x00, 0xE0]),
            // After modifying the register struct, write_register() will do another read, and all
            // reserved bits will be set, and the last updated subpage will have changed.
            // NOTE: the final byte is 0xC1, not 0xE1. This library handles step mode, which is
            // controlled by the 6th bit which is reserved in later datasheets).
            Transaction::write_read(address, vec![0x80, 0x00], vec![0xFF, 0xC1]),
            // Ensure that the reserved bits are kept, and the new setting (in this case, data
            // overwite) is also set
            Transaction::write_read(address, vec![0x80, 0x00, 0xFF, 0xD1], vec![]),
        ];
        let mut mock_bus = I2cMock::new(&expected);
        let mut status_register: StatusRegister =
            super::read_register(&mut mock_bus, address).unwrap();
        assert!(!status_register.overwrite_enabled);
        status_register.overwrite_enabled = true;
        super::write_register(&mut mock_bus, address, status_register).unwrap();
    }

    // The super bare, single function tests with magic number are using values from the worked
    // example in the MLX90640 datasheet.

    #[test]
    fn delta_v() {
        let cam = create_mlx90640();
        assert_eq!(cam.delta_v(-13115), 0.018623737)
    }

    #[test]
    fn v_dd() {
        let cam = create_mlx90640();
        // Datasheet has ≈3.319
        approx_eq!(f32, cam.v_dd(0.018623737), 3.3186, F32Margin::default());
    }

    #[test]
    fn v_ptat_art() {
        let cam = create_mlx90640();
        assert_eq!(cam.v_ptat_art(1711, 19442), 12873.57952);
    }

    #[test]
    fn t_a() {
        let mut cam = create_mlx90640();
        let delta_v = cam.delta_v(-13115);
        let v_ptat_art = cam.v_ptat_art(1711, 19442);
        cam.ambient_temperature = None;
        // The datasheet is a bit more precise than I can get with f32, so approx_eq here
        approx_eq!(
            f32,
            cam.calculate_ambient_temperature(v_ptat_art, delta_v),
            39.18440152,
            epsilon = 0.000002
        );
        assert!(cam.ambient_temperature.is_some());
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
    fn v_ir() {
        // also called "pixel offset" in a few places
        let cam = create_mlx90640();
        // The worked example uses pixel(12, 16) (and we use 0-indexing, so subtract one) and
        // subpage 1
        let pixel_index = 11 * mlx90640::WIDTH + 15;
        let offset = cam
            .camera
            .calibration()
            .offset_reference_pixels(Subpage::One)[pixel_index];
        let k_v = cam.camera.calibration().k_v_pixels(Subpage::One)[pixel_index];
        let k_ta = cam.camera.calibration().k_ta_pixels(Subpage::One)[pixel_index];
        let emissivity = cam.effective_emissivity();
        // Taken from the worked example
        let raw_pixel: i16 = 609;
        let gain = 1.01753546947234;
        let t_a = 39.18440152;
        let v_dd = 3.319;
        let v_ir = super::per_pixel_v_ir(raw_pixel, gain, offset, k_v, k_ta, t_a, v_dd, emissivity);
        // I'm getting 700.89, which is close enough considering how many places to lose precision
        // there are in this step.
        approx_eq!(f32, v_ir, 700.882495690866, epsilon = 0.01);
    }

    #[test]
    fn pixel_temperature() {
        let cam = create_mlx90640();
        let native_index = cam.camera.calibration().native_range();
        let k_s_to = cam.camera.calibration().k_s_to()[native_index];
        // The worked example is using TGC, which is done before per_pixel_temparature() is called,
        // so these values are hard-coded from the datasheet.
        let alpha = 1.1876487360496E-7;
        // Pile of values from previous steps.
        let v_ir = 679.250909123826;
        let t_ar = 9516495632.56;
        let t_o = super::per_pixel_temparature(v_ir, alpha, t_ar, k_s_to);
        assert_eq!(t_o, 80.36331);
    }
}
