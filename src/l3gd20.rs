#![allow(dead_code)]

use core::ops::{Deref, DerefMut};

use chip_select::{ChipSelect, ChipSelectActiveLow, ChipSelectGuarded, DeselectOnDrop};
use defmt::Format;
use l3gd20_registers::prelude::SPIRegister;
use l3gd20_registers::*;
use stm32f3xx_hal::gpio::{gpioa, gpioe, Alternate, Gpioa, Gpioe, Output, Pin, PushPull, U};
use stm32f3xx_hal::pac::SPI1;
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::rcc;
use stm32f3xx_hal::spi::{Error, Spi};

type SpiType = Spi<
    SPI1,
    (
        Pin<Gpioa, U<5>, Alternate<PushPull, 5>>,
        Pin<Gpioa, U<6>, Alternate<PushPull, 5>>,
        Pin<Gpioa, U<7>, Alternate<PushPull, 5>>,
    ),
    u8,
>;

/// SPI Driver for Bosch Sensortec L3GD20 Gyroscope
#[allow(non_snake_case)]
pub struct L3GD20SPI<CS> {
    cs: CS,
    spi: SpiType,
}

/// Chip select for the L3GD20.
pub struct L3GD20ChipSelect(
    bool,
    ChipSelectActiveLow<Pin<Gpioe, U<3>, Output<PushPull>>>,
);

impl ChipSelect for L3GD20ChipSelect {
    fn select(&mut self) {
        self.1.select()
    }

    fn deselect(&mut self) {
        self.1.deselect()
    }
}

impl ChipSelectGuarded for L3GD20ChipSelect {
    type Guard<'a> = DeselectOnDrop<'a, ChipSelectActiveLow<Pin<Gpioe, U<3>, Output<PushPull>>>>;

    fn select_guard(&mut self) -> Self::Guard<'_> {
        self.1.select_guard()
    }
}

impl L3GD20ChipSelect {
    /// Initialize the chip select.
    pub fn new<CsMode>(
        cs: gpioe::PE3<CsMode>,
        moder: &mut gpioe::MODER,
        otyper: &mut gpioe::OTYPER,
    ) -> Self {
        let mut l3gd20_cs = cs.into_push_pull_output(moder, otyper);
        // Disable the line.
        l3gd20_cs.set_high().ok();
        L3GD20ChipSelect(false, l3gd20_cs.into())
    }
}

impl<CS> L3GD20SPI<CS>
where
    CS: ChipSelect,
{
    /// Bit flag for a read command.
    const READ: u8 = 0b1000_0000;

    /// Bit flag for a write command.
    const WRITE: u8 = 0b0000_0000;

    /// Bit flag for a multi-address command; auto-increments addresses after each transfer.
    const MULTI: u8 = 0b0100_0000;

    /// Bit flag for a single-address command.
    const SINGLE: u8 = 0b0000_0000;

    /// Mask for register addresses.
    const REG_ADDR_MASK: u8 = 0b0011_1111;

    /// Initialize the SPI connection.
    ///
    /// This method does not drive the chipselect. Make sure to enable it by calling [`L3GD20SPI::select_chip`].
    #[allow(clippy::too_many_arguments)]
    pub fn new<SckMode, MisoMode, MosiMode>(
        sck: gpioa::PA5<SckMode>,
        miso: gpioa::PA6<MisoMode>,
        mosi: gpioa::PA7<MosiMode>,
        moder: &mut gpioa::MODER,
        otyper: &mut gpioa::OTYPER,
        alternate_function_low: &mut gpioa::AFRL,
        spi: SPI1,
        clocks: rcc::Clocks,
        advanced_periph_bus: &mut rcc::APB2,
        chip_select: CS,
    ) -> Result<Self, Error>
    where
        CS: ChipSelectGuarded,
    {
        let sck = sck.into_af_push_pull(moder, otyper, alternate_function_low);
        let miso = miso.into_af_push_pull(moder, otyper, alternate_function_low);
        let mosi = mosi.into_af_push_pull(moder, otyper, alternate_function_low);

        // Set up SPI.
        let spi = Spi::new(spi, (sck, miso, mosi), 3.MHz(), clocks, advanced_periph_bus);
        let mut device = Self {
            cs: chip_select,
            spi,
        };

        // Apply standard configuration.
        device.reset()?;
        Ok(device)
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn identify(&mut self) -> Result<bool, Error>
    where
        CS: ChipSelectGuarded,
    {
        let ident = self.read_register::<WhoAmI>()?;
        if ident.ident() == 0b11010100 {
            Ok(true)
        } else {
            defmt::warn!(
                "L3GD20 sensor identification failed; got {:08b}",
                ident.ident()
            );
            Ok(false)
        }
    }

    /// Resets the device to reasonable defaults.
    pub fn reset(&mut self) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        // Use a bulk write instead.
        self.write_register(
            ControlRegister1::default()
                .with_power_up(true)
                .with_x_enable(true)
                .with_y_enable(true)
                .with_z_enable(true)
                .with_output_data_rate(OutputDataRate::Hz95)
                .with_bandwidth(Bandwidth::Narrowest),
        )?;
        self.write_register(
            ControlRegister2::default()
                .with_hpm(HighpassFilterMode::NormalModeResetFilter)
                .with_hpcf(0),
        )?;
        self.write_register(
            ControlRegister3::default()
                .with_i1int1(false)
                .with_i1boot(false)
                .with_int1_low(false)
                .with_i2drdy(false)
                .with_i2wtm(false)
                .with_i2orun(false)
                .with_i2empty(false)
                .with_open_drain(false),
        )?;
        self.write_register(
            ControlRegister4::default()
                .with_block_data_update(false)
                .with_big_endian(false)
                .with_full_scale(Sensitivity::G250)
                .with_spi_serial_3wire(false),
        )?;
        self.write_register(ControlRegister5::default().with_boot(true))?; // toggle boot
        self.write_register(
            ControlRegister5::default()
                .with_boot(false)
                .with_fifo_enable(false)
                .with_hpen(false)
                .with_int1_sel(0)
                .with_out_sel(0),
        )?;

        Ok(())
    }

    /// Sets the be powered up and active.
    pub fn power_up(&mut self) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        self.modify_register(|reg: ControlRegister1| {
            reg.with_power_up(true)
                .with_x_enable(true)
                .with_y_enable(true)
                .with_z_enable(true)
        })
    }

    /// Sets the device to sleep mode.
    pub fn sleep_mode(&mut self) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        self.modify_register(|reg: ControlRegister1| {
            reg.with_power_up(true)
                .with_x_enable(false)
                .with_y_enable(false)
                .with_z_enable(false)
        })
    }

    /// Sets the device to be powered down.
    pub fn power_down(&mut self) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        self.modify_register(|reg: ControlRegister1| reg.with_power_up(false))
    }

    /// Sets the output data rate.
    pub fn set_odr(&mut self, data_rate: OutputDataRate) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        self.modify_register(|reg: ControlRegister1| reg.with_output_data_rate(data_rate))
    }

    /// Sets the output data rate.
    pub fn set_bandwidth(&mut self, bandwidth: Bandwidth) -> Result<(), Error>
    where
        CS: ChipSelectGuarded,
    {
        self.modify_register(|reg: ControlRegister1| reg.with_bandwidth(bandwidth))
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn temp_raw(&mut self) -> Result<u8, Error>
    where
        CS: ChipSelectGuarded,
    {
        let ident = self.read_register::<TemperatureRegister>()?;
        Ok(ident.temp())
    }

    /// Fetches all data off the sensor.
    pub fn raw_data(&mut self) -> Result<SensorData, Error>
    where
        CS: ChipSelectGuarded,
    {
        let _guard = self.cs.select_guard();

        // The registers come in the order Temperature (0x26), Status (0x27), XL, XH, YL, YH, ZL, ZH (0x2D)
        let command = Self::read_multi_cmd(*TemperatureRegister::REGISTER_ADDRESS);
        let mut buffer = [command, 0, 0, 0, 0, 0, 0, 0, 0];
        self.spi.transfer(&mut buffer)?;

        let temp = TemperatureRegister::from_bits(buffer[1]);
        let _status = StatusRegister::from_bits(buffer[2]); // TODO:
        let xlo = OutXLow::from_bits(buffer[3]);
        let xhi = OutXHigh::from_bits(buffer[4]);
        let ylo = OutYLow::from_bits(buffer[5]);
        let yhi = OutYHigh::from_bits(buffer[6]);
        let zlo = OutZLow::from_bits(buffer[7]);
        let zhi = OutZHigh::from_bits(buffer[8]);

        defmt::info!("{}", _status);

        let x = (xhi.bits() as i16) << 8 | (xlo.bits() as i16);
        let y = (yhi.bits() as i16) << 8 | (ylo.bits() as i16);
        let z = (zhi.bits() as i16) << 8 | (zlo.bits() as i16);

        Ok(SensorData {
            temperature: temp.temp(),
            x: Reading::new_fresh(x),
            y: Reading::new_fresh(y),
            z: Reading::new_fresh(z),
        })
    }

    /// Creates a read command for a given address. Does not auto-increment the address afterward.
    fn read_single_cmd(address: u8) -> u8 {
        Self::READ | Self::SINGLE | (address & Self::REG_ADDR_MASK)
    }

    /// Creates a read command for a given address. Does not auto-increment the address afterward.
    fn read_multi_cmd(address: u8) -> u8 {
        Self::READ | Self::MULTI | (address & Self::REG_ADDR_MASK)
    }

    /// Creates a write command for a given address. Does not auto-increment the address afterward.
    fn write_single_cmd(address: u8) -> u8 {
        Self::WRITE | Self::SINGLE | (address & Self::REG_ADDR_MASK)
    }

    /// Reads a single register. Assumes the chip is selected.
    pub fn read_register<R>(&mut self) -> Result<R, Error>
    where
        R: Register,
        CS: ChipSelectGuarded,
    {
        let _guard = self.cs.select_guard();
        let command = Self::read_single_cmd(*R::REGISTER_ADDRESS);
        let mut buffer = [command, 0];
        self.spi.transfer(&mut buffer)?;
        Ok(R::from_bits(buffer[1]))
    }

    /// Writes a single register. Assumes the chip is selected.
    pub fn write_register<B, R>(&mut self, register: B) -> Result<(), Error>
    where
        B: core::borrow::Borrow<R>,
        R: WritableRegister,
        CS: ChipSelectGuarded,
    {
        let _guard = self.cs.select_guard();
        let byte = register.borrow().to_bits();
        let command = Self::write_single_cmd(*R::REGISTER_ADDRESS);
        let mut buffer = [command, byte];
        self.spi.transfer(&mut buffer)?;
        Ok(())
    }

    /// Modifies a single register. Assumes the chip is selected.
    pub fn modify_register<F, R>(&mut self, f: F) -> Result<(), Error>
    where
        F: FnOnce(R) -> R,
        R: WritableRegister,
        CS: ChipSelectGuarded,
    {
        let register: R = self.read_register()?;
        let register = f(register);
        self.write_register(register)
    }
}

/// Sensor data.
#[derive(Debug, Format, Clone)]
pub struct SensorData {
    /// The temperature reading
    pub temperature: u8,
    /// The X-axis reading.
    pub x: Reading<i16>,
    /// The Y-axis reading.
    pub y: Reading<i16>,
    /// The Z-axis reading.
    pub z: Reading<i16>,
}

impl SensorData {
    /// Indicates whether any reading is stale.
    #[must_use]
    pub fn stale(&self) -> bool {
        self.x.stale() || self.y.stale() || self.z.stale()
    }

    /// Indicates whether all readings are fresh.
    #[must_use]
    pub fn fresh(&self) -> bool {
        self.x.fresh() && self.y.stale() || self.z.stale()
    }

    /// Indicates whether all readings are fresh or overrun.
    #[must_use]
    pub fn fresh_or_overrun(&self) -> bool {
        self.x.fresh_or_overrun() && self.y.fresh_or_overrun() || self.z.fresh_or_overrun()
    }

    /// Indicates whether this is an overrun reading.
    #[must_use]
    pub fn overrun(&self) -> bool {
        self.x.overrun() && self.y.overrun() || self.z.overrun()
    }
}

/// A sensor reading that captures the notion of recent and outdated information.
#[derive(Format, Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Reading<T> {
    /// The reading is stale.
    Stale(T),
    /// The reading is recent.
    Fresh(T),
    /// New data was written before old data was read.
    Overrun(T),
}

impl<T> Deref for Reading<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            Reading::Stale(x) => x,
            Reading::Fresh(x) => x,
            Reading::Overrun(x) => x,
        }
    }
}

impl<T> DerefMut for Reading<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            Reading::Stale(x) => x,
            Reading::Fresh(x) => x,
            Reading::Overrun(x) => x,
        }
    }
}

impl<T> Reading<T> {
    /// Creates a stale reading.
    #[must_use]
    pub fn new_stale(value: T) -> Self {
        Self::Stale(value)
    }

    /// Creates a fresh reading.
    #[must_use]
    pub fn new_fresh(value: T) -> Self {
        Self::Fresh(value)
    }

    /// Creates a fresh reading.
    #[must_use]
    pub fn new_overrun(value: T) -> Self {
        Self::Overrun(value)
    }

    /// Consumes self and returns the inner value.
    #[must_use]
    pub fn into_inner(self) -> T {
        match self {
            Reading::Stale(x) => x,
            Reading::Fresh(x) => x,
            Reading::Overrun(x) => x,
        }
    }

    /// Indicates whether this is a stale reading.
    #[must_use]
    pub fn stale(&self) -> bool {
        matches!(self, Reading::Stale(_))
    }

    /// Indicates whether this is a fresh reading.
    #[must_use]
    pub fn fresh(&self) -> bool {
        matches!(self, Reading::Fresh(_))
    }

    /// Indicates whether this is a fresh or an overrun reading.
    #[must_use]
    pub fn fresh_or_overrun(&self) -> bool {
        self.fresh() || self.overrun()
    }

    /// Indicates whether this is an overrun reading.
    #[must_use]
    pub fn overrun(&self) -> bool {
        matches!(self, Reading::Overrun(_))
    }
}
