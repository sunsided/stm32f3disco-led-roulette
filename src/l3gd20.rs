#![allow(dead_code)]

use chip_select::{ChipSelect, ChipSelectActiveLow};
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
    fn is_auto_select(&self) -> bool {
        self.1.is_auto_select()
    }

    fn select(&mut self) {
        self.1.select()
    }

    fn deselect(&mut self) {
        self.1.deselect()
    }
}

impl L3GD20ChipSelect {
    /// Initialize the chip select.
    pub fn new<CsMode>(
        cs: gpioe::PE3<CsMode>,
        moder: &mut gpioe::MODER,
        otyper: &mut gpioe::OTYPER,
    ) -> Self {
        let l3gd20_cs = cs.into_push_pull_output(moder, otyper);
        L3GD20ChipSelect(false, l3gd20_cs.into())
    }

    /// Enables auto-select on the chip.
    pub fn with_auto_select(mut self, enabled: bool) -> Self {
        self.0 = enabled;
        self
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
        CS: ChipSelect,
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

    /// Enables chip-select for this device.
    pub fn select_chip(&mut self)
    where
        CS: ChipSelect,
    {
        self.cs.select()
    }

    /// Disables chip-select for this device.
    pub fn deselect_chip(&mut self)
    where
        CS: ChipSelect,
    {
        self.cs.deselect()
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn identify(&mut self) -> Result<bool, Error>
    where
        CS: ChipSelect,
    {
        let ident = self.read_register::<WhoAmI>()?;
        Ok(ident.ident() == 0b11010100)
    }

    /// Resets the device to reasonable defaults.
    pub fn reset(&mut self) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| {
            reg.with_power_up(true)
                .with_x_enable(true)
                .with_y_enable(true)
                .with_z_enable(true)
                .with_output_data_rate(OutputDataRate::Hz380)
                .with_bandwidth(Bandwidth::Medium)
        })
    }

    /// Sets the be powered up and active.
    pub fn power_up(&mut self) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| {
            reg.with_power_up(true)
                .with_x_enable(true)
                .with_y_enable(true)
                .with_z_enable(true)
        })
    }

    /// Sets the device to sleep mode.
    pub fn sleep_mode(&mut self) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| {
            reg.with_power_up(true)
                .with_x_enable(false)
                .with_y_enable(false)
                .with_z_enable(false)
        })
    }

    /// Sets the device to be powered down.
    pub fn power_down(&mut self) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| reg.with_power_up(false))
    }

    /// Sets the output data rate.
    pub fn set_odr(&mut self, data_rate: OutputDataRate) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| reg.with_output_data_rate(data_rate))
    }

    /// Sets the output data rate.
    pub fn set_bandwidth(&mut self, bandwidth: Bandwidth) -> Result<(), Error> {
        self.modify_register(|reg: ControlRegister1| reg.with_bandwidth(bandwidth))
    }

    /// Creates a read command for a given address. Does not auto-increment the address afterward.
    fn read_single_cmd(address: u8) -> u8 {
        Self::READ | Self::SINGLE | (address & Self::REG_ADDR_MASK)
    }

    /// Creates a write command for a given address. Does not auto-increment the address afterward.
    fn write_single_cmd(address: u8) -> u8 {
        Self::WRITE | Self::SINGLE | (address & Self::REG_ADDR_MASK)
    }

    /// Reads a single register.
    pub fn read_register<R>(&mut self) -> Result<R, Error>
    where
        CS: ChipSelect,
        R: Register,
    {
        self.cs.auto_select();
        self.read_register_selected()
    }

    /// Writes a single register.
    pub fn write_register<B, R>(&mut self, register: B) -> Result<(), Error>
    where
        CS: ChipSelect,
        B: core::borrow::Borrow<R>,
        R: WritableRegister,
    {
        self.cs.auto_select();
        self.write_register_selected(register)
    }

    /// Modifies a single register.
    pub fn modify_register<F, R>(&mut self, f: F) -> Result<(), Error>
    where
        CS: ChipSelect,
        F: FnOnce(R) -> R,
        R: WritableRegister,
    {
        self.cs.auto_select();
        self.modify_register_selected(f)
    }

    /// Reads a single register. Assumes the chip is selected.
    fn read_register_selected<R>(&mut self) -> Result<R, Error>
    where
        R: Register,
    {
        let command = Self::read_single_cmd(*R::REGISTER_ADDRESS);
        let mut buffer = [command, 0];
        self.spi.transfer(&mut buffer)?;
        Ok(R::from_bits(buffer[1]))
    }

    /// Writes a single register. Assumes the chip is selected.
    pub fn write_register_selected<B, R>(&mut self, register: B) -> Result<(), Error>
    where
        B: core::borrow::Borrow<R>,
        R: WritableRegister,
    {
        let byte = register.borrow().to_bits();
        let command = Self::write_single_cmd(*R::REGISTER_ADDRESS);
        let mut buffer = [command, byte];
        self.spi.transfer(&mut buffer)?;
        Ok(())
    }

    /// Modifies a single register. Assumes the chip is selected.
    pub fn modify_register_selected<F, R>(&mut self, f: F) -> Result<(), Error>
    where
        F: FnOnce(R) -> R,
        R: WritableRegister,
    {
        let register: R = self.read_register_selected()?;
        let register = f(register);
        self.write_register_selected(register)
    }
}
