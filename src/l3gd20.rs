#![allow(dead_code)]

use chip_select::{ChipSelect, ChipSelectActiveLow, ChipSelectGuarded, DeselectOnDrop};
use l3gd20_ng::{I16x3, L3GD20SPI};
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
pub struct Gyroscope {
    l3gd20: L3GD20SPI<GyroscopeChipSelect, SpiType>,
}

/// Chip select for the L3GD20.
pub struct GyroscopeChipSelect(
    bool,
    ChipSelectActiveLow<Pin<Gpioe, U<3>, Output<PushPull>>>,
);

impl ChipSelect for GyroscopeChipSelect {
    fn select(&mut self) {
        self.1.select()
    }

    fn deselect(&mut self) {
        self.1.deselect()
    }
}

impl ChipSelectGuarded for GyroscopeChipSelect {
    type Guard<'a> = DeselectOnDrop<'a, ChipSelectActiveLow<Pin<Gpioe, U<3>, Output<PushPull>>>>;

    fn select_guard(&mut self) -> Self::Guard<'_> {
        self.1.select_guard()
    }
}

impl GyroscopeChipSelect {
    /// Initialize the chip select.
    pub fn new<CsMode>(
        cs: gpioe::PE3<CsMode>,
        moder: &mut gpioe::MODER,
        otyper: &mut gpioe::OTYPER,
    ) -> Self {
        let mut l3gd20_cs = cs.into_push_pull_output(moder, otyper);
        // Disable the line.
        l3gd20_cs.set_high().ok();
        GyroscopeChipSelect(false, l3gd20_cs.into())
    }
}

impl Gyroscope {
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
        chip_select: GyroscopeChipSelect,
    ) -> Result<Self, Error> {
        let sck = sck.into_af_push_pull(moder, otyper, alternate_function_low);
        let miso = miso.into_af_push_pull(moder, otyper, alternate_function_low);
        let mosi = mosi.into_af_push_pull(moder, otyper, alternate_function_low);

        // Set up SPI.
        let spi = Spi::new(spi, (sck, miso, mosi), 3.MHz(), clocks, advanced_periph_bus);
        let device = L3GD20SPI::new(spi, chip_select)?;
        Ok(Self { l3gd20: device })
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn identify(&mut self) -> Result<bool, Error> {
        self.l3gd20.identify()
    }

    /// Resets the device to reasonable defaults.
    pub fn reset(&mut self) -> Result<(), Error> {
        self.l3gd20.reset()
    }

    /// Make the sensor as slow as possible.
    pub fn slowpoke(&mut self) -> Result<(), Error> {
        self.l3gd20.set_odr(OutputDataRate::Hz95)
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn temp_raw(&mut self) -> Result<u8, Error> {
        self.l3gd20.temp_raw()
    }

    /// Fetches all data off the sensor.
    pub fn xyz_raw(&mut self) -> Result<I16x3, Error> {
        self.l3gd20.xyz_raw()
    }
}
