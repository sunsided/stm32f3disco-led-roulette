// Original source: https://github.com/rubberduck203/stm32f3-discovery/blob/45c7f1b4375d6c4b7ab4f70d5699323d6feb98cc/src/compass.rs
// SPDX-License-Identifier: MIT or Apache-2.0

use accelerometer::{Accelerometer, RawAccelerometer};
use accelerometer::vector::{F32x3, I16x3};
use lsm303dlhc_ng::MagOdr;
use lsm303dlhc_registers::mag::StatusRegisterM;
use stm32f3xx_hal::gpio;
use stm32f3xx_hal::gpio::{gpiob, OpenDrain};
use stm32f3xx_hal::i2c;
use stm32f3xx_hal::pac;
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::rcc;

type Lsm303 = lsm303dlhc_ng::LSM303DLHC<
    i2c::I2c<
        pac::I2C1,
        (
            gpiob::PB6<gpio::AF4<OpenDrain>>,
            gpiob::PB7<gpio::AF4<OpenDrain>>,
        ),
    >,
>;

pub struct Compass {
    lsm303dlhc: Lsm303,
}

impl Compass {
    /// Initialize the onboard Lsm303dhlc e-Compass
    #[allow(clippy::too_many_arguments)]
    pub fn new<Pb6Mode, Pb7Mode>(
        pb6: gpiob::PB6<Pb6Mode>,
        pb7: gpiob::PB7<Pb7Mode>,
        mode: &mut gpiob::MODER,
        otype: &mut gpiob::OTYPER,
        alternate_function_low: &mut gpiob::AFRL,
        i2c1: pac::I2C1,
        clocks: rcc::Clocks,
        advanced_periph_bus: &mut rcc::APB1,
    ) -> Result<Self, i2c::Error> {
        /*
         * Pinout:
         * PB6 -> SCL (clock)
         * PB7 -> SDA (data)
         * PE2 -> DRDY (magnometer data ready)
         * PE4 -> INT1 (configurable interrupt 1)
         * PE5 -> INT2 (configurable interrupt 2)
         * lsm303hdlc driver uses continuous mode, so no need to wait for interrupts on DRDY
         */
        let scl = pb6.into_af_open_drain(mode, otype, alternate_function_low);
        let sda = pb7.into_af_open_drain(mode, otype, alternate_function_low);
        let i2c = i2c::I2c::new(i2c1, (scl, sda), 400_000.Hz(), clocks, advanced_periph_bus);

        let lsm303dhlc = Lsm303::new(i2c)?;
        Ok(Compass {
            lsm303dlhc: lsm303dhlc,
        })
    }

    /// Read the raw magnetometer data
    pub fn mag_raw(&mut self) -> Result<I16x3, i2c::Error> {
        let reading = self.lsm303dlhc.mag_raw()?;
        Ok(I16x3::new(reading.x, reading.y, reading.z))
    }

    /// Read the raw magnetometer data
    pub fn mag_status(&mut self) -> Result<StatusRegisterM, i2c::Error> {
        let reg: StatusRegisterM = self.lsm303dlhc.read_register()?;
        Ok(reg)
    }

    pub fn slow_compass(&mut self) -> Result<(), i2c::Error> {
        self.lsm303dlhc.mag_odr(MagOdr::Hz1_5)
    }

    pub fn identify(&mut self) -> Result<bool, i2c::Error> {
        self.lsm303dlhc.identify()
    }

    /// To obtain float readings, divide by 8 LSB/°C and add an offset, presumably 20°C or 25°C.
    pub fn temp_raw(&mut self) -> Result<i16, i2c::Error> {
        self.lsm303dlhc.temp_raw()
    }

    /// Consume the Compass and return the underlying Lsm303dhlc
    #[allow(unused)]
    pub fn into_lsm303dlhc(self) -> Lsm303 {
        self.lsm303dlhc
    }
}

impl RawAccelerometer<I16x3> for Compass {
    type Error = i2c::Error;

    fn accel_raw(&mut self) -> Result<I16x3, accelerometer::Error<Self::Error>> {
        RawAccelerometer::<I16x3>::accel_raw(&mut self.lsm303dlhc)
    }
}

impl Accelerometer for Compass {
    type Error = i2c::Error;
    fn accel_norm(&mut self) -> Result<F32x3, accelerometer::Error<Self::Error>> {
        self.lsm303dlhc.accel_norm()
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<<Self as Accelerometer>::Error>> {
        self.lsm303dlhc.sample_rate()
    }
}
