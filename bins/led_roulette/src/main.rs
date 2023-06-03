#![no_main]
#![no_std]
#![deny(unsafe_code)]

use stm32f3_discovery::cortex_m;
use stm32f3_discovery::cortex_m_rt::entry;
use stm32f3_discovery::leds::Leds;
use stm32f3_discovery::stm32f3xx_hal::gpio::{gpioe, Output, PushPull};
use stm32f3_discovery::stm32f3xx_hal::{delay::Delay, pac, prelude::*};
use stm32f3_discovery::switch_hal::{ActiveHigh, OutputSwitch, Switch};

#[entry]
fn main() -> ! {
    let (mut delay, mut leds): (Delay, LedArray) = init();

    let ms = 30_u8;
    loop {
        for curr in 0..8 {
            let next = (curr + 1) % 8;

            leds[next].on().ok();
            delay.delay_ms(ms);

            leds[curr].off().ok();
            delay.delay_ms(ms);
        }
    }
}

/// The array of 8 LEDs available on the STM32F3 Discovery board.
pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

/// Initializes the MCU and prepares the LED array.
pub fn init() -> (Delay, LedArray) {
    let device_periphs = pac::Peripherals::take().unwrap();
    let mut reset_and_clock_control = device_periphs.RCC.constrain();

    let core_periphs = cortex_m::Peripherals::take().unwrap();
    let mut flash = device_periphs.FLASH.constrain();
    let clocks = reset_and_clock_control.cfgr.freeze(&mut flash.acr);
    let delay = Delay::new(core_periphs.SYST, clocks);

    // initialize user leds
    let mut gpioe = device_periphs.GPIOE.split(&mut reset_and_clock_control.ahb);
    let leds = Leds::new(
        gpioe.pe8,
        gpioe.pe9,
        gpioe.pe10,
        gpioe.pe11,
        gpioe.pe12,
        gpioe.pe13,
        gpioe.pe14,
        gpioe.pe15,
        &mut gpioe.moder,
        &mut gpioe.otyper,
    );

    (delay, leds.into_array())
}
