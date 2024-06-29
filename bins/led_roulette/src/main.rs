//! Target board: STM32F3DISCOVERY

// #![deny(unsafe_code)]  // TODO: Bring that back in
#![no_main]
#![no_std]

mod leds;

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m_rt::entry;
use stm32f3xx_hal::{delay::Delay, pac, prelude::*};
use stm32f3xx_hal::gpio::{gpioe, Output, PushPull};
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use crate::leds::Leds;

pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();;

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

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

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // STM32F3 Discovery has an 8 MHz quartz.
        .sysclk(48.MHz()) // Set system clock to 48 MHz.
        .pclk1(24.MHz()) // Set APB1 clock to half the system clock.
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);
    let mut leds = leds.into_array();

    let ms: u16 = 1000;
    let mut curr = 0;
    loop {
        let next = (curr + 1) % 8;

        leds[next].on().ok();
        delay.delay_ms(ms);

        leds[curr].off().ok();
        delay.delay_ms(ms);

        curr = next;

        // cortex_m::asm::delay(8_000_000);
    }
}
