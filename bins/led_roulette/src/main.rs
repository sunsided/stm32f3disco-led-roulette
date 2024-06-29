//! Target board: STM32F3DISCOVERY

#![no_main]
#![no_std]

mod leds;

use defmt_rtt as _; // global logger
use panic_probe as _;

use cortex_m_semihosting::debug;
use cortex_m_rt::entry;
use stm32f3xx_hal::{delay::Delay, pac, prelude::*};
use stm32f3xx_hal::gpio::{gpioe, Output, PushPull};
use switch_hal::{ActiveHigh, InputSwitch, IntoSwitch, OutputSwitch, Switch};
use crate::leds::Leds;

pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

#[entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    defmt::info!("info");
    defmt::trace!("trace");
    defmt::warn!("warn");
    defmt::debug!("debug");
    defmt::error!("error");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
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

    // Initialize PA0 as input with pull-down resistor
    let button = gpioa.pa0.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr)
        .downgrade()
        .into_active_high_switch();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // STM32F3 Discovery has an 8 MHz quartz.
        .sysclk(48.MHz()) // Set system clock to 48 MHz.
        .pclk1(24.MHz()) // Set APB1 clock to half the system clock.
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);
    let mut leds = leds.into_array();

    let ms: u16 = 30;
    let mut curr = 0;
    loop {
        let next = (curr + 1) % 8;

        leds[next].on().ok();
        delay.delay_ms(ms);

        leds[curr].off().ok();
        delay.delay_ms(ms);

        curr = next;

        match button.is_active() {
            Ok(true) => { defmt::info!("Button was pressed!"); }
            Ok(false) => { defmt::info!("Button was depressed! :(");  }
            Err(_) => { defmt::error!("Failed to read button state"); }
        }

        // cortex_m::asm::delay(8_000_000);
    }
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {
        debug::exit(debug::EXIT_FAILURE);
    }
}

/// Signals the process to go into low power mode until an interrupt occurs
pub fn wait_for_interrupt() {
    cortex_m::asm::wfi()
}
