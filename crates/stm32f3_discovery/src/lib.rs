//! Initialization code
// SPDX-License-Identifier: MIT or Apache-2.0

#![no_std]
#![allow(unsafe_code)]

pub use cortex_m;
pub use cortex_m_rt::{self, entry};
pub use panic_itm; // panic handler
pub use stm32f3xx_hal;
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe, Output, PushPull},
    pac,
};
pub use switch_hal::{self, ActiveHigh, OutputSwitch, Switch, ToggleableOutputSwitch};

pub mod button;
pub mod compass;
pub mod leds;

pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

pub fn init() -> (Delay, LedArray) {
    let device_periphs = pac::Peripherals::take().unwrap();
    let mut reset_and_clock_control = device_periphs.RCC.constrain();

    let core_periphs = cortex_m::Peripherals::take().unwrap();
    let mut flash = device_periphs.FLASH.constrain();
    let clocks = reset_and_clock_control.cfgr.freeze(&mut flash.acr);
    let delay = Delay::new(core_periphs.SYST, clocks);

    // initialize user leds
    let mut gpioe = device_periphs.GPIOE.split(&mut reset_and_clock_control.ahb);
    let leds = leds::Leds::new(
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

/// Signals the process to go into low power mode until an interrupt occurs
pub fn wait_for_interrupt() {
    cortex_m::asm::wfi()
}
