#![no_main]
#![no_std]
#![deny(unsafe_code)]

use led_roulette_aux::LedArray;
use stm32f3_discovery::cortex_m_rt::entry;
use stm32f3_discovery::stm32f3xx_hal::{delay::Delay, prelude::*};
use stm32f3_discovery::switch_hal::OutputSwitch;

#[entry]
fn main() -> ! {
    let (mut delay, mut leds): (Delay, LedArray) = led_roulette_aux::init();

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
