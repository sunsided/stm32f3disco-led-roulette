//! Initialization code
// SPDX-License-Identifier: MIT or Apache-2.0

#![no_std]
#![allow(unsafe_code)]

pub use cortex_m;
pub use cortex_m_rt;
pub use panic_itm; // panic handler
pub use stm32f3xx_hal;
pub use switch_hal;

pub mod button;
pub mod compass;
pub mod leds;

/// Signals the process to go into low power mode until an interrupt occurs
pub fn wait_for_interrupt() {
    cortex_m::asm::wfi()
}
