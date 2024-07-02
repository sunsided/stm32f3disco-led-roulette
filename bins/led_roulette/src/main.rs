//! Target board: STM32F3DISCOVERY

#![no_main]
#![no_std]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use accelerometer::RawAccelerometer;
use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use critical_section::Mutex;
use defmt_rtt as _;
use panic_probe as _;
use stm32f3xx_hal::{interrupt, pac, prelude::*, timer};
use stm32f3xx_hal::gpio::{Edge, gpioe, Gpioe, Input, Output, Pin, PushPull, U};
use stm32f3xx_hal::i2c::Error;
use stm32f3xx_hal::timer::Timer;
use stm32f3xx_hal::usb::{Peripheral, UsbBus};
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::compass::Compass;
use crate::leds::Leds;

mod leds;
mod compass;

/// Determines how often the timer interrupt should fire.
const WAKE_UP_EVERY_MS: u16 = 5;

/// Determines the duration between LED updates.
const DRIVE_LED_EVERY_MS: u16 = 30;

pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

/// Timer interrupt from TIM2.
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

/// LSM303DLHC magnetometer data ready (DRDY) signal.
/// PE2 pin, used to clear the interrupt in the `EXTI2` (`EXTI2_TSC`) handler.
static PE2_INT: Mutex<RefCell<Option<Pin<Gpioe, U<2>, Input>>>> = Mutex::new(RefCell::new(None));

/// PE4 pin, used to clear the interrupt in the EXTI4 handler
static PE4_INT: Mutex<RefCell<Option<Pin<Gpioe, U<4>, Input>>>> = Mutex::new(RefCell::new(None));

/// Flag to drive the LED roulette.
static UPDATE_LED_ROULETTE: AtomicBool = AtomicBool::new(false);

/// Indicates whether magnetometer data is ready.
/// This is indicated by an interrupt on the [`PE2_INT`] pin and flagged in the `EXTI2_TSC` handler.
static MAGNETOMETER_READY: AtomicBool = AtomicBool::new(true);

/// Indicates whether accelerometer data is ready.
/// This is indicated by an interrupt on the [`PE4_INT`] pin and flagged in the `EXTI4` handler.
static ACCELEROMETER_READY: AtomicBool = AtomicBool::new(true);

#[entry]
fn main() -> ! {
    defmt::info!("Running {} {} (serial {})", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"), env!("SERIAL"));

    let mut dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // Initialize the system clock(s).
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // STM32F3 Discovery has an 8 MHz quartz.
        .sysclk(48.MHz()) // Set system clock to 48 MHz.
        .use_pll()
        .pclk1(24.MHz()) // Set APB1 clock to half the system clock.
        .pclk2(24.MHz())
        .freeze(&mut flash.acr);
    assert!(clocks.usbclk_valid());

    // Prepare the LEDs.
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
    let mut leds = leds.into_array();

    // Initialize the LSM303DLHC/LSM303AGR MEMS e-compass.
    let mut compass = Compass::new(
        gpiob.pb6, gpiob.pb7, &mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl, dp.I2C1, clocks, &mut rcc.apb1,
    ).expect("failed to set up compass");

    // Configure a timer to generate interrupts.
    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    let timer_interrupt = timer.interrupt();

    timer.enable_interrupt(timer::Event::Update);
    timer.start(5.milliseconds());

    // Put the timer in the global context.
    critical_section::with(|cs| {
        TIMER.replace(cs, Some(timer));
    });

    unsafe {
        NVIC::unmask(timer_interrupt);
    }

    // Configure PE2 as interrupt source for the LSM303DLHC's DRDY line.
    let mut pe2 = gpioe.pe2.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    syscfg.select_exti_interrupt_source(&pe2);
    pe2.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    pe2.enable_interrupt(&mut dp.EXTI);
    let mems_drdy_interrupt = pe2.interrupt();

    critical_section::with(|cs| *PE2_INT.borrow(cs).borrow_mut() = Some(pe2));
    unsafe { NVIC::unmask(mems_drdy_interrupt) };

    // Configure PE4 as interrupt source for the LSM303DLHC's INT1 line.
    let mut pe4 = gpioe.pe4.into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    syscfg.select_exti_interrupt_source(&pe4);
    pe4.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    pe4.enable_interrupt(&mut dp.EXTI);
    let mems_int1_interrupt = pe4.interrupt();

    critical_section::with(|cs| *PE4_INT.borrow(cs).borrow_mut() = Some(pe4));
    unsafe { NVIC::unmask(mems_int1_interrupt) };

    // F3 Discovery board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa
        .pa12
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    usb_dp.set_low().ok();

    defmt::trace!("Waiting after boot-up USB power cycle");
    asm::delay(clocks.sysclk().0 / 100);
    defmt::trace!("Done waiting for USB power cycle");

    // Enable USB.
    let usb_dm = gpioa
        .pa11
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let usb_dp = usb_dp.into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    let usb_bus = UsbBus::new(usb);
    let mut serial = SerialPort::new(&usb_bus);

    let descriptors = StringDescriptors::default()
        .manufacturer("github.com/sunsided")
        .product("stm32f3disco-rust")
        .serial_number(env!("SERIAL"));

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[descriptors]).unwrap()
        .device_class(USB_CLASS_CDC)
        // .self_powered(false)
        .build();

    let mut curr = 0;
    let mut led_state = FlipFlop::Flip;

    // Run a bit of welcoming logic.
    identify_compass(&mut compass);

    // Make the sensor really slow to simplify debugging.
    compass.slowpoke().unwrap();

    loop {
        // Must be called at least every 10 ms, i.e. at 100 Hz.
        let usb_event = usb_dev.poll(&mut [&mut serial]);

        // Check for a magnetometer event.
        if ACCELEROMETER_READY.swap(false, Ordering::AcqRel) {
            match compass.accel_raw() {
                Ok(value) => {
                    defmt::warn!("Received accelerometer data: {}, {}, {}", value.x, value.y, value.z)
                }
                Err(_) => { defmt::error!("Failed to read accelerometer data") }
            }
        }

        // Check for a magnetometer event.
        if MAGNETOMETER_READY.swap(false, Ordering::AcqRel) {
            match compass.mag_raw() {
                Ok(value) => {
                    use micromath::F32Ext;

                    let x = value.x as f32;
                    let y = value.y as f32;
                    let z = value.z as f32;
                    let inv_norm = (x * x + y * y + z * z).invsqrt();
                    let x = x * inv_norm;
                    let y = y * inv_norm;
                    let z = z * inv_norm;

                    defmt::info!("Received compass data: {}, {}, {} - ({}, {}, {})", value.x, value.y, value.z, x, y, z)
                }
                Err(err) => {
                    log_i2c_error(err);
                }
            }

            match compass.temp_raw() {
                Ok(value) => {
                    let base_value = value as f32 / 8.0;
                    defmt::info!("Received temperature: ±{}°C ({}°C)", base_value, base_value + 20.0)
                }
                Err(err) => {
                    log_i2c_error(err);
                }
            }
        }


        if UPDATE_LED_ROULETTE.swap(false, Ordering::AcqRel) {
            match led_state {
                FlipFlop::Flip => {
                    let next = (curr + 1) % 8;
                    leds[next].on().ok();
                    led_state = FlipFlop::Flop(curr);
                    curr = next;
                }
                FlipFlop::Flop(curr) => {
                    leds[curr].off().ok();
                    led_state = FlipFlop::Flip;
                }
            }
        }

        if !usb_event {
            // wait_for_interrupt(); // TODO: might be slower than necessary
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf[..]) {
            Ok(_count) => {
                // count bytes were read to &buf[..count]
                defmt::trace!("Received USB data");
            }
            Err(UsbError::WouldBlock) => {
                // No data received
                defmt::trace!("Received no USB data");
            }
            Err(err) => {
                // An error occurred
                defmt::error!("Failed to receive USB data: {}", err);
            }
        };

        match serial.write(&[0x3a, 0x29]) {
            Ok(_count) => {
                // count bytes were written
            }
            Err(UsbError::WouldBlock) => {
                // No data could be written (buffers full)
                defmt::trace!("Buffer full while writing USB data");
            }
            Err(err) => {
                // An error occurred
                defmt::error!("Failed to send USB data: {}", err);
            }
        };
    }
}

fn identify_compass(compass: &mut Compass) {
    match compass.identify() {
        Ok(true) => {
            defmt::info!("LSM303DLHC sensor identification succeeded");
        }
        Ok(false) => {
            defmt::error!("LSM303DLHC sensor identification failed");
        }
        Err(err) => {
            log_i2c_error(err);
        }
    };
}

fn log_i2c_error(err: Error) {
    match err {
        Error::Arbitration => defmt::error!("I2C arbitration error"),
        Error::Bus => defmt::error!("I2C bus error"),
        Error::Busy => defmt::warn!("I2C bus busy"),
        Error::Nack => defmt::error!("I2C NACK"),
        _ => defmt::error!("Unknown I2C error")
    }
}

/// LED flip-flop state machine.
enum FlipFlop {
    /// Turns on the next LED.
    Flip,
    /// Turns off the specified LED (index).
    Flop(usize),
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
    asm::wfi()
}


#[interrupt]
fn TIM2() {
    static mut COUNT: u16 = 0;
    const LED_UPDATE_COUNT: u16 = DRIVE_LED_EVERY_MS / WAKE_UP_EVERY_MS;

    // Just handle the pending interrupt event.
    critical_section::with(|cs| {
        if let Some(ref mut timer) = TIMER
            // Unlock resource for use in critical section
            .borrow(cs)
            // Get a mutable reference from the RefCell
            .borrow_mut()
            // Make the inner Option<T> -> Option<&mut T>
            .as_mut()
        {
            *COUNT += 1;
            if *COUNT == LED_UPDATE_COUNT {
                UPDATE_LED_ROULETTE.store(true, Ordering::Release);
                *COUNT = 0;
            }

            // Finally operate on the timer itself.
            timer.clear_event(timer::Event::Update);
        }
    })
}

/// Interrupt for PE2: DRDY of the LSM303DLHC
///
/// The external interrupt number maps to the MCU pin number.
#[interrupt]
fn EXTI2_TSC() {
    critical_section::with(|cs| {
        if let Some(ref mut pin) = PE2_INT
            .borrow(cs)
            .borrow_mut()
            .as_mut()
        {
            MAGNETOMETER_READY.store(true, Ordering::Release);
            pin.clear_interrupt();
        } else {
            defmt::error!("PE2_INT not set up");
        }
    });
}

/// Interrupt for PE4: INT1 of the LSM303DLHC
///
/// The external interrupt number maps to the MCU pin number.
#[interrupt]
fn EXTI4() {
    critical_section::with(|cs| {
        if let Some(ref mut pin) = PE4_INT
            .borrow(cs)
            .borrow_mut()
            .as_mut()
        {
            ACCELEROMETER_READY.store(true, Ordering::Release);
            pin.clear_interrupt();
        } else {
            defmt::error!("PE4_INT not set up");
        }
    });
}
