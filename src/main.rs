//! Target board: STM32F3DISCOVERY

#![no_main]
#![no_std]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use accelerometer::RawAccelerometer;
use cortex_m::asm;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::debug;
use critical_section::Mutex;
use defmt_rtt as _;
use panic_probe as _;
use serial_sensors_proto::{ScalarData, Vector3Data};
use stm32f3xx_hal::gpio::{gpioe, Edge, Gpioe, Input, Output, Pin, PushPull, U};
use stm32f3xx_hal::time::rate::Hertz;
use stm32f3xx_hal::timer::Timer;
use stm32f3xx_hal::usb::{Peripheral, UsbBus};
use stm32f3xx_hal::{i2c, interrupt, pac, prelude::*, spi, timer};
use switch_hal::{ActiveHigh, OutputSwitch, Switch};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::byot::Byot;
use crate::compass::Compass;
use crate::gyro::{Gyroscope, GyroscopeChipSelect};
use crate::leds::Leds;
use crate::sensor_out_buffer::SensorOutBuffer;
use crate::utils::{Micros, Millis};

mod byot;
mod compass;
mod gyro;
mod leds;
mod sensor_out_buffer;
mod utils;

/// Timer interrupt from TIM2.
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

/// L3GD20 gyroscope data ready (DRDY) signal.
/// PE1 pin, used to clear the interrupt in the `EXTI1` handler.
static PE1_INT: Mutex<RefCell<Option<Pin<Gpioe, U<1>, Input>>>> = Mutex::new(RefCell::new(None));

/// LSM303DLHC magnetometer data ready (DRDY) signal.
/// PE2 pin, used to clear the interrupt in the `EXTI2` (`EXTI2_TSC`) handler.
static PE2_INT: Mutex<RefCell<Option<Pin<Gpioe, U<2>, Input>>>> = Mutex::new(RefCell::new(None));

/// PE4 pin, used to clear the interrupt in the EXTI4 handler
static PE4_INT: Mutex<RefCell<Option<Pin<Gpioe, U<4>, Input>>>> = Mutex::new(RefCell::new(None));

/// Flag to drive the LED roulette.
static ROTATE_LED_ROULETTE: AtomicBool = AtomicBool::new(false);

/// Indicates whether gyroscope data is ready.
/// This is indicated by an interrupt on the [`PE1_INT`] pin and flagged in the `EXTI1` handler.
static GYRO_READY: AtomicBool = AtomicBool::new(true);

/// Indicates whether magnetometer data is ready.
/// This is indicated by an interrupt on the [`PE2_INT`] pin and flagged in the `EXTI2_TSC` handler.
static MAGNETOMETER_READY: AtomicBool = AtomicBool::new(true);

/// Indicates whether accelerometer data is ready.
/// This is indicated by an interrupt on the [`PE4_INT`] pin and flagged in the `EXTI4` handler.
static ACCELEROMETER_READY: AtomicBool = AtomicBool::new(true);

/// Determines how often the timer interrupt should fire. See also [`TIM2_COUNTER`].
const TIM2_WAKE_UP_EVERY_US: Micros = Micros::new(100);

/// The wake-up frequency of timer 2.
const TIM2_FREQUENCY: Hertz = TIM2_WAKE_UP_EVERY_US.frequency();

/// The counter value of timer 2. See [`TIM2_WAKE_UP_EVERY_US`].
static TIM2_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Drive the LEDs at 100 Hz, i.e. a maximum duty cycle length of 10 ms.
const LED_PWM_FREQUENCY: Hertz = Hertz(100);

/// The 100% value for a duty cycle.
const LED_PWM_DUTY_CYCLE_MAX: u16 = u16::MAX;

/// The LED PWM duty-time counter.
static LED_PWM_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Determines the duration between LED updates.
const ROTATE_LED_RING_EVERY_MS: Millis = Millis::new(100);

pub type LedArray = [Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>; 8];

#[entry]
fn main() -> ! {
    defmt::info!(
        "Running {} {} (serial {})",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION"),
        env!("SERIAL")
    );

    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

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

    // Configure SysTick
    let mut syst = cp.SYST;
    let ticks_per_ms = clocks.sysclk().0 / 1_000; // SysTick ticks per millisecond
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(ticks_per_ms - 1); // 1 ms interval
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

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
        gpiob.pb6,
        gpiob.pb7,
        &mut gpiob.moder,
        &mut gpiob.otyper,
        &mut gpiob.afrl,
        dp.I2C1,
        clocks,
        &mut rcc.apb1,
    )
    .expect("failed to set up compass");

    // Initialize the L2GD20 SPI gyroscope.
    let gyro_cs = GyroscopeChipSelect::new(gpioe.pe3, &mut gpioe.moder, &mut gpioe.otyper);
    let mut gyro = Gyroscope::new(
        gpioa.pa5,
        gpioa.pa6,
        gpioa.pa7,
        &mut gpioa.moder,
        &mut gpioa.otyper,
        &mut gpioa.afrl,
        dp.SPI1,
        clocks,
        &mut rcc.apb2,
        gyro_cs,
    )
    .expect("failed to set up gyro");

    // Configure a timer to generate interrupts.
    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    let timer_interrupt = timer.interrupt();

    timer.enable_interrupt(timer::Event::Update);
    timer.start(TIM2_WAKE_UP_EVERY_US.microseconds());

    // Put the timer in the global context.
    critical_section::with(|cs| {
        TIMER.replace(cs, Some(timer));
    });

    unsafe {
        NVIC::unmask(timer_interrupt);
    }

    // Configure PE1 as interrupt source for the L3GD20's DRDY line.
    let mut pe1 = gpioe
        .pe1
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    syscfg.select_exti_interrupt_source(&pe1);
    pe1.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    pe1.enable_interrupt(&mut dp.EXTI);
    let gyro_drdy_interrupt = pe1.interrupt();

    critical_section::with(|cs| *PE1_INT.borrow(cs).borrow_mut() = Some(pe1));
    unsafe { NVIC::unmask(gyro_drdy_interrupt) };

    // Configure PE2 as interrupt source for the LSM303DLHC's DRDY line.
    let mut pe2 = gpioe
        .pe2
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    syscfg.select_exti_interrupt_source(&pe2);
    pe2.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    pe2.enable_interrupt(&mut dp.EXTI);
    let mems_drdy_interrupt = pe2.interrupt();

    critical_section::with(|cs| *PE2_INT.borrow(cs).borrow_mut() = Some(pe2));
    unsafe { NVIC::unmask(mems_drdy_interrupt) };

    // Configure PE4 as interrupt source for the LSM303DLHC's INT1 line.
    let mut pe4 = gpioe
        .pe4
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
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
        .strings(&[descriptors])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        // .self_powered(false)
        .build();

    // Run a bit of welcoming logic.
    identify_compass(&mut compass);
    identify_gyro(&mut gyro);

    // Make the sensor really slow to simplify debugging.
    if cfg!(feature = "slowpoke") {
        compass.slowpoke().unwrap();
    }

    // Handle sensor events with style.
    let mut sensor_buffer = SensorOutBuffer::new();

    // Get the gyro calibration data.
    let characteristics = gyro.characteristics().unwrap();
    sensor_buffer.update_gyro_characteristics(characteristics);

    // Prepare a table for LED duty cycles.
    let mut gamma_table: [u16; 100] = [0; 100];
    leds::populate_gamma_table(&mut gamma_table, LED_PWM_DUTY_CYCLE_MAX);
    defmt::debug!("{}", gamma_table);

    // The PWM duty cycles for each LED
    let mut ref_led_duty_cycles: [u16; 8] = [
        gamma_table[0],
        gamma_table[25 / 4],
        gamma_table[37 / 3],
        gamma_table[50 / 3],
        gamma_table[62 / 3],
        gamma_table[75 / 3],
        gamma_table[87 / 3],
        gamma_table[99 / 3],
    ];

    // Most recent accelerometer and heading data.
    let mut accelerometer: Option<Vector3Data<i16>> = None;
    let mut heading: Option<i16> = None;

    let mut previous = Byot::now();
    let mut last_ident_send = Byot::now();
    loop {
        // Check clock handling.
        let now = Byot::now();
        let a_second_has_passed = if now.whole_seconds_since(&previous) >= 1 {
            previous = now;
            if let Some(heading) = heading {
                defmt::info!("Compass heading: {} degree", heading);
            }
            true
        } else {
            false
        };

        // Rotate the ring according the heading.
        let phase = if let Some(heading) = heading {
            if (180_i16..=315).contains(&heading) {
                if ROTATE_LED_ROULETTE.swap(false, Ordering::Acquire) {
                    ref_led_duty_cycles.rotate_right(1);
                }
                false
            } else if (45_i16..=180).contains(&heading) {
                if ROTATE_LED_ROULETTE.swap(false, Ordering::Acquire) {
                    ref_led_duty_cycles.rotate_left(1);
                }
                false
            } else {
                true
            }
        } else {
            true
        };

        // Create copy of the duty cycles for processing.
        let mut led_duty_cycles = ref_led_duty_cycles;

        // Phase the ring on and off
        if phase {
            for entry in led_duty_cycles.iter_mut() {
                *entry =
                    gamma_table[(gamma_table.len() - 1) * Byot::subsec_millis() as usize / 1000];
            }
        }

        const NORTH: usize = 0;
        const NORTH_EAST: usize = 1;
        const EAST: usize = 2;
        const SOUTH_EAST: usize = 3;
        const SOUTH: usize = 4;
        const SOUTH_WEST: usize = 5;
        const WEST: usize = 6;
        const NORTH_WEST: usize = 7;

        // When accelerometer measurements are available, indicate them on the LED ring.
        if let Some(data) = accelerometer {
            const REF: i16 = 8192;

            if data.x >= REF {
                // Positive X is towards USB ports.
                if data.y >= REF {
                    led_duty_cycles[NORTH] = 0;
                    led_duty_cycles[WEST] = 0;
                    led_duty_cycles[NORTH_WEST] = LED_PWM_DUTY_CYCLE_MAX;
                } else if data.y <= -REF {
                    led_duty_cycles[NORTH] = 0;
                    led_duty_cycles[NORTH_EAST] = LED_PWM_DUTY_CYCLE_MAX;
                    led_duty_cycles[EAST] = 0;
                } else {
                    led_duty_cycles[NORTH] = LED_PWM_DUTY_CYCLE_MAX;
                    led_duty_cycles[NORTH_EAST] = 0;
                    led_duty_cycles[NORTH_WEST] = 0;
                }
            } else if data.x <= -REF {
                // Positive X is towards LED ring ports.
                if data.y >= REF {
                    led_duty_cycles[SOUTH] = 0;
                    led_duty_cycles[WEST] = 0;
                    led_duty_cycles[SOUTH_WEST] = LED_PWM_DUTY_CYCLE_MAX;
                } else if data.y <= -REF {
                    led_duty_cycles[SOUTH] = 0;
                    led_duty_cycles[SOUTH_EAST] = LED_PWM_DUTY_CYCLE_MAX;
                    led_duty_cycles[EAST] = 0;
                } else {
                    led_duty_cycles[SOUTH] = LED_PWM_DUTY_CYCLE_MAX;
                    led_duty_cycles[SOUTH_EAST] = 0;
                    led_duty_cycles[SOUTH_WEST] = 0;
                }
            } else if data.y >= REF {
                // Positive Y is towards USR button.
                led_duty_cycles[WEST] = LED_PWM_DUTY_CYCLE_MAX;
                led_duty_cycles[SOUTH_WEST] = 0;
                led_duty_cycles[NORTH_WEST] = 0;
            } else if data.y <= -REF {
                // Positive Y is towards RESET button.
                led_duty_cycles[EAST] = LED_PWM_DUTY_CYCLE_MAX;
                led_duty_cycles[NORTH_EAST] = 0;
                led_duty_cycles[SOUTH_EAST] = 0;
            }
        }

        // Drive LEDs.
        let led_duty_cycle_count = LED_PWM_COUNTER.load(Ordering::Acquire);
        for i in 0..led_duty_cycles.len() {
            let on_time = led_duty_cycles[i];
            if led_duty_cycle_count <= u32::from(on_time) {
                leds[i].on().ok();
            } else {
                leds[i].off().ok();
            }
        }

        // Must be called at least every 10 ms, i.e. at 100 Hz.
        let usb_has_events = usb_dev.poll(&mut [&mut serial]);

        // Handle reading of data first.
        if usb_has_events {
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
        }

        // Check for a magnetometer event.
        if ACCELEROMETER_READY.swap(false, Ordering::Acquire) {
            accelerometer = handle_accelerometer_data(&mut compass, &mut sensor_buffer);
        }

        // Check for a magnetometer event.
        if MAGNETOMETER_READY.swap(false, Ordering::Acquire) {
            heading = handle_magnetometer_data(&mut compass, &mut sensor_buffer);
        }

        // Check temperatures.
        if a_second_has_passed {
            handle_mag_temperature_data(&mut compass, &mut sensor_buffer);
            handle_gyro_temperature_data(&mut gyro, &mut sensor_buffer);
        }

        if GYRO_READY.swap(false, Ordering::Acquire) {
            handle_gyroscope_data(&mut gyro, &mut sensor_buffer);
        }

        // Only send sensor identifications every once in a while.
        if now.whole_seconds_since(&last_ident_send) >= 10 {
            last_ident_send = now;
            sensor_buffer.send_identification_data();
        }

        // Try to fill the buffer.
        let has_sensor_data = sensor_buffer.update_transmit_buffer();

        // If the USB bus isn't ready, go to sleep and retry later.
        // NOTE: This does not (seem to) mean that the device is ready for writing. If we
        // need to write, we have to actually do it regardless of the event flag.
        if !usb_has_events && !has_sensor_data {
            wait_for_interrupt();
            continue;
        }

        // Short-circuit and leave the bus alone.
        if !has_sensor_data {
            wait_for_interrupt();
            continue;
        }

        // Only serialize when there's nothing more to write.
        let transmit_buffer = sensor_buffer.transmit_buffer();
        if transmit_buffer.is_empty() {
            defmt::error!("Got empty buffer for transmission");
        }

        match serial.write(transmit_buffer) {
            Ok(count) => {
                let remaining = sensor_buffer.commit_read(count);
                if remaining > 0 {
                    defmt::warn!(
                        "Couldn't write completely, range is now {} (length {})",
                        sensor_buffer.buffer_range(),
                        remaining
                    );
                }
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

fn handle_mag_temperature_data(compass: &mut Compass, sensor_buffer: &mut SensorOutBuffer) {
    match compass.temp_raw() {
        Ok(value) => {
            sensor_buffer.update_mag_temp(ScalarData::new(value));

            let base_value = value as f32 / 8.0;
            defmt::info!(
                "Received magnetometer temperature: {} = ±{}°C ({}°C)",
                value,
                base_value,
                base_value + 25.0
            )
        }
        Err(err) => {
            log_i2c_error(err);
        }
    }
}

fn handle_gyro_temperature_data(gyro: &mut Gyroscope, sensor_buffer: &mut SensorOutBuffer) {
    match gyro.temp_raw() {
        Ok(value) => {
            sensor_buffer.update_gyro_temp(ScalarData::new(value as _));

            defmt::info!(
                "Received gyro temperature: {} = ±{}°C ({}°C)",
                value,
                value,
                value as i16 + 20
            )
        }
        Err(err) => {
            log_spi_error(err);
        }
    }
}

fn handle_magnetometer_data(
    compass: &mut Compass,
    sensor_buffer: &mut SensorOutBuffer,
) -> Option<i16> {
    match compass.mag_raw() {
        Ok(value) => {
            sensor_buffer.update_mag(Vector3Data::new(value.x, value.y, value.z));

            use micromath::F32Ext;
            let x = value.x as f32;
            let y = value.y as f32;
            let z = value.z as f32;
            let inv_norm = (x * x + y * y + z * z).invsqrt();
            let x = x * inv_norm;
            let y = y * inv_norm;
            let z = z * inv_norm;

            let mut heading = y.atan2(x).to_degrees();
            if heading < 0.0 {
                heading += 360.0;
            }

            sensor_buffer.update_heading(ScalarData::<i16>::new(heading as _));

            defmt::debug!(
                "Received compass data: {}, {}, {} - ({}, {}, {}) - {} degrees",
                value.x,
                value.y,
                value.z,
                x,
                y,
                z,
                heading
            );

            Some(heading as _)
        }
        Err(err) => {
            log_i2c_error(err);
            None
        }
    }
}

fn handle_accelerometer_data(
    compass: &mut Compass,
    sensor_buffer: &mut SensorOutBuffer,
) -> Option<Vector3Data<i16>> {
    match compass.accel_raw() {
        Ok(value) => {
            let data = Vector3Data::new(value.x, value.y, value.z);
            sensor_buffer.update_accel(data);
            defmt::debug!(
                "Received accelerometer data: {}, {}, {}",
                value.x,
                value.y,
                value.z
            );
            Some(data)
        }
        Err(_) => {
            defmt::error!("Failed to read accelerometer data");
            None
        }
    }
}

fn handle_gyroscope_data(gyro: &mut Gyroscope, sensor_buffer: &mut SensorOutBuffer) {
    match gyro.xyz_raw() {
        Ok(value) => {
            sensor_buffer.update_gyro(Vector3Data::new(value.x, value.y, value.z));
            defmt::debug!(
                "Received gyroscope data: {}, {}, {}",
                value.x,
                value.y,
                value.z
            )
        }
        Err(_) => {
            defmt::error!("Failed to read gyroscope data")
        }
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

fn identify_gyro(gryo: &mut Gyroscope) {
    match gryo.identify() {
        Ok(true) => {
            defmt::info!("L3GD20 gyro sensor identification succeeded");
        }
        Ok(false) => {
            defmt::error!("L3GD20 gyro sensor identification failed");
        }
        Err(err) => {
            log_spi_error(err);
        }
    };
}

fn log_i2c_error(err: i2c::Error) {
    match err {
        i2c::Error::Arbitration => defmt::error!("I2C arbitration error"),
        i2c::Error::Bus => defmt::error!("I2C bus error"),
        i2c::Error::Busy => defmt::warn!("I2C bus busy"),
        i2c::Error::Nack => defmt::error!("I2C NACK"),
        _ => defmt::error!("Unknown I2C error"),
    }
}

fn log_spi_error(err: spi::Error) {
    match err {
        spi::Error::Overrun => defmt::error!("SPI overrun"),
        spi::Error::ModeFault => defmt::error!("SPI CRC mode fault"),
        spi::Error::Crc => defmt::error!("SPI CRC error"),
        _ => defmt::error!("Unknown SPI error"),
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
    asm::wfi()
}

#[interrupt]
fn TIM2() {
    // A bit of silly counter logic. Legacy.
    static mut COUNT: u32 = 0;
    const LED_ROTATE_UPDATE_COUNT: u32 = ROTATE_LED_RING_EVERY_MS.div(TIM2_WAKE_UP_EVERY_US);

    // Increment the counter based on our selected frequency.
    // Trivially: At a timer frequency of 2 kHz, this will reach 2000 in one second.
    // The value is delayed by one count, but that shouldn't matter.
    let _count = TIM2_COUNTER.fetch_add(1, Ordering::Release);

    // At 2 kHz, the counter reaches 2000 in 1 second, with one tick every 0.5 ms (500 ns).
    //
    // If we want to drive our LEDs at 100 Hz, we need to reach 0..MAX in 10 ms.
    // For that, we need to drive the LED PWM counter by 1 tick every 20 tick of the TIM2
    // counter (since 10 ms / 0.5 ms = 20, or 2000 Hz / 100 Hz = 20).
    const TICKS: u32 = TIM2_FREQUENCY.0 / LED_PWM_FREQUENCY.0;
    let value = LED_PWM_COUNTER.fetch_add(LED_PWM_DUTY_CYCLE_MAX as u32 / TICKS, Ordering::Release);
    if value + 1 >= LED_PWM_DUTY_CYCLE_MAX as u32 {
        LED_PWM_COUNTER.fetch_sub(LED_PWM_DUTY_CYCLE_MAX as u32, Ordering::Release);
    }

    // Rotate the LED ring.
    *COUNT += 1;
    if *COUNT == LED_ROTATE_UPDATE_COUNT {
        ROTATE_LED_ROULETTE.store(true, Ordering::Release);
        *COUNT = 0;
    }

    // Just handle the pending interrupt event.
    critical_section::with(|cs| {
        if let Some(ref mut timer) = TIMER.borrow_ref_mut(cs).as_mut() {
            // Finally operate on the timer itself.
            timer.clear_event(timer::Event::Update);
        }
    })
}

/// Interrupt for PE1: DRDY of the L3GD20
///
/// The external interrupt number maps to the MCU pin number.
#[interrupt]
fn EXTI1() {
    critical_section::with(|cs| {
        if let Some(ref mut pin) = PE1_INT.borrow(cs).borrow_mut().as_mut() {
            GYRO_READY.store(true, Ordering::Release);
            pin.clear_interrupt();
        } else {
            defmt::error!("PE1_INT not set up");
        }
    });
}

/// Interrupt for PE2: DRDY of the LSM303DLHC
///
/// The external interrupt number maps to the MCU pin number.
#[interrupt]
fn EXTI2_TSC() {
    critical_section::with(|cs| {
        if let Some(ref mut pin) = PE2_INT.borrow(cs).borrow_mut().as_mut() {
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
        if let Some(ref mut pin) = PE4_INT.borrow(cs).borrow_mut().as_mut() {
            ACCELEROMETER_READY.store(true, Ordering::Release);
            pin.clear_interrupt();
        } else {
            defmt::error!("PE4_INT not set up");
        }
    });
}

// Define the SysTick interrupt handler
#[exception]
fn SysTick() {
    Byot::systick();
}
