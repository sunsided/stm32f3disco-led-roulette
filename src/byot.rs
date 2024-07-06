use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

/// System ticks. Seconds since system start. Driven by the `SysTick` exception.
static TIME_SECONDS: AtomicU32 = AtomicU32::new(0);

/// Milliseconds since of the [`crate::TIME_SECONDS`] second. Driven by the `SysTick` exception.
static TIME_SUBSEC_MILLIS: AtomicU16 = AtomicU16::new(0);

/// Bring your own time. ✌️
pub struct Byot;

impl Byot {
    /// Called by the SysTick interrupt at a 1ms rate.
    pub fn systick() {
        let value = TIME_SUBSEC_MILLIS.fetch_add(1, Ordering::Release);
        if value == 1000 {
            TIME_SUBSEC_MILLIS.store(0, Ordering::Release);
            TIME_SECONDS.fetch_add(1, Ordering::Release);
        }
    }

    /// The current seconds since system time start.
    pub fn seconds() -> u32 {
        TIME_SECONDS.load(Ordering::Acquire)
    }

    /// The milliseconds of the current second.
    pub fn subsec_millis() -> u16 {
        TIME_SUBSEC_MILLIS.load(Ordering::Acquire)
    }
}
