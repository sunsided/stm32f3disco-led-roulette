use core::ops::Sub;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use core::time::Duration;

use defmt::Format;

/// System ticks. Seconds since system start. Driven by the `SysTick` exception.
static TIME_SECONDS: AtomicU32 = AtomicU32::new(0);

/// Milliseconds since of the [`crate::TIME_SECONDS`] second. Driven by the `SysTick` exception.
static TIME_SUBSEC_MILLIS: AtomicU16 = AtomicU16::new(0);

/// Bring your own time. ✌️
///
/// [`timer::MonoTimer`] probably also works.
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

    /// A duration since system start.
    #[allow(dead_code)]
    pub fn elapsed() -> Duration {
        Duration::new(Self::seconds() as _, Self::subsec_millis() as u32 * 1000)
    }

    /// Returns an instant representing now.
    #[must_use]
    pub fn now() -> ByotInstant {
        ByotInstant(Self::seconds(), Self::subsec_millis())
    }
}

#[derive(Format, Debug, Copy, Clone)]
pub struct ByotInstant(u32, u16);

impl ByotInstant {
    /// Determines the number of whole seconds elapsed since the previous instant.
    #[must_use]
    pub fn whole_seconds_since(&self, other: &ByotInstant) -> u32 {
        self.0 - other.0
    }
}

impl Sub<ByotInstant> for ByotInstant {
    type Output = Duration;

    fn sub(self, rhs: ByotInstant) -> Self::Output {
        Duration::new(self.0 as _, self.1 as u32 * 1000)
            - Duration::new(rhs.0 as _, rhs.1 as u32 * 1000)
    }
}
