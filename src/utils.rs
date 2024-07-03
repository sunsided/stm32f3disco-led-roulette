use core::ops::{Deref, Div};

use stm32f3xx_hal::time::duration::Milliseconds;

/// Wraps a time, in milliseconds.
#[derive(Default, Debug, Copy, Clone, Eq, Ord, PartialOrd, PartialEq)]
pub struct Millis(u32);

impl Millis {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn div(&self, rhs: Millis) -> u32 {
        self.0 / rhs.0
    }

    pub const fn milliseconds(&self) -> Milliseconds<u32> {
        Milliseconds(self.0)
    }
}

impl Deref for Millis {
    type Target = u32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Div for Millis {
    type Output = u32;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl From<Millis> for Milliseconds<u32> {
    fn from(value: Millis) -> Self {
        Milliseconds(value.0)
    }
}
