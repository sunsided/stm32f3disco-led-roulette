use core::cell::Cell;

use serial_sensors_proto::types::{AccelerometerI16, MagnetometerI16, TemperatureI16};

pub struct SensorOutBuffer {
    accelerometer: Cell<Option<AccelerometerI16>>,
    magnetometer: Cell<Option<MagnetometerI16>>,
    temperature: Cell<Option<TemperatureI16>>,
}

#[allow(dead_code)]
impl SensorOutBuffer {
    pub fn update_accel(&self, reading: AccelerometerI16) {
        self.accelerometer.replace(Some(reading));
    }

    pub fn update_mag(&self, reading: MagnetometerI16) {
        self.magnetometer.replace(Some(reading));
    }

    pub fn update_temp(&self, reading: TemperatureI16) {
        self.temperature.replace(Some(reading));
    }
}
