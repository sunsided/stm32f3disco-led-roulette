//! This module aims at simplifying the sending of data over the USB CDC in a controlled way.

use core::ops::Range;

use serial_sensors_proto::types::{AccelerometerI16, MagnetometerI16, TemperatureI16};
use serial_sensors_proto::versions::Version1DataFrame;

/// This type ensures that we store sensor data until we're ready to process them,
/// and handles the serialization to the target buffer where possible.
pub struct SensorOutBuffer {
    accelerometer: Option<AccelerometerI16>,
    accel_events: u32,
    magnetometer: Option<MagnetometerI16>,
    mag_events: u32,
    temperature: Option<TemperatureI16>,
    temp_events: u32,
    total_events: u32,
    transmit_buffer: [u8; 64],
    write_remaining: Range<usize>,
}

#[allow(dead_code)]
impl SensorOutBuffer {
    pub fn new() -> Self {
        Self {
            accelerometer: None,
            accel_events: 0,
            magnetometer: None,
            mag_events: 0,
            temperature: None,
            temp_events: 0,
            total_events: 0,
            transmit_buffer: [0_u8; 64],
            write_remaining: 0..0,
        }
    }

    pub fn update_accel<I>(&mut self, reading: I)
    where
        I: Into<AccelerometerI16>,
    {
        self.accelerometer = Some(reading.into());
        self.accel_events = self.accel_events.wrapping_add(1);
    }

    pub fn update_mag<I>(&mut self, reading: I)
    where
        I: Into<MagnetometerI16>,
    {
        self.magnetometer = Some(reading.into());
        self.mag_events = self.mag_events.wrapping_add(1);
    }

    pub fn update_temp<I>(&mut self, reading: I)
    where
        I: Into<TemperatureI16>,
    {
        self.temperature = Some(reading.into());
        self.temp_events = self.temp_events.wrapping_add(1);
    }

    pub fn is_empty(&self) -> bool {
        self.accelerometer.is_none() && self.magnetometer.is_none() && self.temperature.is_none()
    }

    /// Returns the current unprocessed buffer range for debugging.
    pub fn buffer_range(&self) -> Range<usize> {
        self.write_remaining.clone()
    }

    /// Updates the transmit data if possible and returns `true` if data was made
    /// available or is still available from a previous write.
    pub fn update_transmit_buffer(&mut self) -> bool {
        if !self.write_remaining.is_empty() {
            return true;
        }

        let frame = if let Some(accelerometer) = self.accelerometer.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.accel_events,
                lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _,
                accelerometer,
            ))
        } else if let Some(magnetometer) = self.magnetometer.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.mag_events,
                lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                magnetometer,
            ))
        } else if let Some(temperature) = self.temperature.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.temp_events,
                lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                temperature,
            ))
        } else {
            None
        };

        match frame {
            None => false,
            Some(frame) => {
                match serial_sensors_proto::serialize(frame, &mut self.transmit_buffer) {
                    Ok(range) => {
                        self.write_remaining = range.clone();
                        true
                    }
                    Err(_err) => {
                        defmt::error!("A serialization error occurred");
                        false
                    }
                }
            }
        }
    }

    fn increment_total_events(&mut self) {
        self.total_events = self.total_events.wrapping_add(1);
    }

    pub fn transmit_buffer(&self) -> &[u8] {
        &self.transmit_buffer[self.write_remaining.clone()]
    }

    /// Commits the number of bytes consumed from the transmit buffer and
    /// returns the number of bytes remaining.
    pub fn commit_read(&mut self, bytes_read: usize) -> usize {
        self.write_remaining = (self.write_remaining.start + bytes_read)..self.write_remaining.end;
        defmt::info!(
            "Committing read of {} bytes, range now {}",
            bytes_read,
            self.write_remaining
        );
        self.write_remaining.len()
    }
}
