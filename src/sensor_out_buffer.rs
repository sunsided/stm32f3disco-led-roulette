//! This module aims at simplifying the sending of data over the USB CDC in a controlled way.

use core::ops::Range;

use serial_sensors_proto::types::{
    AccelerometerI16, Identification, LinearRangeInfo, MagnetometerI16, TemperatureI16,
};
use serial_sensors_proto::versions::Version1DataFrame;
use serial_sensors_proto::{Identifier, IdentifierCode, LinearRanges, SensorId, SensorIds};

const BUFFER_SIZE: usize = 192;
const ACCEL_SENSOR_ID: SensorId = SensorIds::ACCELEROMETERI16
    .with_sensor_tag(lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _);
const MAG_SENSOR_ID: SensorId = SensorIds::MAGNETOMETERI16
    .with_sensor_tag(lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _);
const TEMP_SENSOR_ID: SensorId = SensorIds::TEMPERATUREI16
    .with_sensor_tag(lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _);

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
    transmit_buffer: [u8; BUFFER_SIZE],
    write_remaining: Range<usize>,
    remaining_identifiers: u8,
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
            transmit_buffer: [0_u8; BUFFER_SIZE],
            write_remaining: 0..0,
            remaining_identifiers: 0,
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

    /// Initiate the sending of identification data.
    pub fn send_identification_data(&mut self) {
        if self.remaining_identifiers == 0 {
            self.remaining_identifiers = 1;
        }
    }

    /// Updates the transmit data if possible and returns `true` if data was made
    /// available or is still available from a previous write.
    pub fn update_transmit_buffer(&mut self) -> bool {
        if !self.write_remaining.is_empty() {
            return true;
        }

        // Send identification frames first, if we can.
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
        } else if self.remaining_identifiers > 0 {
            match self.remaining_identifiers {
                // Board
                1 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        0x00, // device
                        Identification::new(Identifier::new(
                            SensorId::META_IDENTIFIER,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                2 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        0x00, // device
                        Identification::new(Identifier::new(
                            SensorId::META_IDENTIFIER,
                            IdentifierCode::Product,
                            "STM32F3 Discovery",
                        )),
                    ))
                }
                // Accelerometer
                3 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            ACCEL_SENSOR_ID,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                4 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            ACCEL_SENSOR_ID,
                            IdentifierCode::Product,
                            "LSM3030DLHC",
                        )),
                    ))
                }
                5 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _,
                        LinearRangeInfo::new(LinearRanges {
                            target: ACCEL_SENSOR_ID,
                            // (2*2^15)/(16384*(2- -2) = 1
                            resolution_bits: 16,
                            lsb_per_unit: 16384, // (2*32767)/(4 g)
                            meas_range_max: 2,   // + 2 g
                            meas_range_min: -2,  // - 2 g
                            range_decimals: 0,   // range is +/- 2*10^0
                            ..Default::default()
                        }),
                    ))
                }
                // Magnetometer
                6 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            MAG_SENSOR_ID,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                7 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            MAG_SENSOR_ID,
                            IdentifierCode::Product,
                            "LSM3030DLHC",
                        )),
                    ))
                }
                8 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        LinearRangeInfo::new(LinearRanges {
                            target: MAG_SENSOR_ID,
                            // e.g. (2*2^11)/(1100*(1.3- -1.3)) = 1.43... Gauss
                            resolution_bits: 12, // -2048 .. 2047
                            lsb_per_unit: 1100,  // LSB/Gauss
                            meas_range_max: 13,  // + 1.3 Gauss
                            meas_range_min: -13, // - 1.3 Gauss
                            range_decimals: 1,   // range is +/- 1.3*10^-1
                            ..Default::default()
                        }),
                    ))
                }
                // Temperature
                9 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            TEMP_SENSOR_ID,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                10 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            TEMP_SENSOR_ID,
                            IdentifierCode::Product,
                            "LSM3030DLHC",
                        )),
                    ))
                }
                11 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                        LinearRangeInfo::new(LinearRanges {
                            target: TEMP_SENSOR_ID,
                            // (2*2^11)/(8*(80- -40)) + 20 = 26.023529411764706
                            resolution_bits: 12,
                            lsb_per_unit: 8, // 8 LSB/°C
                            meas_range_max: 80,
                            meas_range_min: -40,
                            range_decimals: 0, // range is times 10^0
                            offset: 20,
                            offset_decimals: 0,
                        }),
                    ))
                }
                _ => {
                    self.remaining_identifiers = 0;
                    None
                }
            }
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
