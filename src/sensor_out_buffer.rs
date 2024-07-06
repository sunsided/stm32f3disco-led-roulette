//! This module aims at simplifying the sending of data over the USB CDC in a controlled way.

use core::ops::Range;

use l3gd20_ng::Characteristics;
use serial_sensors_proto::types::*;
use serial_sensors_proto::versions::Version1DataFrame;
use serial_sensors_proto::*;

const BUFFER_SIZE: usize = 192;
const ACCEL_SENSOR_ID: SensorId = SensorIds::ACCELEROMETERI16
    .with_sensor_tag(lsm303dlhc_registers::accel::DEFAULT_DEVICE_ADDRESS as _);
const MAG_SENSOR_ID: SensorId = SensorIds::MAGNETOMETERI16
    .with_sensor_tag(lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _);
const MAG_TEMP_SENSOR_ID: SensorId = SensorIds::TEMPERATUREI16
    .with_sensor_tag(lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _);
const GYRO_SENSOR_ID: SensorId =
    SensorIds::GYROSCOPEI16.with_sensor_tag(l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _);
const GYRO_TEMP_SENSOR_ID: SensorId =
    SensorIds::TEMPERATUREI16.with_sensor_tag(l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _);

/// This type ensures that we store sensor data until we're ready to process them,
/// and handles the serialization to the target buffer where possible.
pub struct SensorOutBuffer {
    accelerometer: Option<AccelerometerI16>,
    accel_events: u32,
    magnetometer: Option<MagnetometerI16>,
    mag_events: u32,
    mag_temperature: Option<TemperatureI16>,
    mag_temp_events: u32,
    heading: Option<HeadingI16>,
    heading_events: u32,
    gyro: Option<GyroscopeI16>,
    gyro_events: u32,
    gyro_characteristics: Characteristics,
    gyro_temperature: Option<TemperatureI16>,
    gyro_temp_events: u32,
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
            mag_temperature: None,
            mag_temp_events: 0,
            heading: None,
            heading_events: 0,
            gyro: None,
            gyro_events: 0,
            gyro_characteristics: Characteristics::default(),
            gyro_temperature: None,
            gyro_temp_events: 0,
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

    pub fn update_mag_temp<I>(&mut self, reading: I)
    where
        I: Into<TemperatureI16>,
    {
        self.mag_temperature = Some(reading.into());
        self.mag_temp_events = self.mag_temp_events.wrapping_add(1);
    }

    pub fn update_gyro<I>(&mut self, reading: I)
    where
        I: Into<GyroscopeI16>,
    {
        self.gyro = Some(reading.into());
        self.gyro_events = self.gyro_events.wrapping_add(1);
    }

    pub fn update_gyro_characteristics(&mut self, characteristics: Characteristics) {
        self.gyro_characteristics = characteristics;
    }

    pub fn update_gyro_temp<I>(&mut self, reading: I)
    where
        I: Into<TemperatureI16>,
    {
        self.gyro_temperature = Some(reading.into());
        self.gyro_temp_events = self.gyro_temp_events.wrapping_add(1);
    }

    pub fn update_heading<I>(&mut self, reading: I)
    where
        I: Into<HeadingI16>,
    {
        self.heading = Some(reading.into());
        self.heading_events = self.heading_events.wrapping_add(1);
    }

    pub fn is_empty(&self) -> bool {
        self.accelerometer.is_none()
            && self.magnetometer.is_none()
            && self.mag_temperature.is_none()
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
        let frame = if let Some(gyro) = self.gyro.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.gyro_events,
                l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                gyro,
            ))
        } else if let Some(accelerometer) = self.accelerometer.take() {
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
        } else if let Some(temperature) = self.mag_temperature.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.mag_temp_events,
                lsm303dlhc_registers::mag::DEFAULT_DEVICE_ADDRESS as _,
                temperature,
            ))
        } else if let Some(temperature) = self.gyro_temperature.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.gyro_temp_events,
                l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                temperature,
            ))
        } else if let Some(heading) = self.heading.take() {
            self.increment_total_events();
            Some(Version1DataFrame::new(
                self.total_events,
                self.heading_events,
                0x01,
                heading,
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
                            "LSM303DLHC",
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
                            resolution_bits: 12,
                            scale: 16384,
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
                            "LSM303DLHC",
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
                            resolution_bits: 12,
                            scale: 1100,
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
                            MAG_TEMP_SENSOR_ID,
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
                            MAG_TEMP_SENSOR_ID,
                            IdentifierCode::Product,
                            "LSM303DLHC",
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
                            target: MAG_TEMP_SENSOR_ID,
                            resolution_bits: 12,
                            scale: 8,
                            offset: 20,
                            ..Default::default()
                        }),
                    ))
                }
                // Gyroscope
                12 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            GYRO_SENSOR_ID,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                13 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            GYRO_SENSOR_ID,
                            IdentifierCode::Product,
                            "L3GD20",
                        )),
                    ))
                }
                14 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        LinearRangeInfo::new(LinearRanges {
                            target: GYRO_SENSOR_ID,
                            resolution_bits: 16,
                            scale: (self.gyro_characteristics.sensitivity.recip() * 100_000.0)
                                as i32,
                            scale_decimals: 6,
                            ..Default::default()
                        }),
                    ))
                }
                // Gyroscope Temperature
                15 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            GYRO_TEMP_SENSOR_ID,
                            IdentifierCode::Maker,
                            "STMicroelectronics",
                        )),
                    ))
                }
                16 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        Identification::new(Identifier::new(
                            GYRO_TEMP_SENSOR_ID,
                            IdentifierCode::Product,
                            "L3GD20",
                        )),
                    ))
                }
                17 => {
                    self.increment_total_events();
                    self.remaining_identifiers += 1;
                    Some(Version1DataFrame::new(
                        self.total_events,
                        0,
                        l3gd20_registers::DEFAULT_DEVICE_ADDRESS as _,
                        LinearRangeInfo::new(LinearRanges {
                            target: GYRO_TEMP_SENSOR_ID,
                            resolution_bits: 8,
                            scale: 1,
                            scale_decimals: 0,
                            offset: 20, // TODO: Super empirical.
                            ..Default::default()
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
            Some(frame) => match serialize(frame, &mut self.transmit_buffer) {
                Ok(range) => {
                    self.write_remaining = range.clone();
                    true
                }
                Err(_err) => {
                    defmt::error!("A serialization error occurred");
                    false
                }
            },
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
        defmt::trace!(
            "Committing read of {} bytes, range now {}",
            bytes_read,
            self.write_remaining
        );
        self.write_remaining.len()
    }
}
