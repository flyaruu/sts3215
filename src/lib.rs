use core::result::Result;
use std::io::{Read, Write};

use crate::comm::{
    Command, ERROR_REGISTER, LOAD_REGISTER, MOVING_REGISTER, POSITION_REGISTER, SPEED_REGISTER, TEMPERATURE_REGISTER, VOLTAGE_REGISTER, send_ping, write_position
};

mod comm;

// const REG_WRITE_ID: u8 = 0x04;
// const ACTION_ID: u8 = 0x05;
// const RESET_ID: u8 = 0x06;
// const SYNC_WRITE_ID: u8 = 0x83;
// const SYNC_READ_ID: u8 = 0x82;
// const BROADCAST_ID: u8 = 0xfe; //?

#[derive(Debug, thiserror::Error)]
pub enum ServoError {
    #[error("Serial port error: {0}")]
    SerialPortError(#[from] serialport::Error),
    #[error("Serial port write error: {0}")]
    WriteError(std::io::Error),
    #[error("Serial port read error: {0}")]
    ReadError(std::io::Error),
    #[error("Servo returned error status: {0}")]
    StatusError(u8),
}

pub fn read_temperature<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u8, ServoError> {
    read_u8_register(port, buffer, servo_id, TEMPERATURE_REGISTER)
}

pub fn read_voltage<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u8, ServoError> {
    read_u8_register(port, buffer, servo_id, VOLTAGE_REGISTER)
}

pub fn is_moving<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<bool, ServoError> {
    read_u8_register(port, buffer, servo_id, MOVING_REGISTER).map(|value| value != 0)
}

pub fn has_error<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<bool, ServoError> {
    read_u8_register(port, buffer, servo_id, ERROR_REGISTER).map(|value| value != 0)
}

pub fn read_position<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u16, ServoError> {
    read_u16_register(port, buffer, servo_id, POSITION_REGISTER)
}

pub fn read_speed<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u16, ServoError> {
    read_u16_register(port, buffer, servo_id, SPEED_REGISTER)
}

pub fn read_load<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u16, ServoError> {
    read_u16_register(port, buffer, servo_id, LOAD_REGISTER)
}

pub fn read_u8_register<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
    register_id: u8,
) -> Result<u8, ServoError> {
    let result = Command::Read(servo_id, register_id, 1).send_command(port, buffer)?;
    result
        .data_as_u8()
        .ok_or(ServoError::SerialPortError(serialport::Error::new(
            serialport::ErrorKind::InvalidInput,
            "Insufficient data length",
        )))
}

pub fn read_u16_register<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
    register_id: u8,
) -> Result<u16, ServoError> {
    let result = Command::Read(servo_id, register_id, 2).send_command(port, buffer)?;
    result
        .data_as_u16()
        .ok_or(ServoError::SerialPortError(serialport::Error::new(
            serialport::ErrorKind::InvalidInput,
            "Insufficient data length",
        )))
}

pub fn enable_torque<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<(), ServoError> {
    let torque_enable_addr = 0x30;
    let enable_value = [0x01]; // 1 to enable torque
    Command::Write(servo_id, torque_enable_addr, &enable_value)
        .send_command(port, buffer)
        .and_then(|response| response.is_error())
}

pub fn disable_torque<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<(), ServoError> {
    let torque_enable_addr = 0x30;
    let enable_value = [0x00]; // 0 to disable torque
    Command::Write(servo_id, torque_enable_addr, &enable_value)
        .send_command(port, buffer)
        .and_then(|response| response.is_error())
}


pub fn move_to_position<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
    position: u16,
    time: Option<u16>,
    accel: Option<u16>,
) -> Result<(), ServoError> {
    write_position(port, buffer, servo_id, position, time, accel)?.is_error()
}

pub fn ping_servo<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<(), ServoError> {
    send_ping(port, buffer, servo_id)?.is_error()
    
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command() {
        let cmd = Command::Ping(0x01);
        let mut buffer = [0u8; 10];
        let len = cmd.write_buffer(&mut buffer);
        println!("Command buffer: {:02X?}", &buffer[..len]);
    }
}
