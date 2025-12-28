#![cfg_attr(not(feature = "std"), no_std)]

use embedded_io::{Read, Write};

use crate::comm::{
    CURRENT_REGISTER, Command, LOAD_REGISTER, MOVING_REGISTER, POSITION_REGISTER, SPEED_REGISTER,
    STATUS_REGISTER, TEMPERATURE_REGISTER, VOLTAGE_REGISTER, send_ping, write_position,
};

mod comm;

#[cfg(feature = "ui")]
pub mod info;

pub mod lerobot;

// const REG_WRITE_ID: u8 = 0x04;
// const ACTION_ID: u8 = 0x05;
// const RESET_ID: u8 = 0x06;
// const SYNC_WRITE_ID: u8 = 0x83;
// const SYNC_READ_ID: u8 = 0x82;
// const BROADCAST_ID: u8 = 0xfe; //?

#[derive(Debug, thiserror::Error)]
pub enum ServoError {
    #[error("Serial port write error")]
    WriteError,
    #[error("Serial port read error")]
    ReadError,
    #[error("Servo returned error status: {0}")]
    StatusError(u8),
    #[error("Failed to parse servo response")]
    ResponseParseError,
    #[error("Invalid header bytes: {0:#X}, {1:#X}")]
    InvalidHeader(u8, u8),
    #[error("Checksum mismatch: calculated {0:#X}, received {1:#X}")]
    ChecksumMismatch(u8, u8),
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

pub fn read_current<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<u16, ServoError> {
    read_u16_register(port, buffer, servo_id, CURRENT_REGISTER)
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
    read_u8_register(port, buffer, servo_id, STATUS_REGISTER).map(|value| value != 0)
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
    result.data_as_u8().ok_or(ServoError::ReadError)
}

pub fn read_u16_register<P: Write + Read>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
    register_id: u8,
) -> Result<u16, ServoError> {
    let result = Command::Read(servo_id, register_id, 2).send_command(port, buffer)?;
    result.data_as_u16().ok_or(ServoError::ReadError)
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
