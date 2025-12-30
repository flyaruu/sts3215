use embedded_io::{Read, Write};

use log::info;

use crate::ServoError;

const PING_ID: u8 = 0x01;
const READ_DATA_ID: u8 = 0x02;
const WRITE_DATA_ID: u8 = 0x03;

pub const GOAL_POSITION_REGISTER: u8 = 0x2A;

pub const POSITION_REGISTER: u8 = 0x38;
pub const SPEED_REGISTER: u8 = 0x3a;
pub const LOAD_REGISTER: u8 = 0x3c;
pub const VOLTAGE_REGISTER: u8 = 0x3e;
pub const TEMPERATURE_REGISTER: u8 = 0x3f; // ok
pub const STATUS_REGISTER: u8 = 0x41;
pub const MOVING_REGISTER: u8 = 0x42;
pub const CURRENT_REGISTER: u8 = 0x43;

pub(crate) enum Command<'a> {
    Ping(u8),
    Read(u8, u8, u8),
    Write(u8, u8, &'a [u8]),
}

impl<'cmd> Command<'cmd> {
    pub(crate) fn write_buffer(&self, buffer: &mut [u8]) -> usize {
        buffer[0] = 0xff;
        buffer[1] = 0xff;
        let checksum_index = match self {
            Command::Ping(servo_id) => {
                buffer[2] = *servo_id;
                buffer[3] = 0x02;
                buffer[4] = PING_ID;
                5
            }
            Command::Read(servo_id, addr, reply_length) => {
                buffer[2] = *servo_id;
                buffer[3] = 0x04;
                buffer[4] = READ_DATA_ID;
                buffer[5] = *addr;
                buffer[6] = *reply_length;
                7
            }
            Command::Write(servo_id, addr, data) => {
                buffer[2] = *servo_id;
                buffer[3] = (3 + data.len()) as u8; // length = instruction + addr + data
                buffer[4] = WRITE_DATA_ID;
                buffer[5] = *addr;
                for (i, &byte) in data.iter().enumerate() {
                    buffer[6 + i] = byte;
                }
                6 + data.len()
            }
        };
        let chk = Self::calculate_checksum(buffer, checksum_index);
        buffer[checksum_index] = chk;
        checksum_index + 1
    }

    fn calculate_checksum(buffer: &[u8], length: usize) -> u8 {
        let mut counter = 0_u8;
        for index in 2..length {
            let value = buffer[index];
            counter = counter.wrapping_add(value);
        }

        !counter
    }

    pub(crate) fn send_command<'a, P: Write + Read>(
        &self,
        mut port: P,
        buffer: &'a mut [u8],
    ) -> Result<CommandResponse<'a>, ServoError> {
        let index = self.write_buffer(buffer);
        port.write_all(&buffer[..index])
            .map_err(|_| ServoError::WriteError)?;
        info!("Command buffer: {:02x?}", &buffer[..index]);
        let read_count = port.read(buffer).map_err(|_| ServoError::ReadError)?;
        info!("Response buffer: {:02x?}", &buffer[..read_count]);
        CommandResponse::parse_response(&buffer[..read_count])
    }
}

#[derive(Debug)]
pub(crate) struct CommandResponse<'a> {
    _id: u8,
    status: u8,
    data: &'a [u8],
}

impl<'a> CommandResponse<'a> {
    fn parse_response(buffer: &'a [u8]) -> Result<CommandResponse<'a>, ServoError> {
        // info!("Parsing response buffer: {:x?}", buffer);
        if buffer[0] != 0xFF || buffer[1] != 0xFF {
            info!("Invalid header");
            return Err(ServoError::InvalidHeader(buffer[0], buffer[1])); // Invalid header
        }

        let id = buffer[2];
        let length = buffer[3] as usize;
        let status = buffer[4];
        let checksum = buffer[3 + length];

        let calculated_checksum = Command::calculate_checksum(buffer, length);

        if !calculated_checksum == checksum {
            info!("Checksum mismatch");
            return Err(ServoError::ChecksumMismatch(calculated_checksum, checksum)); // Checksum mismatch
        }

        let data = &buffer[5..5 + length - 2];
        Ok(Self {
            _id: id,
            status,
            data,
        })
    }

    pub(crate) fn is_ok(&self) -> bool {
        self.status == 0
    }

    pub(crate) fn is_error(&self) -> Result<(), ServoError> {
        if self.is_ok() {
            Ok(())
        } else {
            Err(ServoError::StatusError(self.status))
        }
    }

    pub(crate) fn data_as_u16(&self) -> Option<u16> {
        if self.data.len() >= 2 {
            Some(u16::from_le_bytes(self.data[0..2].try_into().ok()?))
        } else {
            None
        }
    }

    pub(crate) fn data_as_u8(&self) -> Option<u8> {
        if !self.data.is_empty() {
            Some(self.data[0])
        } else {
            None
        }
    }

    pub(crate) fn status(&self) -> u8 {
        self.status
    }
}

pub fn send_ping<'a, P: Write + Read>(
    port: &mut P,
    buffer: &'a mut [u8],
    servo_id: u8,
) -> Result<CommandResponse<'a>, ServoError> {
    Command::Ping(servo_id).send_command(port, buffer)
}

pub fn write_position<'a, P: Write + Read>(
    port: &mut P,
    buffer: &'a mut [u8],
    servo_id: u8,
    position: u16,
    speed: Option<u16>,
    acc: Option<u16>,
) -> Result<CommandResponse<'a>, ServoError> {
    let mut data = [0u8; 6];
    let mut len = 0;

    data[0..2].copy_from_slice(&position.to_le_bytes());
    len += 2;

    if let Some(s) = speed {
        data[len..len + 2].copy_from_slice(&s.to_le_bytes());
        len += 2;
    }
    if let Some(a) = acc {
        data[len..len + 2].copy_from_slice(&a.to_le_bytes());
        len += 2;
    }

    info!("Writing buffer to servo {}: {:02x?}", servo_id, &data[..len]);
    Command::Write(servo_id, GOAL_POSITION_REGISTER, &data[..len]).send_command(port, buffer)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_position_command_buffer() {
        // Test scenario: Write to position 2048 with speed 0 (steps/sec) and acc 1000
        // According to STS3215 docs: speed is steps/sec (50 steps/sec ≈ 0.732RPM)
        // acc is acceleration 0-150, smaller = lower acceleration
        let servo_id = 1u8;
        let position = 2048u16; // 0x0800 in hex, [0x00, 0x08] in little-endian
        let speed = 0u16;       // 0x0000 in hex, [0x00, 0x00] in little-endian
        let acc = 1000u16;      // 0x03E8 in hex, [0xE8, 0x03] in little-endian

        // Create data buffer as write_position would
        let mut data = [0u8; 6];
        data[0..2].copy_from_slice(&position.to_le_bytes());
        data[2..4].copy_from_slice(&speed.to_le_bytes());
        data[4..6].copy_from_slice(&acc.to_le_bytes());

        // Create the command
        let cmd = Command::Write(servo_id, GOAL_POSITION_REGISTER, &data);

        // Write to buffer
        let mut buffer = [0u8; 256];
        let length = cmd.write_buffer(&mut buffer);

        // Assertions on the buffer
        assert_eq!(buffer[0], 0xFF, "Header byte 1 should be 0xFF");
        assert_eq!(buffer[1], 0xFF, "Header byte 2 should be 0xFF");
        assert_eq!(buffer[2], servo_id, "Servo ID should be {}", servo_id);
        assert_eq!(buffer[3], 9, "Length should be 9 (3 + 6 data bytes)");
        assert_eq!(buffer[4], WRITE_DATA_ID, "Command ID should be WRITE_DATA_ID");
        assert_eq!(buffer[5], GOAL_POSITION_REGISTER, "Register should be GOAL_POSITION_REGISTER");
        
        // Position bytes (little-endian: 2048 = 0x0800 = [0x00, 0x08])
        assert_eq!(buffer[6], 0x00, "Position low byte should be 0x00");
        assert_eq!(buffer[7], 0x08, "Position high byte should be 0x08");
        
        // Speed bytes (little-endian: 0 = [0x00, 0x00])
        assert_eq!(buffer[8], 0x00, "Speed low byte should be 0x00");
        assert_eq!(buffer[9], 0x00, "Speed high byte should be 0x00");
        
        // ACC bytes (little-endian: 1000 = 0x03E8 = [0xE8, 0x03])
        assert_eq!(buffer[10], 0xE8, "ACC low byte should be 0xE8");
        assert_eq!(buffer[11], 0x03, "ACC high byte should be 0x03");
        
        // Checksum is at buffer[12]
        // Checksum = !(ID + Length + Instruction + Addr + Data bytes)
        let expected_checksum = !(servo_id
            .wrapping_add(9)
            .wrapping_add(WRITE_DATA_ID)
            .wrapping_add(GOAL_POSITION_REGISTER)
            .wrapping_add(0x00)
            .wrapping_add(0x08)
            .wrapping_add(0x00)
            .wrapping_add(0x00)
            .wrapping_add(0xE8)
            .wrapping_add(0x03));
        assert_eq!(buffer[12], expected_checksum, "Checksum should be calculated correctly");
        
        // Total length should be 13 (header + id + length + instruction + addr + 6 data bytes + checksum)
        assert_eq!(length, 13, "Total buffer length should be 13");
    }
}
