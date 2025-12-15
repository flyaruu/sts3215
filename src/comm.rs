use std::io::{Read, Write};

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
pub const TEMPERATURE_REGISTER: u8 = 0x3f;
pub const MOVING_REGISTER: u8 = 0x41;
pub const ERROR_REGISTER: u8 = 0x42;

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
        let inverted = !counter & 0xff;
        inverted
    }

    pub(crate) fn send_command<'a, P: Write + Read>(
        &self,
        mut port: P,
        buffer: &'a mut [u8],
    ) -> Result<CommandResponse<'a>, ServoError> {
        let index = self.write_buffer(buffer);
        port.write_all(&buffer[..index])
            .map_err(|e| ServoError::WriteError(e))?;
        // info!("Command buffer: {:?}", &buffer[..index]);
        let read_count = port.read(buffer).map_err(|e| ServoError::WriteError(e))?;
        let response = CommandResponse::parse_response(&buffer[..read_count]).unwrap();
        Ok(response)
    }
}

#[derive(Debug)]
pub(crate) struct CommandResponse<'a> {
    _id: u8,
    status: u8,
    data: &'a [u8],
}

impl<'a> CommandResponse<'a> {
    fn parse_response(buffer: &'a [u8]) -> Option<CommandResponse<'a>> {
        // info!("Parsing response buffer: {:x?}", buffer);
        if buffer[0] != 0xFF || buffer[1] != 0xFF {
            info!("Invalid header");
            return None; // Invalid header
        }

        let id = buffer[2];
        let length = buffer[3] as usize;
        let status = buffer[4];
        let checksum = buffer[3 + length];

        let calculated_checksum = Command::calculate_checksum(buffer, length);

        if !calculated_checksum == checksum {
            info!("Checksum mismatch");
            return None; // Checksum mismatch
        }
        let data = &buffer[5..5 + length - 2];
        Some(Self { _id: id, status, data })
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
            // let low = self.data[0] as u16;
            // let high = self.data[1] as u16;
            // Some((high << 8) | low)
        } else {
            None
        }
    }

    pub(crate) fn data_as_u8(&self) -> Option<u8> {
        if self.data.len() >= 1 {
            Some(self.data[0])
            // let low = self.data[0] as u16;
            // let high = self.data[1] as u16;
            // Some((high << 8) | low)
        } else {
            None
        }
    }

    pub(crate) fn id(&self) -> u8 {
        self._id
    }
}

pub fn send_ping<'a, P: Write + Read>(
    port: &mut P,
    buffer: &'a mut [u8],
    servo_id: u8,
) -> Result<CommandResponse<'a>, ServoError> {
    Command::Ping(servo_id)
        .send_command(port, buffer)
        .map_err(|e| e.into())
}

pub fn write_position<'a, P: Write + Read>(
    port: &mut P,
    buffer: &'a mut [u8],
    servo_id: u8,
    position: u16,
    time: Option<u16>,
    accel: Option<u16>,
) -> Result<CommandResponse<'a>, ServoError> {
    let mut data = vec![];
    data.extend_from_slice(&position.to_le_bytes());
    if let Some(t) = time {
        data.extend_from_slice(&t.to_le_bytes());
    }
    if let Some(a) = accel {
        data.extend_from_slice(&a.to_le_bytes());
    }

    Command::Write(servo_id, GOAL_POSITION_REGISTER, &data).send_command(port, buffer)
}
