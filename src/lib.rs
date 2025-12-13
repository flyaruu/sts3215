use core::result::Result;
use std::{io::{Read, Write}, time::Duration};


use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};


pub fn create_servo_port(port: &str)->Result<Box<dyn SerialPort>, serialport::Error> {
    let port = serialport::new(port, 1_000_000)
        .timeout(Duration::from_millis(1000))
        .data_bits(DataBits::Eight)
        .stop_bits(StopBits::One)
        .parity(Parity::None)
        .flow_control(FlowControl::None)
        .open()?;
    
    info!("Port opened successfully");
    Ok(port)
}

const PING_ID: u8 = 0x01;
const READ_DATA_ID: u8 = 0x02;
const WRITE_DATA_ID: u8 = 0x03;
const REG_WRITE_ID: u8 = 0x04;
const ACTION_ID: u8 = 0x05;
const RESET_ID: u8 = 0x06;
const SYNC_WRITE_ID: u8 = 0x83;
const SYNC_READ_ID: u8 = 0x82;
const BROADCAST_ID: u8 = 0xfe; //?

pub const GOAL_POSITION_REGISTER: u8 = 0x2A;
pub const POSITION_REGISTER: u8 = 0x38;
pub const SPEED_REGISTER: u8 = 0x3a;
pub const LOAD_REGISTER: u8 = 0x3c;
pub const VOLTAGE_REGISTER: u8 = 0x3e;
pub const TEMPERATURE_REGISTER: u8 = 0x3f;


#[derive(Debug, thiserror::Error, )]
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

pub enum Command<'a> {
    Ping(u8),
    Read(u8, u8, u8),
    Write(u8, u8, &'a [u8]),
}

impl<'cmd> Command<'cmd> {
    pub fn write_buffer(&self, buffer: &mut [u8]) -> usize {
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
                info!("Read command - addr: {:x}, length: {}", addr, reply_length);
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
                info!("Write command - addr: {:x}, data: {:x?}", addr, data);
                6 + data.len()
            }
        };
        let chk = Self::calculate_checksum(buffer, checksum_index);
        buffer[checksum_index] = chk;
        checksum_index + 1
    }


    fn calculate_checksum(buffer: &[u8], length: usize)-> u8 {
        let mut counter = 0_u8;
        for index in 2..length {
            let value = buffer[index];
            counter = counter.wrapping_add(value);
        }
        let inverted = !counter & 0xff;
        inverted
   }


    pub fn send_command<'a, P: Write + Read>(&self, mut port: P, buffer: &'a mut [u8]) -> Result<CommandResponse<'a>, ServoError> {
        let index = self.write_buffer(buffer);
        port.write_all(&buffer[..index]).map_err(|e| ServoError::WriteError(e))?;
        info!("Command buffer: {:?}", &buffer[..index]);
        let read_count = port.read(buffer).map_err(|e| ServoError::WriteError(e))?;
        let response = CommandResponse::parse_response(&buffer[..read_count]).unwrap();
        Ok(response)
    }
}


    pub fn send_ping<'a>(port: impl Write + Read, buffer: &'a mut [u8], servo_id: u8) -> Result<CommandResponse<'a>, ServoError> {
        Command::Ping(servo_id).send_command(port, buffer).map_err(|e| e.into())
    }

    pub fn read_u16_value<'a>(port: impl Write + Read, buffer: &'a mut [u8], servo_id: u8, register: u8, length: u8) -> Result<u16, ServoError> {
        let result = Command::Read(servo_id, register, length).send_command(port, buffer)?;
        result.data_as_u16().ok_or(ServoError::SerialPortError(serialport::Error::new(
            serialport::ErrorKind::InvalidInput,
            "Insufficient data length",
        )))
    }

    pub fn enable_torque<'a>(port: impl Write + Read, buffer: &'a mut [u8], servo_id: u8) -> Result<(), ServoError> {
        let torque_enable_addr = 0x30;
        let enable_value = [0x01]; // 1 to enable torque
        Command::Write(servo_id, torque_enable_addr, &enable_value).send_command(port, buffer).and_then(|response| response.is_error())
    }


    pub fn disable_torque<'a>(port: impl Write + Read, buffer: &'a mut [u8], servo_id: u8) -> Result<(), ServoError> {
        let torque_enable_addr = 0x30;
        let enable_value = [0x00]; // 0 to disable torque
        Command::Write(servo_id, torque_enable_addr, &enable_value).send_command(port, buffer).and_then(|response| response.is_error())
    }


    pub fn write_position<'a>(port: impl Write + Read, buffer: &'a mut [u8], servo_id: u8, position: u16, time: Option<u16>, accel: Option<u16>) -> Result<CommandResponse<'a>, ServoError> {
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


#[derive(Debug)]
pub struct CommandResponse<'a> {
    id: u8,
    status: u8,
    data: &'a [u8],
}

impl<'a> CommandResponse<'a> {
    fn parse_response(buffer: &'a [u8]) -> Option<CommandResponse<'a>> {
        info!("Parsing response buffer: {:x?}", buffer);
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
        info!("Checksum valid");
        let data = &buffer[5..5 + length - 2];
        Some(Self {
            id,
            status,
            data,
        })

   }
 
    pub fn is_ok(&self) -> bool {
        self.status == 0
    }

    pub fn is_error(&self) -> Result<(), ServoError> {
        if self.is_ok()    {
            Ok(())
        } else {
            Err(ServoError::StatusError(self.status))
        }
    }

    pub fn data_as_u16(&self) -> Option<u16> {

        if self.data.len() >= 2 {
            Some(u16::from_le_bytes(self.data[0..2].try_into().ok()?))
            // let low = self.data[0] as u16;
            // let high = self.data[1] as u16;
            // Some((high << 8) | low)
        } else {
            None
        }
    }

    pub fn id(&self) -> u8 {
        self.id 
    }
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
