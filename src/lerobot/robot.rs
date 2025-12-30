use log::info;

use crate::{
    ServoError,
    comm::{
        CURRENT_REGISTER, LOAD_REGISTER, MOVING_REGISTER, POSITION_REGISTER, SPEED_REGISTER,
        STATUS_REGISTER, TEMPERATURE_REGISTER, VOLTAGE_REGISTER, send_ping, write_position,
    },
    has_error, is_moving, read_current, read_load, read_position, read_speed, read_temperature,
    read_u8_register, read_u16_register, read_voltage,
};
use embedded_io::{Read, Write};

#[derive(Default, Debug, Clone, Copy)]
pub struct ServoPositionCommand {
    pub id: u8,
    pub position: u16,
    pub speed: Option<u16>,
    pub acc: Option<u16>,
}
#[derive(Default, Debug, Clone, Copy)]
pub struct ServoInfo {
    pub id: u8,
    pub position: u16,
    pub goal_position: u16,
    pub speed: u16,
    pub temperature: u8,
    pub load: u16,
    pub voltage: u8,
    pub current: u16,
    pub is_moving: bool,
    pub has_error: bool,
}

#[derive(Debug)]
pub struct ServoState<const SERVO_COUNT: usize, const COMMAND_QUEUE_SIZE: usize = 16> {
    pub infos: [ServoInfo; SERVO_COUNT],
    pub servo_ids: [u8; SERVO_COUNT],
    pub queued_commands: heapless::Vec<ServoPositionCommand,COMMAND_QUEUE_SIZE>,
}

impl<const N: usize> ServoState<N> {
    pub fn new(servo_ids: &[u8; N]) -> Self {
        Self {
            servo_ids: servo_ids.clone(),
            infos: [ServoInfo::default(); N],
            queued_commands: heapless::Vec::new(),
        }
    }


    pub fn update<P: Read + Write>(&mut self, port: &mut P, buffer: &mut [u8]) {
        for (index, &id) in self.servo_ids.iter().enumerate() {
            if let Ok(info) = read_servo_info(port, buffer, id) {
                self.infos[index] = info;
            }
        }
    }

    pub fn send_absolute_move_command(&mut self, servo_index: u8, position: u16, speed: Option<u16>, acc: Option<u16>)->Result<(), ServoError> {
        let servo_id = self.servo_ids[servo_index as usize];
        self.infos[servo_index as usize].goal_position = position;
        self.queued_commands.push(ServoPositionCommand {
            id: servo_id,
            position,
            speed,
            acc,
        }).map_err(|_| ServoError::CommandOverflow)        
    }
    pub fn send_relative_move_command(&mut self, servo_index: u8, delta: i16, speed: Option<u16>, acc: Option<u16>)->Result<(), ServoError> {
        let servo_id = self.servo_ids[servo_index as usize];
        let new_position = self.infos[servo_index as usize].goal_position as i16 + delta;
        self.infos[servo_index as usize].goal_position = new_position.rem_euclid(4096) as u16;
        info!(
            "Queued position command for servo {}: new_position={}",
            servo_id, self.infos[servo_index as usize].goal_position
        );
        self.queued_commands.push(ServoPositionCommand {
            id: servo_id,
            position: self.infos[servo_index as usize].goal_position,
            speed: speed,
            acc: acc,
        }).map_err(|_| ServoError::CommandOverflow)
    }

    pub fn process_queued_commands<P: Read + Write>(
        &mut self,
        port: &mut P,
        buffer: &mut [u8],
    ) -> Result<(), ServoError> {
        if let Some(command) = self.queued_commands.pop() {
            let response = write_position(
                port,
                buffer,
                command.id,
                command.position,
                command.speed,
                command.acc,
            )?;
            info!(
                "Sent position command to servo {}: position={}, speed={:?}, acc={:?}",
                command.id, command.position, command.speed, command.acc
            );
            if response.is_ok() {
                Ok(())
            } else {
                Err(ServoError::StatusError(response.status()))
            }
        } else {
            info!("No queued commands to process.");
            Ok(())
        }
    }

    // pub fn read_servo_set<const N: usize, P: Read + Write>(
    //     port: &mut P,
    //     buffer: &mut [u8],
    //     servo_ids: &[u8; N],
    //     servo_info: &mut [ServoInfo; N],
    // ) -> Result<(), ServoError> {
    //     for (index, &id) in servo_ids.iter().enumerate() {
    //         servo_info[index] = read_servo_info(port, buffer, id)?;
    //     }
    //     Ok(())
    // }
}

fn read_servo_info<P: Read + Write>(
    port: &mut P,
    buffer: &mut [u8],
    servo_id: u8,
) -> Result<ServoInfo, ServoError> {
    let position = read_position(port, buffer, servo_id).unwrap_or(0);
    let speed = read_speed(port, buffer, servo_id).unwrap_or(0);
    let temperature = read_temperature(port, buffer, servo_id).unwrap_or(0);
    let load = read_load(port, buffer, servo_id).unwrap_or(0);
    let voltage = read_voltage(port, buffer, servo_id).unwrap_or(0);
    let current = read_current(port, buffer, servo_id).unwrap_or(0);
    let is_moving = is_moving(port, buffer, servo_id).unwrap_or(false);
    let has_error = has_error(port, buffer, servo_id).unwrap_or(true);
    Ok(ServoInfo {
        id: servo_id,
        position,
        goal_position: position,
        speed,
        temperature,
        load,
        voltage,
        current,
        is_moving,
        has_error,
    })
}

pub struct Robot<PORT: Read + Write> {
    port: PORT,
    servo_state: ServoState<6>,
    buffer: [u8; 256],
}

impl <PORT: Read + Write>Robot<PORT> {
    pub fn  new(port: PORT) -> Result<Self, ServoError> {
        let servo_ids = [1u8, 2, 3, 4, 5, 6];
        let state = ServoState::new(&servo_ids);
        let buffer = [0u8; 256];
        Ok(Robot {
            port,
            buffer,
            servo_state: state,
        })
    }

    #[cfg(feature = "std")]
    pub fn new_std_robot(port_name: &str) ->Result<Robot<embedded_io_adapters::std::FromStd<Box<dyn serialport::SerialPort>>>, ServoError> {
        super::std::new_std_robot(port_name)
    }

    pub fn send_absolute_move_command(&mut self, servo_index: u8, position: u16, time: Option<u16>, accel: Option<u16>)->Result<(), ServoError> {
        self.servo_state.send_absolute_move_command(servo_index, position, time, accel)
    }
    pub fn send_relative_move_command(&mut self, servo_id: u8, delta: i16, time: Option<u16>, accel: Option<u16>)->Result<(), ServoError> {
        self.servo_state.send_relative_move_command(servo_id, delta, time, accel)
    }

    pub fn process_queued_commands(
        &mut self,
    ) -> Result<(), ServoError> {
        self.servo_state.process_queued_commands(&mut self.port, &mut self.buffer)
    }

    pub fn update_servo_state(&mut self)->Result<(),ServoError> {
        self.servo_state.update(&mut self.port, &mut self.buffer);
        Ok(())
    }
    
    pub fn servo_state(&self) -> &ServoState<6> {
        &self.servo_state
    }

    pub fn move_to_position(
        &mut self,
        servo_id: u8,
        position: u16,
        speed: Option<u16>,
        acc: Option<u16>,
    ) -> Result<(), ServoError> {
        // let prt = &mut *self.port;

        write_position(
            &mut self.port,
            &mut self.buffer,
            servo_id,
            position,
            speed,
            acc,
        )?
        .is_error()
    }

    pub fn ping_servo(&mut self, servo_id: u8) -> Result<(), ServoError> {
        send_ping(&mut self.port, &mut self.buffer, servo_id)?.is_error()
    }

    pub fn read_temperature(&mut self, servo_id: u8) -> Result<u8, ServoError> {
        read_u8_register(
            &mut self.port,
            &mut self.buffer,
            servo_id,
            TEMPERATURE_REGISTER,
        )
    }

    pub fn read_voltage(&mut self, servo_id: u8) -> Result<u8, ServoError> {
        read_u8_register(&mut self.port, &mut self.buffer, servo_id, VOLTAGE_REGISTER)
    }

    pub fn read_current(&mut self, servo_id: u8) -> Result<u16, ServoError> {
        read_u16_register(&mut self.port, &mut self.buffer, servo_id, CURRENT_REGISTER)
    }

    pub fn is_moving(&mut self, servo_id: u8) -> Result<bool, ServoError> {
        read_u8_register(&mut self.port, &mut self.buffer, servo_id, MOVING_REGISTER)
            .map(|value| value != 0)
    }

    pub fn has_error(&mut self, servo_id: u8) -> Result<bool, ServoError> {
        read_u8_register(&mut self.port, &mut self.buffer, servo_id, STATUS_REGISTER)
            .map(|value| value != 0)
    }

    pub fn read_position<P: Write + Read>(
        port: &mut P,
        buffer: &mut [u8],
        servo_id: u8,
    ) -> Result<u16, ServoError> {
        read_u16_register(port, buffer, servo_id, POSITION_REGISTER)
    }

    pub fn read_speed(&mut self, servo_id: u8) -> Result<u16, ServoError> {
        read_u16_register(&mut self.port, &mut self.buffer, servo_id, SPEED_REGISTER)
    }

    pub fn read_load(&mut self, servo_id: u8) -> Result<u16, ServoError> {
        read_u16_register(&mut self.port, &mut self.buffer, servo_id, LOAD_REGISTER)
    }

    // Robot related methods would go here
}
