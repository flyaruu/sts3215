use core::time::Duration;

use embedded_io_adapters::std::FromStd;
use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

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
    pub time: Option<u16>,
    pub accel: Option<u16>,
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
pub struct ServoState<const N: usize> {
    pub infos: [ServoInfo; N],
    pub servo_ids: [u8; N],
    pub selected_index: usize,
    pub queued_commands: Vec<ServoPositionCommand>,
}

impl<const N: usize> ServoState<N> {
    pub fn new(servo_ids: &[u8; N]) -> Self {
        Self {
            servo_ids: servo_ids.clone(),
            infos: [ServoInfo::default(); N],
            selected_index: 0,
            queued_commands: Vec::new(),
        }
    }

    pub fn update<P: Read + Write>(&mut self, port: &mut P, buffer: &mut [u8]) {
        for (index, &id) in self.servo_ids.iter().enumerate() {
            if let Ok(info) = read_servo_info(port, buffer, id) {
                self.infos[index] = info;
            }
        }
    }

    pub fn select_next(&mut self) {
        if self.selected_index < N.saturating_sub(1) {
            self.selected_index += 1;
        }
    }

    pub fn select_previous(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        }
    }

    pub fn selected_servo_id(&self) -> u8 {
        self.servo_ids[self.selected_index]
    }

    pub fn move_position(&mut self, delta: i16) {
        let id = self.selected_servo_id();
        let index = self.selected_index;
        let new_position = self.infos[index].goal_position as i16 + delta;
        self.infos[index].goal_position = new_position.rem_euclid(4096) as u16;
        info!(
            "Queued position command for servo {}: new_position={}",
            id, self.infos[index].goal_position
        );
        self.queued_commands.push(ServoPositionCommand {
            id,
            position: self.infos[index].goal_position,
            time: None,
            accel: None,
        });
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
                command.time,
                command.accel,
            )?;
            info!(
                "Sent position command to servo {}: position={}, time={:?}, accel={:?}",
                command.id, command.position, command.time, command.accel
            );
            if response.is_ok() {
                Ok(())
            } else {
                Err(ServoError::StatusError(response.status()))
            }
        } else {
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

pub struct Robot {
    port: Box<dyn SerialPort>,
    servo_state: ServoState<6>,
    buffer: [u8; 256],
}

impl Robot {
    pub fn  new(port_name: &str) -> Result<Self, serialport::Error> {
        let port = Self::create_servo_port(port_name)?;
        let servo_ids = [1u8, 2, 3, 4, 5, 6];
        let state = ServoState::new(&servo_ids);
        let buffer = [0u8; 256];
        Ok(Robot {
            port,
            buffer,
            servo_state: state,
        })
    }

    fn create_servo_port(port_name: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
        let port = serialport::new(port_name, 1_000_000)
            .timeout(Duration::from_millis(1000))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()?;

        info!("Port opened successfully: {}", port_name);
        Ok(port)
    }

    pub fn move_to_position(
        &mut self,
        servo_id: u8,
        position: u16,
        time: Option<u16>,
        accel: Option<u16>,
    ) -> Result<(), ServoError> {
        // let prt = &mut *self.port;
        let mut adapter = FromStd::new(&mut *self.port);

        write_position(
            &mut adapter,
            &mut self.buffer,
            servo_id,
            position,
            time,
            accel,
        )?
        .is_error()
    }

    pub fn ping_servo(&mut self, servo_id: u8) -> Result<(), ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        send_ping(&mut adapter, &mut self.buffer, servo_id)?.is_error()
    }

    pub fn read_temperature(&mut self, servo_id: u8) -> Result<u8, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u8_register(
            &mut adapter,
            &mut self.buffer,
            servo_id,
            TEMPERATURE_REGISTER,
        )
    }

    pub fn read_voltage(&mut self, servo_id: u8) -> Result<u8, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u8_register(&mut adapter, &mut self.buffer, servo_id, VOLTAGE_REGISTER)
    }

    pub fn read_current(&mut self, servo_id: u8) -> Result<u16, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u16_register(&mut adapter, &mut self.buffer, servo_id, CURRENT_REGISTER)
    }

    pub fn is_moving(&mut self, servo_id: u8) -> Result<bool, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u8_register(&mut adapter, &mut self.buffer, servo_id, MOVING_REGISTER)
            .map(|value| value != 0)
    }

    pub fn has_error(&mut self, servo_id: u8) -> Result<bool, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u8_register(&mut adapter, &mut self.buffer, servo_id, STATUS_REGISTER)
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
        let mut adapter = FromStd::new(&mut *self.port);
        read_u16_register(&mut adapter, &mut self.buffer, servo_id, SPEED_REGISTER)
    }

    pub fn read_load(&mut self, servo_id: u8) -> Result<u16, ServoError> {
        let mut adapter = FromStd::new(&mut *self.port);
        read_u16_register(&mut adapter, &mut self.buffer, servo_id, LOAD_REGISTER)
    }

    // Robot related methods would go here
}
