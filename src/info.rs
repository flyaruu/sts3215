use crate::{
    ServoError, comm::write_position, has_error, is_moving, read_current, read_load, read_position,
    read_speed, read_temperature, read_voltage,
};
use embedded_io::{Read, Write};
use log::info;

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

use ratatui::{prelude::*, widgets::*};

pub fn ui<const N: usize>(f: &mut Frame, servo_state: &ServoState<N>) {
    let area = f.area();

    // Create the table header
    let header = Row::new(vec![
        Cell::from("ID"),
        Cell::from("Position"),
        Cell::from("Goal Position"),
        Cell::from("Speed"),
        Cell::from("Temp (°C)"),
        Cell::from("Load"),
        Cell::from("Voltage"),
        Cell::from("Current"),
        Cell::from("Moving"),
        Cell::from("Error"),
    ])
    .style(Style::default().fg(Color::Yellow).bold());

    // Create table rows from servo data with selection highlighting
    let rows: Vec<Row> = servo_state
        .infos
        .iter()
        .enumerate()
        .map(|(index, info)| {
            let is_selected = index == servo_state.selected_index;
            let row = Row::new(vec![
                Cell::from(info.id.to_string()),
                Cell::from(info.position.to_string()),
                Cell::from(info.goal_position.to_string()),
                Cell::from(info.speed.to_string()),
                Cell::from(info.temperature.to_string()),
                Cell::from(info.load.to_string()),
                Cell::from(info.voltage.to_string()),
                Cell::from(info.current.to_string()),
                Cell::from(if info.is_moving { "Yes" } else { "No" }).style(if info.is_moving {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default().fg(Color::Gray)
                }),
                Cell::from(if info.has_error { "Yes" } else { "No" }).style(if info.has_error {
                    Style::default().fg(Color::Red)
                } else {
                    Style::default().fg(Color::Green)
                }),
            ]);

            // Apply selection highlighting
            if is_selected {
                row.style(Style::default().bg(Color::DarkGray).fg(Color::White))
            } else {
                row
            }
        })
        .collect();

    // Create the table widget
    let table = Table::new(
        rows,
        vec![
            Constraint::Length(4),  // ID
            Constraint::Length(10), // Position
            Constraint::Length(10), // Goal Position
            Constraint::Length(8),  // Speed
            Constraint::Length(12), // Temperature
            Constraint::Length(8),  // Load
            Constraint::Length(10), // Voltage
            Constraint::Length(10), // Voltage
            Constraint::Length(8),  // Moving
            Constraint::Length(8),  // Error
        ],
    )
    .header(header)
    .block(
        Block::default()
            .title("Servo Status Monitor (↑↓: Select, q: Quit)")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan)),
    )
    .style(Style::default().fg(Color::White));

    f.render_widget(table, area);
}

#[cfg(test)]
mod tests {
    use crate::info::ServoState;

    #[test]
    fn it_works() {
        let mut state = ServoState::new(&[1]);
        state.infos[0].position = 100;
        state.move_position(-90);
        assert_eq!(state.infos[0].position, 10);
        state.move_position(-20);
        assert_eq!(state.infos[0].position, 4086);
        state.move_position(20);
        assert_eq!(state.infos[0].position, 10);
    }
}
