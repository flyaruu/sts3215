use ratatui::{prelude::*, widgets::*};

use crate::lerobot::robot::ServoState;

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
