use std::{io, time::Duration};

use embedded_io::{Read, Write};
use embedded_io_adapters::std::FromStd;
use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use sts3215::{ServoError, has_error, is_moving, read_load, read_position, read_speed, read_temperature, read_voltage};
use ratatui::{prelude::*, widgets::*};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};

const MAX_BUFFER_SIZE: usize = 256;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut buffer = [0u8; MAX_BUFFER_SIZE];

    let serialport: Box<dyn SerialPort> =
        create_servo_port("/dev/cu.wchusbserial5AAF2182201")?;
    
    // Wrap the serial port with the embedded-io adapter
    let mut port = FromStd::new(serialport);

    let result = run_app(&mut terminal, &mut port, &mut buffer);

    // Restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    result
}

fn run_app<B: Backend, P: Read + Write>(
    terminal: &mut Terminal<B>,
    port: &mut P,
    buffer: &mut [u8],
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let servo_ids = [1u8, 2, 3, 4, 5, 6];
        let mut servo_info = [ServoInfo::default(); 6];
        read_servo_set(port, buffer, &servo_ids, &mut servo_info)?;

        terminal.draw(|f| {
            ui(f, &servo_info);
        })?;

        // Poll for events with a timeout
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if key.code == KeyCode::Char('q') || key.code == KeyCode::Esc {
                    return Ok(());
                }
            }
        }
    }
}

fn ui(f: &mut Frame, servo_info: &[ServoInfo]) {
    let area = f.area();

    // Create the table header
    let header = Row::new(vec![
        Cell::from("ID"),
        Cell::from("Position"),
        Cell::from("Speed"),
        Cell::from("Temp (°C)"),
        Cell::from("Load"),
        Cell::from("Voltage"),
        Cell::from("Moving"),
        Cell::from("Error"),
    ])
    .style(Style::default().fg(Color::Yellow).bold());

    // Create table rows from servo data
    let rows: Vec<Row> = servo_info
        .iter()
        .map(|info| {
            Row::new(vec![
                Cell::from(info.id.to_string()),
                Cell::from(info.position.to_string()),
                Cell::from(info.speed.to_string()),
                Cell::from(info.temperature.to_string()),
                Cell::from(info.load.to_string()),
                Cell::from(info.voltage.to_string()),
                Cell::from(if info.is_moving { "Yes" } else { "No" })
                    .style(if info.is_moving {
                        Style::default().fg(Color::Green)
                    } else {
                        Style::default().fg(Color::Gray)
                    }),
                Cell::from(if info.has_error { "Yes" } else { "No" })
                    .style(if info.has_error {
                        Style::default().fg(Color::Red)
                    } else {
                        Style::default().fg(Color::Green)
                    }),
            ])
        })
        .collect();

    // Create the table widget
    let table = Table::new(
        rows,
        vec![
            Constraint::Length(4),  // ID
            Constraint::Length(10), // Position
            Constraint::Length(8),  // Speed
            Constraint::Length(12), // Temperature
            Constraint::Length(8),  // Load
            Constraint::Length(10), // Voltage
            Constraint::Length(8),  // Moving
            Constraint::Length(8),  // Error
        ],
    )
    .header(header)
    .block(
        Block::default()
            .title("Servo Status Monitor (Press 'q' to quit)")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan)),
    )
    .style(Style::default().fg(Color::White));

    f.render_widget(table, area);
}

pub fn create_servo_port(port: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
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

#[derive(Default, Debug, Clone, Copy)]
pub struct ServoInfo {
    pub id: u8,
    pub position: u16,
    pub speed: u16,
    pub temperature: u8,
    pub load: u16,
    pub voltage: u8,
    pub is_moving: bool,
    pub has_error: bool,
}

fn read_servo_info<P: Read + Write>(port: &mut P, buffer: &mut [u8], servo_id: u8)->Result<ServoInfo,ServoError> {
    let position = read_position(port, buffer, servo_id).unwrap_or(0);
    let speed = read_speed(port, buffer, servo_id).unwrap_or(0);
    let temperature = read_temperature(port, buffer, servo_id).unwrap_or(0);
    let load = read_load(port, buffer, servo_id).unwrap_or(0);
    let voltage = read_voltage(port, buffer, servo_id).unwrap_or(0);
    let is_moving = is_moving(port, buffer, servo_id).unwrap_or(false);
    let has_error = has_error(port, buffer, servo_id).unwrap_or(true);
    Ok(ServoInfo {
        id: servo_id,
        position,
        speed,
        temperature,
        load,
        voltage,
        is_moving,
        has_error,
    })
}

pub fn read_servo_set<const N: usize, P: Read + Write>(port: &mut P, buffer: &mut [u8], servo_ids: &[u8; N], servo_info: &mut [ServoInfo; N]) -> Result<(), ServoError> {
    for (index, &id) in servo_ids.iter().enumerate() {
        servo_info[index] = read_servo_info(port, buffer, id)?;
    }
    Ok(())
}

pub fn show_servo_info<P: Read + Write>(port: &mut P, buffer: &mut [u8])->Result<(), ServoError> {
    let servo_ids = [1u8, 2, 3, 4, 5, 6];
    let mut servo_info = [ServoInfo::default(); 6];
    read_servo_set(port, buffer, &servo_ids, &mut servo_info)
}