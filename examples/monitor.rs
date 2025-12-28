use std::{
    fs::OpenOptions,
    io::{self},
    time::Duration,
};

use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use embedded_io::{Read, Write};
use embedded_io_adapters::std::FromStd;
use log::info;
use ratatui::prelude::*;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use sts3215::{info::ui, lerobot::robot::ServoState};

const MAX_BUFFER_SIZE: usize = 256;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup logging to file
    let log_file = OpenOptions::new()
        .create(true)
        .append(true)
        .open("servo_monitor.log")?;

    env_logger::Builder::from_default_env()
        .target(env_logger::Target::Pipe(Box::new(log_file)))
        .filter_level(log::LevelFilter::Info)
        .init();

    info!("Starting servo monitor");

    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut buffer = [0u8; MAX_BUFFER_SIZE];

    let serialport: Box<dyn SerialPort> = create_servo_port("/dev/cu.wchusbserial5AAF2185891")?;

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
    let servo_ids = [1u8, 2, 3, 4, 5, 6];
    let mut state = ServoState::new(&servo_ids);

    loop {
        match state.process_queued_commands(port, buffer) {
            Ok(_) => {}
            Err(e) => {
                info!("Error processing queued commands: {:?}", e);
            }
        }
        // Update servo data
        state.update(port, buffer);

        terminal.draw(|f| {
            ui(f, &state);
        })?;

        // Poll for events with a timeout
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => {
                        return Ok(());
                    }
                    KeyCode::Up => {
                        state.select_previous();
                    }
                    KeyCode::Down => {
                        state.select_next();
                    }
                    KeyCode::Left => {
                        let delta = if key.modifiers.contains(KeyModifiers::SHIFT) {
                            -200
                        } else {
                            -20
                        };
                        state.move_position(delta);
                    }
                    KeyCode::Right => {
                        let delta = if key.modifiers.contains(KeyModifiers::SHIFT) {
                            200
                        } else {
                            20
                        };
                        state.move_position(delta);
                    }
                    _ => {}
                }
            }
        }
    }
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
