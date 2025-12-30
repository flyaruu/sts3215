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
use sts3215::{ServoError, info::render_tui, lerobot::robot::Robot};

pub fn main() -> Result<(), ServoError> {
    // Setup logging to file
    let log_file = OpenOptions::new()
        .create(true)
        .append(true)
        .open("servo_monitor.log")
        .map_err(|_| ServoError::IOError)?;

    env_logger::Builder::from_default_env()
        .target(env_logger::Target::Pipe(Box::new(log_file)))
        .filter_level(log::LevelFilter::Info)
        .init();

    info!("Starting servo monitor");

    // Setup terminal
    enable_raw_mode()
        .map_err(|_e| ServoError::IOError)?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)
        .map_err(|_e| ServoError::IOError)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)
        .map_err(|_e| ServoError::IOError)?;


    // let mut robot = Robot::<FromStd<Box<dyn SerialPort>>>::new_std_robot("/dev/cu.wchusbserial5AAF2185891")?;
    let mut robot = Robot::<FromStd<Box<dyn SerialPort>>>::new_std_robot("/dev/cu.wchusbserial5AAF2185891").unwrap();

    let result = run_app::<_, FromStd<Box<dyn SerialPort>>>(&mut terminal, &mut robot);

    // Restore terminal
    disable_raw_mode()
        .map_err(|_e| ServoError::IOError)?;

    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    ).map_err(|_e| ServoError::IOError)?;

    terminal.show_cursor()
        .map_err(|_e| ServoError::IOError)?;


    result
}

fn run_app<B: Backend, P: Read + Write>(
    terminal: &mut Terminal<B>,
    robot: &mut Robot<FromStd<Box<dyn SerialPort>>>,
) -> Result<(), ServoError> {
    // let servo_ids = [1u8, 2, 3, 4, 5, 6];

    // let robot = Robot::new(port);
    // let mut state = ServoState::new(&servo_ids);

    let mut selected_servo_index: usize = 0;

    loop {
        match robot.process_queued_commands() {
            Ok(_) => {}
            Err(e) => {
                info!("Error processing queued commands: {:?}", e);
            }
        }
        robot.update_servo_state()?;
        // Update servo data

        terminal.draw(|f| {
            render_tui::<_>(f, &robot, selected_servo_index);
        }).map_err(|_| ServoError::IOError).unwrap();

        // Poll for events with a timeout
        if event::poll(Duration::from_millis(100)).map_err(|_| ServoError::IOError)? {
            let state = robot.servo_state();
            if let Event::Key(key) = event::read().map_err(|_| ServoError::IOError)? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => {
                        return Ok(());
                    }
                    KeyCode::Up => {
                        select_previous(&mut selected_servo_index);
                    }
                    KeyCode::Down => {
                        select_next(&mut selected_servo_index, state.infos.len());
                    }
                    KeyCode::Left => {
                        let delta = if key.modifiers.contains(KeyModifiers::SHIFT) {
                            -200
                        } else {
                            -20
                        };
                        robot.send_relative_move_command(selected_servo_index as u8, delta, None, None).unwrap();
                    }
                    KeyCode::Right => {
                        let delta = if key.modifiers.contains(KeyModifiers::SHIFT) {
                            200
                        } else {
                            20
                        };
                        robot.send_relative_move_command(selected_servo_index as u8, delta, None, None).unwrap();
                    }
                    _ => {}
                }
            }
        }
    }
}



fn select_next(selected_index: &mut usize, servo_count: usize) {
    if *selected_index < servo_count.saturating_sub(1) {
        *selected_index += 1;
    }
}

pub fn select_previous(selected_index: &mut usize) {
    if *selected_index > 0 {
        *selected_index -= 1;
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
