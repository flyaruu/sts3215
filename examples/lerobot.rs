use std::fs::OpenOptions;

use embedded_io_adapters::std::FromStd;
use serialport::SerialPort;
use sts3215::{ServoError, lerobot::robot::Robot};

pub fn main() -> Result<(), ServoError> {
    let log_file = OpenOptions::new()
        .create(true)
        .append(true)
        .open("servo_monitor.log")
        .map_err(|_| ServoError::IOError)?;

    env_logger::Builder::from_default_env()
        .target(env_logger::Target::Pipe(Box::new(log_file)))
        .filter_level(log::LevelFilter::Info)
        .init();
    
    let mut leader = Robot::<FromStd<Box<dyn SerialPort>>>::new_std_robot("/dev/cu.wchusbserial5AAF2185891").unwrap();
    // let mut _follower = Robot::<FromStd<Box<dyn SerialPort>>>::new_std_robot("/dev/cu.wchusbserial5AAF2182201").unwrap();

    loop {
        leader.send_absolute_move_command(0,1200, Some(500), Some(10)).unwrap();
        leader.process_queued_commands().unwrap();
        // leader.update_servo_state().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(8000));
        leader.send_absolute_move_command(0,800, Some(500), Some(10)).unwrap();
        leader.process_queued_commands().unwrap();
        // leader.update_servo_state().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(8000));
    }
}