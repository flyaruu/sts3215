use std::{thread::sleep, time::Duration};

use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use sts3215::{read_position, read_speed, read_temperature, read_voltage};

const MAX_BUFFER_SIZE: usize = 256;

pub fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let mut buffer = [0u8; MAX_BUFFER_SIZE];

    let mut serialport: Box<dyn SerialPort> =
        create_servo_port("/dev/cu.wchusbserial5AAF2182201").unwrap();
    // let a = Command::Ping(0x01).send_command(&mut serialport, &mut buffer).unwrap();
    // info!("Received response: {:?}", a);

    // enable_torque(&mut *serialport, &mut buffer, 1).unwrap();
    // write_position(&mut *serialport, &mut buffer, 1, 3500, Some(7000), None).unwrap();

    loop {
        // write_position(&mut *serialport, &mut buffer, 3, 2000, Some(7000), None).unwrap();
        // sleep(Duration::from_millis(1500));
        // write_position(&mut *serialport, &mut buffer, 3, 3000, Some(7000), None).unwrap();
        // sleep(Duration::from_millis(1500));

        sleep(Duration::from_millis(100));
        // let position =read_position(&mut serialport, &mut buffer, 3).unwrap();
        // let b = Command::Read(3, POSITION_REGISTER, 2).send_command(&mut serialport, &mut buffer).unwrap();
        read_servo_info(&mut serialport, &mut buffer, 3);
        // info!("Received response: {:?}", position);
    }
    // serialport.flush().unwrap();
    // sleep(Duration::from_secs(1));
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

fn read_servo_info(port: &mut Box<dyn SerialPort>, buffer: &mut [u8], servo_id: u8) {
    let position = read_position(port, buffer, servo_id).unwrap();
    let speed = read_speed(port, buffer, servo_id).unwrap();

    let temperature = read_temperature(port, buffer, servo_id).unwrap();
    let load = sts3215::read_load(port, buffer, servo_id).unwrap();
    let voltage = read_voltage(port, buffer, servo_id).unwrap();
    let is_moving = sts3215::is_moving(port, buffer, servo_id).unwrap();
    let has_error = sts3215::has_error(port, buffer, servo_id).unwrap();
    info!(
        "Position: {}, Speed: {}, Temperature: {}, Load: {}, Voltage: {}, Moving: {}, Error: {}",
        position, speed, temperature, load, voltage, is_moving, has_error
    );
    // let position = read_position(port, buffer, servo_id).unwrap();
}
