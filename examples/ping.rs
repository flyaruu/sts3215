use std::{thread::sleep, time::Duration};

use log::info;
use serialport::SerialPort;
use sts3215::{Command, POSITION_REGISTER, create_servo_port, enable_torque, write_position};

const MAX_BUFFER_SIZE : usize = 256;

pub fn main() {
    env_logger::builder().filter_level(log::LevelFilter::Info).init();

    let mut buffer = [0u8; MAX_BUFFER_SIZE];

    let mut serialport: Box<dyn SerialPort> = create_servo_port("/dev/ttyACM0").unwrap();
    // let a = Command::Ping(0x01).send_command(&mut serialport, &mut buffer).unwrap();
    // info!("Received response: {:?}", a);
        enable_torque(&mut *serialport, &mut buffer, 1).unwrap();
        write_position(&mut *serialport, &mut buffer, 1, 2500, Some(7000), None).unwrap();
    
    loop {

        
        let b = Command::Read(1, POSITION_REGISTER, 2).send_command(&mut serialport, &mut buffer).unwrap();
        info!("Received response: {:?}", b.data_as_u16().unwrap());
        sleep(Duration::from_millis(100));
    }
    // serialport.flush().unwrap();
    // sleep(Duration::from_secs(1));

}
