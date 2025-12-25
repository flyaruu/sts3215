use std::time::Duration;

use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

pub struct Robot {
    port: Box<dyn SerialPort>,
}

impl Robot {
    pub fn new(port_name: &str) -> Result<Self, serialport::Error> {
        let port = Self::create_servo_port(port_name)?;
        Ok(Robot { port })
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
    // Robot related methods would go here
}
