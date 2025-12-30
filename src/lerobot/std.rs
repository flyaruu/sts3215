use std::time::Duration;

use embedded_io_adapters::std::FromStd;
use log::info;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::{ServoError, lerobot::robot::Robot};

    pub(crate) fn new_std_robot(port_name: &str) ->Result<Robot<FromStd<Box<dyn SerialPort>>>, ServoError> {
        let port = create_servo_port(port_name)
            .map_err(|_e| ServoError::IOError)?;
        let rw = FromStd::new(port);
        Robot::new(rw)
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