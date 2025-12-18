use embedded_io::{Read, Write};



#[cfg(feature = "std")]
pub fn show_servo_info<P: Read + Write>(
    port: &mut P,
    buffer: &mut [u8]) {
        
    }
