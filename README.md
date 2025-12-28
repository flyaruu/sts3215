## STS3215

This is a no-std capable library to use serial sts3215 servo's

It works alright on std for a 6 servo lerobot model. The monitor example uses a ratatui text view to monitor servo id's 1-6, this is pretty convenient to check if your setup of your servos id configuration etc is correct.

The lerobot builds a general 'Robot' struct which owns a SerialPort, a buffer and a number of servo's, which makes the whole thing easier to use, but it is still in development and I'm not 100% convinced it's adding much.

The comm module does the actual reading and writing to the serial port, it uses the embedded-io traits for no-std compatibility, and converts it into std Read/Write traits using the embedded-io-adapters crate.

For reference, check [[this document|https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf]]

Note: **I haven't tried running it in no-std mode**





### Future work
- It queries all aspects of a servo separately (i.e. the read_servo_info function). I guess this is inefficient, it would be quicker to query all the relevant registers at once and parse it into a ServoState structure