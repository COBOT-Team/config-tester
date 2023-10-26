//! # Binary Protocol
//!
//! Each message begins with a 3-byte header followed by a payload.
//!
//! | Byte | Description                |
//! | ---- | -------------------------- |
//! | 0    | Start byte (0x24)          |
//! | 1    | Payload length             |
//! | 2    | CRC of payload (crc8ccitt) |
//! | 3... | Payload                    |
//!
//! ## Outgoing Message Payloads
//!
//! ### Log
//!
//! | Byte | Description    |
//! | ---- | -------------- |
//! | 0    | 0x00 (log)     |
//! | 1    | Log level      |
//! | 2    | Message length |
//! | 3... | Message        |
//!
//! ### Response
//!
//! | Byte | Description      |
//! | ---- | ---------------- |
//! | 0    | 0x01 (response)  |
//! | 1    | Response type    |
//! | 2-5  | Command ID       |
//! | 6... | Response payload |
//!
//! #### Ack Response
//!
//! No payload
//!
//! #### Done Response
//!
//! No payload
//!
//! ### Error Response
//!
//! | Byte | Description          |
//! | ---- | -------------------- |
//! | 0    | Error code           |
//! | 1    | Error message length |
//! | 2... | Error message        |
//!
//! #### Joints Response
//!
//! | Byte    | Description                              |
//! | ------- | ---------------------------------------- |
//! | 0       | Number of joints                         |
//! | N + 1-4 | Joint N angle (int32) (deg \* 10^-3)     |
//! | N + 5-8 | Joint N speed (int32) (deg \* 10^-3) / s |
//!
//! ## Incoming Message Payloads
//!
//! | Byte | Description  |
//! | ---- | ------------ |
//! | 0    | Request type |
//! | 1-4  | Command ID   |
//!
//! ### Init
//!
//! | Byte  | Description               |
//! | ----- | ------------------------- |
//! | 0 - 3 | Expected firmware version |
//!
//! ### Calibrate
//!
//! | Byte | Description                     |
//! | ---- | ------------------------------- |
//! | 0    | Bitfield of joints to calibrate |
//!
//! ### Override
//!
//! | Byte    | Description                      |
//! | ------- | -------------------------------- |
//! | N + 0   | Joint ID                         |
//! | N + 1-4 | New angle (int32) (deg \* 10^-3) |
//!
//! ### Get Joints
//!
//! No payload
//!
//! ### Move To
//!
//! | Byte    | Description                      |
//! | ------- | -------------------------------- |
//! | N + 0   | Joint ID                         |
//! | N + 1-4 | New angle (int32) (deg \* 10^-3) |
//! | N + 5-8 | Speed (int32) (deg \* 10^-3) / s |
//!
//! ### Move Speed
//!
//! | Byte    | Description                      |
//! | ------- | -------------------------------- |
//! | N + 0   | Joint ID                         |
//! | N + 1-4 | Speed (int32) (deg \* 10^-3) / s |
//!
//! ### Follow Trajectory
//!
//! For N in [0 - 5]:
//!
//! | Byte    | Description                         |
//! | ------- | ----------------------------------- |
//! | N + 0-3 | Target angle (int32) (deg \* 10^-3) |
//! | N + 4-7 | Speed (int32) (deg \* 10^-3) / s    |
//!
//! ### Stop
//!
//! | Byte | Description                |
//! | ---- | -------------------------- |
//! | 0    | Stop immediately?          |
//! | 1    | Bitfield of joints to stop |
//!
//! ### Go Home
//!
//! | Byte | Description                   |
//! | ---- | ----------------------------- |
//! | 0    | Bitfield of joints to go home |
//!
//! ### Reset
//!
//! No payload
//!
//! ### Set Log Level
//!
//! | Byte | Description                 |
//! | ---- | --------------------------- |
//! | 0    | Log level (0-3, 4 for none) |
//!
//! ### Set Feedback
//!
//! | Byte | Description                                   |
//! | ---- | --------------------------------------------- |
//! | 0    | Bitfield of joints to enable/disable feedback |

use crate::checksum::{crc8ccitt, crc8ccitt_check};
use log::{info, warn};
use serialport::SerialPort;
use std::{
    error::Error,
    time::{Duration, Instant},
};

/// Map of error codes to error messages.
pub const ERROR_CODES: [&str; 8] = [
    "Other",
    "Malformed request",
    "Out of range",
    "Invalid joint",
    "Not initialized",
    "Not calibrated",
    "Cancelled",
    "Invalid firmware version",
];

/// Log levels used by the COBOT.
pub mod log_level {
    pub const DEBUG: u8 = 0x00;
    pub const INFO: u8 = 0x01;
    pub const WARN: u8 = 0x02;
    pub const ERROR: u8 = 0x03;
    pub const NONE: u8 = 0x04;
}

/// Message types that can be received from the COBOT
pub mod received_msg_type {
    pub const LOG: u8 = 0x00;
    pub const RESPONSE: u8 = 0x01;
}

/// Type of response message.
pub mod response_type {
    pub const ACK: u8 = 0x00;
    pub const DONE: u8 = 0x01;
    pub const ERROR: u8 = 0x02;
    pub const JOINTS: u8 = 0x03;
}

/// Message types that can be sent to the COBOT.
pub mod request_type {
    pub const INIT: u8 = 0x00;
    pub const CALIBRATE: u8 = 0x01;
    pub const _OVERRIDE: u8 = 0x02;
    pub const GET_JOINTS: u8 = 0x03;
    pub const MOVE_TO: u8 = 0x04;
    pub const MOVE_SPEED: u8 = 0x05;
    pub const _FOLLOW_TRAJECTORY: u8 = 0x06;
    pub const STOP: u8 = 0x07;
    pub const GO_HOME: u8 = 0x08;
    pub const RESET: u8 = 0x09;
    pub const SET_LOG_LEVEL: u8 = 0x0A;
    pub const SET_FEEDBACK: u8 = 0x0B;
}

/// Connection to the COBOT. Handles sending and receiving messages.
///
/// This struct will pass any received log messages to the standard logger. Responses are accessed
/// by ID and will be buffered for up to 1 second before being discarded.
pub struct CobotConnection {
    /// Serial port to communicate with the COBOT.
    port: Box<dyn SerialPort>,

    /// Firmware version of the COBOT.
    firmware_version: u32,

    /// Command ID to use for the next command.
    next_command_id: u32,

    /// Time to wait for a response before timing out.
    timeout: Duration,

    /// List of responses and the time they were received.
    responses: Vec<(Response, std::time::Instant)>,
}

/// Response received from the COBOT.
#[derive(Clone, Debug)]
pub struct Response {
    /// Command ID of the command that generated this response.
    pub command_id: u32,

    /// Type of response.
    pub response_type: u8,

    /// Payload of the response.
    pub payload: Vec<u8>,
}

/// Error returned by the COBOT.
#[derive(Clone, Debug)]
pub struct CobotError {
    /// Error code.
    pub code: u8,

    /// Error message.
    pub message: String,
}
impl std::fmt::Display for CobotError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let error_message = if self.code < ERROR_CODES.len() as u8 {
            ERROR_CODES[self.code as usize]
        } else {
            "Unknown error"
        };
        write!(
            f,
            "COBOT ERROR {} ({}): {}",
            self.code, error_message, self.message
        )
    }
}
impl std::error::Error for CobotError {}

impl CobotConnection {
    /// Creates a new connection to the COBOT.
    ///
    /// # Arguments
    ///
    /// * `port` - Serial port to communicate with the COBOT.
    /// * `firmware_version` - Firmware version of the COBOT.
    pub fn new(port: Box<dyn SerialPort>, firmware_version: u32, timeout: Duration) -> Self {
        CobotConnection {
            port,
            firmware_version,
            next_command_id: 0,
            timeout,
            responses: Vec::new(),
        }
    }

    /// Sends a request to the COBOT.
    ///
    /// # Arguments
    ///
    /// * `request_type` - Type of request to send.
    /// * `payload` - Payload of the request.
    ///
    /// # Returns
    ///
    /// The command ID of the request.
    pub fn send_request(
        &mut self,
        request_type: u8,
        payload: &[u8],
    ) -> Result<u32, Box<dyn Error>> {
        let command_id = self.next_command_id;
        self.next_command_id += 1;

        let mut message = vec![request_type];
        message.extend_from_slice(&command_id.to_le_bytes());
        message.extend_from_slice(payload);
        let length = message.len() as u8;

        let crc = crc8ccitt(&message);
        message.insert(0, crc);
        message.insert(0, length);
        message.insert(0, 0x24);

        self.port.write_all(&message)?;

        Ok(command_id)
    }

    /// Waits for a response from the COBOT. This will continually read from the serial port until
    /// a response is received, or the timeout is reached.
    ///
    /// # Arguments
    ///
    /// * `command_id` - Command ID of the request to wait for.
    /// * `timeout` - Maximum time to wait for the response.
    ///
    /// # Returns
    ///
    /// The response, or `None` if the response was not received before the timeout.
    pub fn wait_for_response(
        &mut self,
        command_id: u32,
        timeout: Duration,
    ) -> Result<Option<Response>, Box<dyn Error>> {
        let start_time = Instant::now();

        loop {
            // Filter out any responses that are too old.
            self.responses
                .retain(|(_, time)| start_time < *time + Duration::from_secs(30));

            // Check if the response has been received and return it if it has.
            if let Some(response_idx) = self
                .responses
                .iter()
                .position(|(response, _)| response.command_id == command_id)
            {
                return Ok(Some(self.responses.swap_remove(response_idx).0));
            }

            // Check if the timeout has been reached.
            let time_elapsed = Instant::now() - start_time;
            if time_elapsed >= timeout {
                return Ok(None);
            }

            // Read a response from the serial port.
            self.read_response(timeout - time_elapsed)?;
        }
    }

    /// Wait for an ACK response from the COBOT. If an error response is received, it will be
    /// returned.
    ///
    /// # Arguments
    ///
    /// * `command_id` - Command ID of the request to wait for.
    ///
    /// # Returns
    ///
    /// Ok if an ACK response was received, or an error if an error response was received.
    pub fn wait_for_ack(&mut self, command_id: u32) -> Result<(), Box<dyn Error>> {
        match self.wait_for_response(command_id, self.timeout)? {
            Some(response) => match response.response_type {
                response_type::ACK => Ok(()),
                response_type::ERROR => Err(Box::new(CobotError {
                    code: response.payload[0],
                    message: String::from_utf8_lossy(&response.payload[2..]).to_string(),
                })),
                _ => Err(Box::new(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Received unexpected response type",
                ))),
            },
            None => Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "Timed out waiting for response",
            ))),
        }
    }

    /// Wait for a DONE response from the COBOT. If an error response is received, it will be
    /// returned.
    ///
    /// # Arguments
    ///
    /// * `command_id` - Command ID of the request to wait for.
    ///
    /// # Returns
    ///
    /// Ok if a DONE response was received, or an error if an error response was received.
    pub fn wait_for_done(&mut self, command_id: u32) -> Result<(), Box<dyn Error>> {
        match self.wait_for_response(command_id, Duration::from_secs(60))? {
            Some(response) => match response.response_type {
                response_type::DONE => Ok(()),
                response_type::ERROR => Err(Box::new(CobotError {
                    code: response.payload[0],
                    message: String::from_utf8_lossy(&response.payload[2..]).to_string(),
                })),
                _ => Err(Box::new(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Received unexpected response type",
                ))),
            },
            None => Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "Timed out waiting for response",
            ))),
        }
    }

    /// Initialize the COBOT.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT was initialized successfully, or an error if the COBOT failed to initialize.
    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        let payload = &self.firmware_version.to_le_bytes();
        self.send_request(request_type::INIT, payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;

        Ok(())
    }

    /// Calibrate the COBOT.
    ///
    /// # Arguments
    ///
    /// * `joints` - Bitfield of joints to calibrate.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT was calibrated successfully, or an error if the COBOT failed to calibrate.
    pub fn calibrate(&mut self, joints: u8) -> Result<(), Box<dyn Error>> {
        let payload = [joints];
        self.send_request(request_type::CALIBRATE, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Get the current joint angles and speeds.
    ///
    /// # Returns
    ///
    /// Vector of tuples containing the joint angles and speeds in degrees and degrees per second,
    /// respectively.
    pub fn get_joints(&mut self) -> Result<Vec<(f32, f32)>, Box<dyn Error>> {
        self.send_request(request_type::GET_JOINTS, &[])?;
        let response = self.wait_for_response(self.next_command_id - 1, self.timeout)?;
        match response {
            Some(response) => match response.response_type {
                response_type::JOINTS => {
                    let joint_count = response.payload[0];
                    let mut joints = Vec::new();
                    for i in 0..joint_count {
                        let angle = i32::from_le_bytes([
                            response.payload[1 + i as usize * 8],
                            response.payload[2 + i as usize * 8],
                            response.payload[3 + i as usize * 8],
                            response.payload[4 + i as usize * 8],
                        ]) as f32
                            / 1000.0;
                        let speed = i32::from_le_bytes([
                            response.payload[5 + i as usize * 8],
                            response.payload[6 + i as usize * 8],
                            response.payload[7 + i as usize * 8],
                            response.payload[8 + i as usize * 8],
                        ]) as f32
                            / 1000.0;
                        joints.push((angle, speed));
                    }
                    Ok(joints)
                }
                response_type::ERROR => Err(Box::new(CobotError {
                    code: response.payload[0],
                    message: String::from_utf8_lossy(&response.payload[2..]).to_string(),
                })),
                _ => Err(Box::new(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Received unexpected response type",
                ))),
            },
            None => Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "Timed out waiting for response",
            ))),
        }
    }

    /// Move the given joints to the given angles at the given speeds. If a speed is `0` or `None`,
    /// the COBOT will use the default speed.
    ///
    /// # Arguments
    ///
    /// * `joints` - List of tuples containing the joint ID, angle, and speed to move to.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT moved successfully, or an error if the COBOT failed to move.
    pub fn move_to(&mut self, joints: &[(u8, f32, Option<f32>)]) -> Result<(), Box<dyn Error>> {
        let mut payload = Vec::new();
        for (joint_id, angle_f, speed_f) in joints {
            let angle = (angle_f * 1000.0) as i32;
            let speed = match speed_f {
                Some(speed_f) => (speed_f * 1000.0) as i32,
                None => 0,
            };
            payload.extend_from_slice(&joint_id.to_le_bytes());
            payload.extend_from_slice(&angle.to_le_bytes());
            payload.extend_from_slice(&speed.to_le_bytes());
        }
        self.send_request(request_type::MOVE_TO, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Move the given joints at the given speeds.
    ///
    /// # Arguments
    ///
    /// * `joints` - List of tuples containing the joint ID and speed to move at.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT moved successfully, or an error if the COBOT failed to move.
    #[allow(dead_code)]
    pub fn move_speed(&mut self, joints: &[(u8, f32)]) -> Result<(), Box<dyn Error>> {
        let mut payload = Vec::new();
        for (joint_id, speed_f) in joints {
            let speed = (speed_f * 1000.0) as i32;
            payload.extend_from_slice(&joint_id.to_le_bytes());
            payload.extend_from_slice(&speed.to_le_bytes());
        }
        self.send_request(request_type::MOVE_SPEED, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Stop the given joints.
    ///
    /// # Arguments
    ///
    /// * `joints` - Bitfield of joints to stop.
    /// * `immediately` - If true, the COBOT will stop immediately. Otherwise, it will decelerate
    ///
    /// # Returns
    ///
    /// Ok if the COBOT stopped successfully, or an error if the COBOT failed to stop.
    pub fn stop(&mut self, joints: u8, immediately: bool) -> Result<(), Box<dyn Error>> {
        let payload = [if immediately { 1 } else { 0 }, joints];
        self.send_request(request_type::STOP, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Home the given joints.
    ///
    /// # Arguments
    ///
    /// * `joints` - Bitfield of joints to home.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT homed successfully, or an error if the COBOT failed to home.
    #[allow(dead_code)]
    pub fn go_home(&mut self, joints: u8) -> Result<(), Box<dyn Error>> {
        let payload = [joints];
        self.send_request(request_type::GO_HOME, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Reset the COBOT.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT reset successfully, or an error if the COBOT failed to reset.
    #[allow(dead_code)]
    pub fn reset(&mut self) -> Result<(), Box<dyn Error>> {
        self.send_request(request_type::RESET, &[])?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Set the log level of the COBOT.
    ///
    /// # Arguments
    ///
    /// * `log_level` - Log level to set.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT set the log level successfully, or an error if the COBOT failed to set the
    /// log level.
    #[allow(dead_code)]
    pub fn set_log_level(&mut self, log_level: u8) -> Result<(), Box<dyn Error>> {
        let payload = [log_level];
        self.send_request(request_type::SET_LOG_LEVEL, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Set the feedback of the given joints.
    ///
    /// # Arguments
    ///
    /// * `joints` - Bitfield of joints to enable/disable feedback.
    ///
    /// # Returns
    ///
    /// Ok if the COBOT set the feedback successfully, or an error if the COBOT failed to set the
    /// feedback.
    #[allow(dead_code)]
    pub fn set_feedback(&mut self, joints: u8) -> Result<(), Box<dyn Error>> {
        let payload = [joints];
        self.send_request(request_type::SET_FEEDBACK, &payload)?;
        self.wait_for_ack(self.next_command_id - 1)?;
        self.wait_for_done(self.next_command_id - 1)?;

        Ok(())
    }

    /// Reads a response from the serial port and adds it to the list of responses. If log messages
    /// are received, they will be passed to the standard logger.
    ///
    /// # Arguments
    ///
    /// * `timeout` - Maximum time to wait for the response.
    ///
    /// # Returns
    ///
    /// The response, or `None` if the response was not received before the timeout.
    fn read_response(&mut self, timeout: Duration) -> Result<(), Box<dyn Error>> {
        let start_time = Instant::now();

        // Wait for a start byte.
        let mut start_byte = [0];
        while start_byte[0] != 0x24 {
            self.read_exact(&mut start_byte, self.remaining_timeout(start_time, timeout))?;
        }

        // Read the length and CRC.
        let mut length_crc = [0; 2];
        self.read_exact(&mut length_crc, self.remaining_timeout(start_time, timeout))?;
        let length = length_crc[0];
        let crc = length_crc[1];

        // Read the payload.
        let mut payload = vec![0; length as usize];
        self.read_exact(&mut payload, self.remaining_timeout(start_time, timeout))?;

        // Check the CRC.
        if !crc8ccitt_check(&payload, crc) {
            warn!("Received message with invalid CRC");
            return Ok(());
        }

        // Handle the message.
        match payload[0] {
            received_msg_type::LOG => {
                let level = match payload[1] {
                    log_level::DEBUG => log::Level::Debug,
                    log_level::INFO => log::Level::Info,
                    log_level::WARN => log::Level::Warn,
                    log_level::ERROR => log::Level::Error,
                    log_level::NONE => return Ok(()),
                    _ => {
                        warn!("Received message with invalid log level");
                        return Ok(());
                    }
                };
                let message = String::from_utf8_lossy(&payload[3..]);
                log::logger().log(
                    &log::Record::builder()
                        .args(format_args!("{}", message))
                        .level(level)
                        .target("cobot")
                        .file(Some("cobot"))
                        .line(Some(0))
                        .module_path(Some("cobot"))
                        .build(),
                );
            }
            received_msg_type::RESPONSE => {
                let response_type = payload[1];
                let command_id =
                    u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                let payload = payload[6..].to_vec();

                let response = Response {
                    command_id,
                    response_type,
                    payload,
                };
                self.responses.push((response, std::time::Instant::now()));
            }
            _ => {
                warn!("Received message with invalid type");
            }
        }

        Ok(())
    }

    /// Reads enough bytes from the serial port to fill the given buffer.
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to fill.
    /// * `timeout` - Maximum time to wait for the buffer to be filled.
    ///
    /// # Returns
    ///
    /// True if the buffer was filled, or false if the timeout was reached before the buffer was
    /// filled.
    fn read_exact(&mut self, buffer: &mut [u8], timeout: Duration) -> Result<bool, Box<dyn Error>> {
        self.port.set_timeout(timeout)?;
        if let Err(e) = self.port.read_exact(buffer) {
            if e.kind() == std::io::ErrorKind::TimedOut {
                return Ok(false);
            } else {
                return Err(Box::new(e));
            }
        }

        Ok(true)
    }

    /// Determine the remaining time until the timeout is reached. Will return 0 if the timeout has
    /// already been reached.
    ///
    /// # Arguments
    ///
    /// * `start_time` - Time the timeout started.
    /// * `timeout` - Timeout duration.
    ///
    /// # Returns
    ///
    /// The remaining time until the timeout is reached.
    fn remaining_timeout(&self, start_time: Instant, timeout: Duration) -> Duration {
        let time_elapsed = Instant::now() - start_time;
        if time_elapsed >= timeout {
            Duration::from_secs(0)
        } else {
            timeout - time_elapsed
        }
    }
}
