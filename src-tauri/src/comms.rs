use std::{error::Error, io::ErrorKind};

use log::error;
use serialport::SerialPort;

use crate::checksum::{crc8ccitt, crc8ccitt_check};

/// Message types that can be sent or received from the COBOT
pub mod msg_type {
    pub const MSG_ERR: u8 = 0x00;
    pub const MSG_ACK: u8 = 0x01;
    pub const MSG_DONE: u8 = 0x02;
    pub const MSG_INIT: u8 = 0x03;
    pub const MSG_CAL: u8 = 0x04;
    pub const _MSG_SET: u8 = 0x05;
    pub const MSG_GET: u8 = 0x06;
    pub const MSG_MOV: u8 = 0x07;
    pub const MSG_STP: u8 = 0x08;
    pub const _MSG_RST: u8 = 0x09;
    pub const _MSG_HOM: u8 = 0x0A;
    pub const MSG_POS: u8 = 0x0B;

    /// Length of payload, indexed by message type
    pub const PAYLOAD_LEN: [u8; 12] = [u8::MAX, 0, 0, 2, 0, 12, 0, 36, 1, 0, 0, 12];
}

pub type PayloadPair = (u8, Vec<u8>);

/// Connection to the COBOT. Handles sending and receiving messages.
pub struct CobotConnection {
    path: String,
    initialized: bool,
    fw_major: u8,
    fw_minor: u8,
    done_queue: Vec<u8>,
}

impl CobotConnection {
    pub fn new(path: &str, fw_major: u8, fw_minor: u8) -> Self {
        Self {
            path: path.to_string(),
            initialized: false,
            fw_major,
            fw_minor,
            done_queue: Vec::new(),
        }
    }

    fn connect_to_serial(&mut self) -> Result<Box<dyn SerialPort>, Box<dyn Error>> {
        let mut serial_port = serialport::new(self.path.as_str(), 115_200)
            .timeout(std::time::Duration::from_millis(100))
            .open()?;

        if !self.initialized {
            self.init(&mut serial_port)?;
        }

        Ok(serial_port)
    }

    /// Calibrates the COBOT. This is required before the COBOT can be used.
    /// This function blocks until the calibration is complete.
    pub fn calibrate(&mut self) -> Result<(), Box<dyn Error>> {
        let mut serial_port = self.connect_to_serial()?;
        self.send_and_wait(&mut serial_port, msg_type::MSG_CAL, &[])?;
        self.wait_for_done_serial(&mut serial_port, msg_type::MSG_CAL);
        Ok(())
    }

    /// Gets the current position of the COBOT. This function blocks until the position is received.
    pub fn get_position(&mut self) -> Result<[i16; 6], Box<dyn Error>> {
        let mut serial_port = self.connect_to_serial()?;
        let payload = self.send_and_wait(&mut serial_port, msg_type::MSG_GET, &[])?;
        let (received_type, received_payload) = payload.ok_or("Received an ACK instead of POS")?;
        if received_type != msg_type::MSG_POS {
            return Err(format!("Received unexpected message type: {}", received_type).into());
        }
        if received_payload.len() != msg_type::PAYLOAD_LEN[received_type as usize] as usize {
            return Err(format!(
                "Received unexpected payload length: {}",
                received_payload.len()
            )
            .into());
        }

        // Decode joint positions. Each is a 16-bit signed integer in little-endian format.
        let mut positions = [0; 6];
        for i in 0..6 {
            positions[i] =
                (received_payload[2 * i] as i16) | ((received_payload[2 * i + 1] as i16) << 8);
        }

        Ok(positions)
    }

    /// Moves a single joint to the specified position at the specified speed. If the speed is None,
    /// the COBOT will follow a calculated acceleration profile. This function does not block until
    /// the move is complete. Use wait_for_done() to wait for completion.
    pub fn move_joint(
        &mut self,
        joint: u8,
        position: i16,
        speed: Option<f32>,
    ) -> Result<(), Box<dyn Error>> {
        let mut joints = [None; 6];
        if joint >= joints.len() as u8 {
            return Err(format!("Invalid joint: {}", joint).into());
        }

        joints[joint as usize] = Some((position, speed));
        self.move_all_joints(joints)
    }

    /// Moves the COBOT's joints to the specified positions at the specified speeds. If a speed is
    /// None, the COBOT will follow a calculated acceleration profile. This function does not block
    /// until the move is complete. Use wait_for_done() to wait for completion.
    pub fn move_all_joints(
        &mut self,
        joints: [Option<(i16, Option<f32>)>; 6],
    ) -> Result<(), Box<dyn Error>> {
        // Encode joint positions and speeds. Each position is a 16-bit signed integer in
        // little-endian format. Each speed is a little-endian IEEE 754 single-precision
        // floating-point number. If a speed is None, it is encoded as `NaN`. If a joint is None,
        // its position is encoded as `0x7FFF` and the speed is ignored.
        let mut payload =
            Vec::with_capacity(msg_type::PAYLOAD_LEN[msg_type::MSG_MOV as usize] as usize);

        for joint in joints {
            match joint {
                Some((position, speed)) => {
                    payload.push(position as u8);
                    payload.push((position >> 8) as u8);
                    match speed {
                        Some(speed) => {
                            payload.extend_from_slice(&speed.to_le_bytes());
                        }
                        None => {
                            payload.extend_from_slice(&f32::NAN.to_le_bytes());
                        }
                    }
                }
                None => {
                    payload.push(0xFF);
                    payload.push(0x7F);
                    payload.extend_from_slice(&f32::NAN.to_le_bytes());
                }
            }
        }

        let mut serial_port = self.connect_to_serial()?;
        self.send_and_wait(&mut serial_port, msg_type::MSG_MOV, &payload)?;

        Ok(())
    }

    /// Stops a specific joint. This is done by commanding a single joint to move at a speed of 0.
    /// This function does not block until the move is complete. Use wait_for_done() to wait for
    /// completion.
    pub fn stop_joint(&mut self, joint: u8) -> Result<(), Box<dyn Error>> {
        self.move_joint(joint, 0, Some(0.0))
    }

    /// Stops all joints. This will wait until all joints have stopped moving. To smoothly stop,
    /// pass `true` for `smooth`. This will cause the COBOT to follow a calculated acceleration
    /// profile to stop. If `smooth` is `false`, the COBOT will stop immediately.
    pub fn stop_all(&mut self, smooth: bool) -> Result<(), Box<dyn Error>> {
        let mut serial_port = self.connect_to_serial()?;
        if smooth {
            self.send_and_wait(&mut serial_port, msg_type::MSG_STP, &[0x01])?;
        } else {
            self.send_and_wait(&mut serial_port, msg_type::MSG_STP, &[0x00])?;
        }
        self.wait_for_done_serial(&mut serial_port, msg_type::MSG_STP);
        Ok(())
    }

    /// Sends a message to the COBOT. This does not wait for a response.
    pub fn send_msg(
        &mut self,
        serial_port: &mut Box<dyn SerialPort>,
        msg_type: u8,
        payload: &[u8],
    ) -> Result<(), Box<dyn Error>> {
        let mut msg = vec![0x55, msg_type];
        msg.extend_from_slice(payload);
        msg.push(crc8ccitt(&msg));

        serial_port.write_all(&msg)?;

        Ok(())
    }

    pub fn receive_msg(
        &mut self,
        serial_port: &mut Box<dyn SerialPort>,
    ) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut received = Vec::with_capacity(3);

        // Wait for start byte (0x24)
        let mut buf = [0; 1];
        loop {
            match serial_port.read_exact(&mut buf) {
                Ok(_) => {
                    if buf[0] == 0x24 {
                        received.push(0x24);
                        break;
                    }
                }
                Err(e) => {
                    if e.kind() != ErrorKind::TimedOut {
                        return Err(e.into());
                    }
                }
            }
        }

        // Read message type
        serial_port.read_exact(&mut buf)?;
        received.push(buf[0]);

        // Read payload if applicable
        let payload_len = msg_type::PAYLOAD_LEN
            .get(received[1] as usize)
            .ok_or(format!("Invalid message type: {}", received[1]))?;
        if *payload_len > 0 {
            let mut payload = vec![0; *payload_len as usize];
            serial_port.read_exact(&mut payload)?;
            received.extend_from_slice(&payload);
        }

        // Read checksum
        serial_port.read_exact(&mut buf)?;
        received.push(buf[0]);

        // Verify checksum
        if !crc8ccitt_check(
            &received[..received.len() - 1],
            received[received.len() - 1],
        ) {
            return Err("Checksum mismatch".into());
        }

        Ok(received)
    }

    /// Sends a message to the COBOT and waits for a response. If the response is an error, an error
    /// is returned. If the response is an ACK, None is returned. If the response is a message with
    /// a payload, a tuple of the message type and payload is returned.
    pub fn send_and_wait(
        &mut self,
        serial_port: &mut Box<dyn SerialPort>,
        msg_type: u8,
        payload: &[u8],
    ) -> Result<Option<PayloadPair>, Box<dyn Error>> {
        self.send_msg(serial_port, msg_type, payload)?;
        loop {
            let received = self.receive_msg(serial_port)?;
            match received[1] {
                // If error, return error
                msg_type::MSG_ERR => return Err(format!("Error: {}", received[2]).into()),

                // If ACK, return None
                msg_type::MSG_ACK => return Ok(None),

                // If DONE, add to done queue and wait for next message
                msg_type::MSG_DONE => {
                    self.done_queue.push(received[2]);
                    continue;
                }

                // If message with payload, return message type and payload
                _ => {
                    return Ok(Some((
                        received[1],
                        received[2..received.len() - 1].to_vec(),
                    )))
                }
            }
        }
    }

    pub fn _wait_for_done(&mut self, msg_type: u8) {
        let mut serial_port = match self.connect_to_serial() {
            Ok(serial_port) => serial_port,
            Err(e) => {
                error!("Error connecting to serial port: {}", e);
                return;
            }
        };

        self.wait_for_done_serial(&mut serial_port, msg_type);
    }

    /// Initializes the COBOT. This is required before the COBOT can be used.
    fn init(&mut self, serial_port: &mut Box<dyn SerialPort>) -> Result<(), Box<dyn Error>> {
        self.send_and_wait(
            serial_port,
            msg_type::MSG_INIT,
            &[self.fw_major, self.fw_minor],
        )?;
        self.initialized = true;

        Ok(())
    }

    fn wait_for_done_serial(&mut self, serial_port: &mut Box<dyn SerialPort>, msg_type: u8) {
        loop {
            // Search for the first instance of the message type in the done queue. If found, remove
            // it and break.
            let mut found = false;
            self.done_queue.retain(|&x| {
                if !found && x == msg_type {
                    found = true;
                    false
                } else {
                    true
                }
            });
            if found {
                break;
            }

            // Wait for next message
            let received = match self.receive_msg(serial_port) {
                Ok(received) => received,
                Err(e) => {
                    error!("Error receiving message: {}", e);
                    continue;
                }
            };
            if received[1] == msg_type::MSG_DONE {
                self.done_queue.push(received[2]);
            }
        }
    }
}
