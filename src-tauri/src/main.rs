// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::time::Duration;

use comms::CobotConnection;
use tauri::async_runtime::Mutex;

mod checksum;
mod comms;

const FIRMWARE_VERSION: u32 = 5;

struct AppState {
    cobot: Mutex<Option<Box<CobotConnection>>>,
}

/// Connect to the cobot over the given serial port.
#[tauri::command]
async fn connect(
    state: tauri::State<'_, AppState>,
    port_name: String,
    baud_rate: u32,
) -> Result<(), String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_some() {
        return Err("Already connected".to_string());
    }

    let port = serialport::new(port_name, baud_rate)
        .timeout(std::time::Duration::from_millis(100))
        .open()
        .map_err(|e| format!("Failed to open port: {}", e))?;

    let connection = CobotConnection::new(port, FIRMWARE_VERSION, Duration::from_millis(100));
    *cobot = Some(Box::new(connection));

    Ok(())
}

/// Disconnect from the cobot.
#[tauri::command]
async fn disconnect(state: tauri::State<'_, AppState>) -> Result<(), String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_none() {
        return Err("Not connected".to_string());
    }

    *cobot = None;

    Ok(())
}

/// Initialize the cobot.
#[tauri::command]
async fn init(state: tauri::State<'_, AppState>) -> Result<(), String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_none() {
        return Err("Not connected".to_string());
    }

    cobot
        .as_mut()
        .unwrap()
        .init()
        .map_err(|e| format!("Failed to initialize: {}", e))?;

    cobot
        .as_mut()
        .unwrap()
        .calibrate(0b111111)
        .map_err(|e| format!("Failed to calibrate: {}", e))?;

    Ok(())
}

/// Get the angles of all joints.
#[tauri::command]
async fn get_angles(state: tauri::State<'_, AppState>) -> Result<Vec<f32>, String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_none() {
        return Err("Not connected".to_string());
    }

    let joint_states = cobot
        .as_mut()
        .unwrap()
        .get_joints()
        .map_err(|e| format!("Failed to get joint states: {}", e))?;

    let angles = joint_states
        .into_iter()
        .map(|joint| joint.0)
        .collect::<Vec<_>>();

    Ok(angles)
}

/// Move a single joint to the given angle at the given speed.
#[tauri::command]
async fn move_joint(
    state: tauri::State<'_, AppState>,
    joint: u8,
    angle: f32,
    speed: f32,
) -> Result<(), String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_none() {
        return Err("Not connected".to_string());
    }

    cobot
        .as_mut()
        .unwrap()
        .move_to(&[(joint, angle, Some(speed))])
        .map_err(|e| format!("Failed to move joint: {}", e))?;

    Ok(())
}

/// Stop a single joint smoothly.
#[tauri::command]
async fn stop_joint(state: tauri::State<'_, AppState>, joint: u8) -> Result<(), String> {
    let mut cobot = state.cobot.lock().await;
    if cobot.is_none() {
        return Err("Not connected".to_string());
    }

    cobot
        .as_mut()
        .unwrap()
        .stop(1 << joint, false)
        .map_err(|e| format!("Failed to stop joint: {}", e))?;

    Ok(())
}

fn main() {
    flexi_logger::Logger::try_with_env_or_str("info")
        .unwrap()
        .start()
        .unwrap();

    tauri::Builder::default()
        .manage(AppState {
            cobot: Mutex::new(None),
        })
        .invoke_handler(tauri::generate_handler![
            connect, disconnect, init, get_angles, move_joint, stop_joint
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
