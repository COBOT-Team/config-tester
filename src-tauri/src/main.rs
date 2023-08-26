// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::sync::Mutex;

use comms::CobotConnection;

mod checksum;
mod comms;

struct AppState {
    cobot: Mutex<CobotConnection>,
}

#[tauri::command]
fn calibrate(state: tauri::State<AppState>) -> Result<(), String> {
    state
        .cobot
        .lock()
        .unwrap()
        .calibrate()
        .map_err(|e| e.to_string())
}

#[tauri::command]
fn get_position(state: tauri::State<AppState>) -> Result<[i16; 6], String> {
    state
        .cobot
        .lock()
        .unwrap()
        .get_position()
        .map_err(|e| e.to_string())
}

#[tauri::command]
fn move_joint(
    state: tauri::State<AppState>,
    joint: u8,
    position: i16,
    speed: Option<f32>,
) -> Result<(), String> {
    state
        .cobot
        .lock()
        .unwrap()
        .move_joint(joint, position, speed)
        .map_err(|e| e.to_string())
}

#[tauri::command]
fn move_all_joints(
    state: tauri::State<AppState>,
    joints: [Option<(i16, Option<f32>)>; 6],
) -> Result<(), String> {
    state
        .cobot
        .lock()
        .unwrap()
        .move_all_joints(joints)
        .map_err(|e| e.to_string())
}

#[tauri::command]
fn stop_joint(state: tauri::State<AppState>, joint: u8) -> Result<(), String> {
    state
        .cobot
        .lock()
        .unwrap()
        .stop_joint(joint)
        .map_err(|e| e.to_string())
}

#[tauri::command]
fn stop_all(state: tauri::State<AppState>, smooth: bool) -> Result<(), String> {
    state
        .cobot
        .lock()
        .unwrap()
        .stop_all(smooth)
        .map_err(|e| e.to_string())
}

fn main() {
    tauri::Builder::default()
        .manage(AppState {
            cobot: Mutex::new(CobotConnection::new("/dev/ttyUSB0", 1, 0)),
        })
        .invoke_handler(tauri::generate_handler![
            calibrate,
            get_position,
            move_joint,
            move_all_joints,
            stop_joint,
            stop_all
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
