// file: src/protocol.rs
#![allow(dead_code)]
use defmt::Format;

pub const SATELLITE_ID: u32 = 0x00000001;
pub const BROADCAST_ADDR: u32 = 0xFFFFFFFF;
pub const GROUND_STATION_ADDR: u32 = 0x00000010;

#[derive(Format, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum FrameType {
    Beacon = 0x01,
    Data = 0x02,
    Command = 0x03,
    CommandAck = 0x04,
    SatTelemetry = 0x05,
    DataDump = 0x06,
    GroundCommand = 0x07,
    Transport = 0x08,
}

#[derive(Format, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum ProtocolId {
    Raw = 0x00,
    CmdControl = 0x01,
    StoreForward = 0x03,
    SatTelemetry = 0x04,
    TimeSync = 0x05,
    Fragmentation = 0x09,
}

#[derive(Format, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum GroundCommandCode {
    GetTelemetry = 0x01,
    Reset = 0x02,
    SetBeaconInterval = 0x03,
    DeployAntenna = 0x04,
    SetTemperature = 0x05,
    SetRelayMode = 0x06,
}

#[derive(Format, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum CommandResultCode {
    Success = 0x00,
    Error = 0x01,
    Received = 0xFE,
}

#[repr(C, packed)]
pub struct FrameHeader {
    pub frame_type: u8,
    pub dest_addr: u32,
    pub src_addr: u32,
    pub protocol_id: u8,
    pub flags: u8,
    pub data_length: u8,
}

// Новая структура маяка без заголовка (соответствует спецификации)
#[repr(C, packed)]
pub struct BeaconFrame {
    pub frame_control: u8,
    pub satellite_id: u32,
    pub beacon_interval: u16,
    pub timestamp: u32,
    pub reserved: u8,
}

#[repr(C, packed)]
pub struct GroundCommandFrame {
    pub header: FrameHeader,
    pub command_code: u8,
}

#[repr(C, packed)]
pub struct CommandAckFrame {
    pub header: FrameHeader,
    pub command_code: u8,
    pub result_code: u8,
}

#[repr(C, packed)]
pub struct SatTelemetryFrame {
    pub header: FrameHeader,
    pub temp_batt1: f32,
    pub temp_batt2: f32,
    pub battery_voltage: f32,
    pub heater_power: f32,
    pub setpoint: f32,
    pub uptime: u32,
    pub relay_mode: u8,
    pub storage_count: u8,
}

#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct StoredDataRecord {
    pub timestamp: u32,
    pub rssi: i16,
    pub snr: i8,
    pub data_length: u8,
    pub data: [u8; 239],
}

impl Default for StoredDataRecord {
    fn default() -> Self {
        Self {
            timestamp: 0,
            rssi: 0,
            snr: 0,
            data_length: 0,
            data: [0u8; 239],
        }
    }
}

#[repr(C, packed)]
pub struct DataDumpFrame {
    pub header: FrameHeader,
    pub record: StoredDataRecord,
}
