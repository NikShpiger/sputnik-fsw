//! Протокол связи спутника и наземной станции.
//! Включает типы кадров, заголовок, структуры команд, маяка и телеметрии.
//! Пока используется только маяк (Beacon), остальное потребуется позже.

use defmt::Format;

// ================== Адреса и идентификаторы ==================
pub const SATELLITE_ID: u32 = 0x00000001; // уникальный ID спутника
pub const BROADCAST_ADDR: u32 = 0xFFFFFFFF; // широковещательный адрес
pub const GROUND_STATION_ADDR: u32 = 0x00000010; // адрес наземной станции (пример)

// ================== Типы кадров ==================
/// Возможные типы пакетов (поле frame_type, младшие 4 бита).
#[derive(Format, Debug, PartialEq, Clone, Copy)]
pub enum FrameType {
    Beacon = 0x01,        // маяк, периодическая рассылка
    Data = 0x02,          // пользовательские данные
    Command = 0x03,       // команда (пока не используется)
    CommandAck = 0x04,    // подтверждение выполнения команды
    SatTelemetry = 0x05,  // телеметрия спутника
    DataDump = 0x06,      // выгрузка накопленных данных
    GroundCommand = 0x07, // команда с Земли
    Transport = 0x08,     // транспортный кадр (фрагментация и пр.)
}

// ================== Коды команд ==================
/// Коды команд, которые может послать наземная станция.
#[derive(Format, Debug, PartialEq, Clone, Copy)]
pub enum CommandCode {
    GetTelemetry = 0x01,      // запрос телеметрии
    Reset = 0x02,             // сброс спутника
    SetBeaconInterval = 0x03, // изменить интервал маяка
    DeployAntenna = 0x04,     // раскрыть антенну
    SetTemperature = 0x05,    // установить уставку температуры
    SetRelayMode = 0x06,      // включить/выключить ретрансляцию
}

// ================== Базовый заголовок ==================
/// Общий заголовок для всех кадров (8 байт).
/// Все поля упакованы без выравнивания (packed).
#[repr(C, packed)]
pub struct FrameHeader {
    /// Младшие 4 бита – тип кадра (FrameType), старшие 4 – версия протокола.
    pub frame_type: u8,
    /// Адрес получателя (32 бита, little-endian).
    pub dest_addr: u32,
    /// Адрес отправителя.
    pub src_addr: u32,
    /// Идентификатор протокола / подтип.
    pub protocol_id: u8,
    /// Флаги (зарезервировано, можно использовать для ACK, приоритета и т.п.).
    pub flags: u8,
    /// Длина полезной нагрузки в байтах (после заголовка).
    pub data_length: u8,
}

// ================== Структуры конкретных кадров ==================

/// Маяк, периодически отправляемый спутником.
#[repr(C, packed)]
pub struct BeaconFrame {
    pub header: FrameHeader,  // frame_type = Beacon, dest = BROADCAST
    pub satellite_id: u32,    // идентификатор спутника
    pub beacon_interval: u16, // текущий интервал между маяками в секундах
    pub timestamp: u32,       // время отправки в миллисекундах (от старта)
    pub reserved: u8,         // зарезервировано (0)
}

/// Команда, посылаемая наземной станцией.
#[repr(C, packed)]
pub struct GroundCommandFrame {
    pub header: FrameHeader, // frame_type = GroundCommand
    pub command_code: u8,    // код команды (CommandCode)
                             // Далее могут идти параметры команды (длина вычисляется из data_length)
}

/// Подтверждение выполнения команды.
#[repr(C, packed)]
pub struct CommandAckFrame {
    pub header: FrameHeader, // frame_type = CommandAck
    pub command_code: u8,    // код исполненной команды
    pub result_code: u8,     // результат (0 = успех, 1 = ошибка)
}

/// Кадр телеметрии спутника.
#[repr(C, packed)]
pub struct SatTelemetryFrame {
    pub header: FrameHeader,  // frame_type = SatTelemetry
    pub temp_batt1: f32,      // температура батареи 1 (°C)
    pub temp_batt2: f32,      // температура батареи 2 (°C)
    pub battery_voltage: f32, // напряжение аккумулятора (В)
    pub heater_power: f32,    // мощность нагревателя (0..255)
    pub setpoint: f32,        // уставка температуры
    pub uptime: u32,          // время работы в миллисекундах
    pub relay_mode: u8,       // 1 – ретрансляция включена
    pub storage_count: u8,    // количество записей в буфере хранения
}
