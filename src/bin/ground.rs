#![no_std]
#![no_main]

#[path = "../protocol.rs"]
mod protocol;

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{Config as UartConfig, DataBits, Parity, StopBits, Uart};
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals, spi, usart};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{LoRa, mod_params::*, sx127x};
use protocol::*;
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQ: u32 = 433_000_000;
const ACK_TIMEOUT_MS: u64 = 3000;
const TELEMETRY_TIMEOUT_MS: u64 = 8000;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    EXTI0 => embassy_stm32::exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// ---------- конкретный тип LoRa-радио (как в спутнике) ----------

type LoRaType = LoRa<
    Sx127x<
        ExclusiveDevice<
            embassy_stm32::spi::Spi<'static, peripherals::SPI2, dma::NoFifo>,
            Output<'static>,
            Delay,
        >,
        GenericSx127xInterfaceVariant<
            Output<'static>,
            ExtiInput<'static, interrupt::typelevel::EXTI0>,
        >,
        Delay,
    >,
>;

// ---------- вспомогательные функции радио ----------

async fn send_packet(
    lora: &mut LoRaType,
    mdltn: &ModulationParams,
    tx_pkt: &mut PacketParams,
    data: &[u8],
) -> Result<(), ()> {
    lora.prepare_for_tx(mdltn, tx_pkt, 15, data)
        .await
        .map_err(|_| ())?;
    lora.tx().await.map_err(|_| ())?;
    Ok(())
}

async fn start_rx(lora: &mut LoRaType, mdltn: &ModulationParams, rx_pkt: &PacketParams) {
    let _ = lora.prepare_for_rx(RxMode::Continuous, mdltn, rx_pkt).await;
}

// ---------- состояние станции ----------

struct StationState {
    waiting_for_ack: bool,
    waiting_for_telemetry: bool,
    last_command_code: u8,
    last_command_time: Instant,
}

impl StationState {
    fn new() -> Self {
        Self {
            waiting_for_ack: false,
            waiting_for_telemetry: false,
            last_command_code: 0,
            last_command_time: Instant::now(),
        }
    }
}

// ---------- функции вывода в UART ----------

async fn uart_write(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, data: &[u8]) {
    let _ = uart.write(data).await;
}

async fn uart_write_str(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, s: &str) {
    uart_write(uart, s.as_bytes()).await;
}

async fn uart_write_u8(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: u8) {
    let mut buf = [0u8; 3];
    let mut pos = 0;
    if val >= 100 {
        buf[pos] = b'0' + val / 100;
        pos += 1;
    }
    if val >= 10 {
        buf[pos] = b'0' + (val % 100) / 10;
        pos += 1;
    }
    buf[pos] = b'0' + val % 10;
    pos += 1;
    uart_write(uart, &buf[..pos]).await;
}

async fn uart_write_u16(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: u16) {
    if val >= 10000 {
        uart_write_u8(uart, (val / 10000) as u8).await;
    }
    let rem = val % 10000;
    if rem >= 1000 {
        uart_write_u8(uart, (rem / 1000) as u8).await;
    }
    // упростим: просто выводим через маленький буфер
    let mut buf = [0u8; 5];
    let mut pos = 0;
    let mut v = val;
    if v == 0 {
        buf[pos] = b'0';
        pos = 1;
    } else {
        while v > 0 {
            buf[pos] = b'0' + (v % 10) as u8;
            v /= 10;
            pos += 1;
        }
        // реверс
        for i in 0..pos / 2 {
            buf.swap(i, pos - 1 - i);
        }
    }
    uart_write(uart, &buf[..pos]).await;
}

async fn uart_write_u32(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: u32) {
    if val <= 0xFFFF {
        uart_write_u16(uart, val as u16).await;
        return;
    }
    let hi = (val / 10000) as u16;
    let lo = (val % 10000) as u16;
    if hi > 0 {
        uart_write_u16(uart, hi).await;
    }
    // выводим lo с ведущими нулями
    let mut buf = [0u8; 5];
    let mut v = lo;
    for i in 0..4 {
        buf[3 - i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    uart_write(uart, &buf).await;
}

async fn uart_write_hex_u8(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: u8) {
    let high = val >> 4;
    let low = val & 0x0F;
    let hex = |d: u8| -> u8 { if d < 10 { b'0' + d } else { b'A' + (d - 10) } };
    uart_write(uart, &[hex(high), hex(low)]).await;
}

async fn uart_write_hex_u32(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: u32) {
    for shift in (0..32).step_by(8).rev() {
        uart_write_hex_u8(uart, ((val >> shift) & 0xFF) as u8).await;
    }
}

async fn uart_write_f32(uart: &mut Uart<'static, peripherals::USART1, usart::Async>, val: f32) {
    let int_part = val as i32;
    let frac = ((val.abs() - (int_part.abs() as f32)) * 10.0) as i32;
    if int_part < 0 {
        uart_write(uart, b"-").await;
    }
    uart_write_u32(uart, int_part.abs() as u32).await;
    uart_write(uart, b".").await;
    uart_write_u32(uart, frac.abs() as u32).await;
}

// ---------- отправка команды ----------

async fn send_ground_command(
    lora: &mut LoRaType,
    mdltn: &ModulationParams,
    tx_pkt: &mut PacketParams,
    rx_pkt: &PacketParams,
    state: &mut StationState,
    uart: &mut Uart<'static, peripherals::USART1, usart::Async>,
    cmd: u8,
    params: &[u8],
) {
    let header_size = core::mem::size_of::<FrameHeader>();
    let data_len = 1 + params.len() as u8;
    let mut buffer = [0u8; 128];
    buffer[0] = (0x00 << 4) | FrameType::GroundCommand as u8;
    unsafe {
        core::ptr::write_unaligned(buffer.as_mut_ptr().add(1) as *mut u32, SATELLITE_ID);
        core::ptr::write_unaligned(buffer.as_mut_ptr().add(5) as *mut u32, GROUND_STATION_ADDR);
    }
    buffer[9] = ProtocolId::CmdControl as u8;
    buffer[10] = 0;
    buffer[11] = data_len;
    buffer[header_size] = cmd;
    if !params.is_empty() {
        buffer[header_size + 1..header_size + 1 + params.len()].copy_from_slice(params);
    }

    let total_len = header_size + data_len as usize;
    uart_write_str(uart, "TX cmd 0x").await;
    uart_write_hex_u8(uart, cmd).await;
    uart_write_str(uart, "\r\n").await;

    if send_packet(lora, mdltn, tx_pkt, &buffer[..total_len])
        .await
        .is_ok()
    {
        state.waiting_for_ack = true;
        state.last_command_code = cmd;
        state.last_command_time = Instant::now();
    } else {
        uart_write_str(uart, "Failed to send command\r\n").await;
    }
    start_rx(lora, mdltn, rx_pkt).await;
}

// ---------- обработка входящих LoRa‑кадров ----------

async fn handle_rx(
    lora: &mut LoRaType,
    mdltn: &ModulationParams,
    tx_pkt: &mut PacketParams,
    rx_pkt: &PacketParams,
    state: &mut StationState,
    uart: &mut Uart<'static, peripherals::USART1, usart::Async>,
    rx_buf: &mut [u8],
    len: usize,
) {
    let frame_type = rx_buf[0] & 0x0F;
    match frame_type {
        x if x == FrameType::Beacon as u8 => {
            if len >= core::mem::size_of::<BeaconFrame>() {
                let sat_id =
                    unsafe { core::ptr::read_unaligned(rx_buf.as_ptr().add(12) as *const u32) };
                let beacon_interval =
                    unsafe { core::ptr::read_unaligned(rx_buf.as_ptr().add(16) as *const u16) };
                let timestamp =
                    unsafe { core::ptr::read_unaligned(rx_buf.as_ptr().add(18) as *const u32) };
                uart_write_str(uart, "BEACON: Sat=0x").await;
                uart_write_hex_u32(uart, sat_id).await;
                uart_write_str(uart, " Interval=").await;
                uart_write_u16(uart, beacon_interval).await;
                uart_write_str(uart, " s Timestamp=").await;
                uart_write_u32(uart, timestamp).await;
                uart_write_str(uart, " ms\r\n").await;
            }
        }
        x if x == FrameType::CommandAck as u8 => {
            if len >= core::mem::size_of::<CommandAckFrame>() {
                let cmd_code = rx_buf[12];
                let result_code = rx_buf[13];
                uart_write_str(uart, "ACK: cmd=0x").await;
                uart_write_hex_u8(uart, cmd_code).await;
                uart_write_str(uart, " result=").await;
                match result_code {
                    x if x == CommandResultCode::Received as u8 => {
                        uart_write_str(uart, "RECEIVED\r\n").await
                    }
                    x if x == CommandResultCode::Success as u8 => {
                        uart_write_str(uart, "SUCCESS\r\n").await
                    }
                    x if x == CommandResultCode::Error as u8 => {
                        uart_write_str(uart, "ERROR\r\n").await
                    }
                    _ => {
                        uart_write_str(uart, "0x").await;
                        uart_write_hex_u8(uart, result_code).await;
                        uart_write_str(uart, "\r\n").await;
                    }
                };
                if state.waiting_for_ack && cmd_code == state.last_command_code {
                    if result_code == CommandResultCode::Received as u8 {
                        state.last_command_time = Instant::now();
                    } else {
                        state.waiting_for_ack = false;
                        if result_code == CommandResultCode::Success as u8
                            && state.last_command_code == GroundCommandCode::GetTelemetry as u8
                        {
                            state.waiting_for_telemetry = true;
                            state.last_command_time = Instant::now();
                            uart_write_str(uart, "Waiting for telemetry...\r\n").await;
                        }
                    }
                }
            }
        }
        x if x == FrameType::SatTelemetry as u8 => {
            if len >= core::mem::size_of::<SatTelemetryFrame>() {
                let base = core::mem::size_of::<FrameHeader>();
                let t1 =
                    unsafe { core::ptr::read_unaligned(rx_buf.as_ptr().add(base) as *const f32) };
                let t2 = unsafe {
                    core::ptr::read_unaligned(rx_buf.as_ptr().add(base + 4) as *const f32)
                };
                let v = unsafe {
                    core::ptr::read_unaligned(rx_buf.as_ptr().add(base + 8) as *const f32)
                };
                let h = unsafe {
                    core::ptr::read_unaligned(rx_buf.as_ptr().add(base + 12) as *const f32)
                };
                let sp = unsafe {
                    core::ptr::read_unaligned(rx_buf.as_ptr().add(base + 16) as *const f32)
                };
                let uptime = unsafe {
                    core::ptr::read_unaligned(rx_buf.as_ptr().add(base + 20) as *const u32)
                };
                let relay = rx_buf[base + 24];
                let storage = rx_buf[base + 25];
                uart_write_str(uart, "TELEMETRY:\r\n Temp1: ").await;
                uart_write_f32(uart, t1).await;
                uart_write_str(uart, "\r\n").await;
                uart_write_str(uart, " Temp2: ").await;
                uart_write_f32(uart, t2).await;
                uart_write_str(uart, "\r\n").await;
                uart_write_str(uart, " Voltage: ").await;
                uart_write_f32(uart, v).await;
                uart_write_str(uart, " V\r\n").await;
                uart_write_str(uart, " Heater: ").await;
                uart_write_f32(uart, h).await;
                uart_write_str(uart, "/255\r\n").await;
                uart_write_str(uart, " Setpoint: ").await;
                uart_write_f32(uart, sp).await;
                uart_write_str(uart, " C\r\n").await;
                uart_write_str(uart, " Uptime: ").await;
                uart_write_u32(uart, uptime / 1000).await;
                uart_write_str(uart, " s\r\n").await;
                uart_write_str(uart, " Relay: ").await;
                uart_write_str(uart, if relay != 0 { "ON\r\n" } else { "OFF\r\n" }).await;
                uart_write_str(uart, " Storage: ").await;
                uart_write_u8(uart, storage).await;
                uart_write_str(uart, "\r\n").await;
                state.waiting_for_telemetry = false;
            }
        }
        _ => {}
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Ground station booting...");

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    // UART1 (PA10 RX, PA9 TX) – без DMA
    let mut uart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        Default::default(),
        Default::default(),
    )
    .unwrap();

    uart_write_str(&mut uart, "=== SATELLITE GROUND STATION ===\r\n").await;
    uart_write_str(&mut uart, "1: Telemetry  2: Reset  3: Beacon interval\r\n").await;
    uart_write_str(&mut uart, "4: Antenna  5: Setpoint  6: Relay  9: Menu\r\n").await;

    // LoRa
    let nss = Output::new(p.PB12, Level::High, Speed::Low);
    let reset = Output::new(p.PB1, Level::High, Speed::Low);
    let irq_pin = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs);

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = khz(200);
    let spi = spi::Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH5, p.DMA1_CH4, Irqs, spi_cfg,
    );
    let spi_dev = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    let lora_cfg = sx127x::Config {
        chip: Sx1276,
        tcxo_used: false,
        rx_boost: false,
        tx_boost: true,
    };
    let iv = GenericSx127xInterfaceVariant::new(reset, irq_pin, None, None).unwrap();
    let mut lora = LoRa::new(Sx127x::new(spi_dev, iv, lora_cfg), false, Delay)
        .await
        .unwrap();

    let mdltn = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQ,
        )
        .unwrap();
    let mut tx_pkt = lora
        .create_tx_packet_params(4, false, true, false, &mdltn)
        .unwrap();
    let rx_pkt = lora
        .create_rx_packet_params(4, false, 128, true, false, &mdltn)
        .unwrap();

    let mut state = StationState::new();
    let mut rx_buf = [0u8; 128];
    start_rx(&mut lora, &mdltn, &rx_pkt).await;

    loop {
        // Таймауты
        if state.waiting_for_ack
            && (Instant::now().as_millis() - state.last_command_time.as_millis()) > ACK_TIMEOUT_MS
        {
            state.waiting_for_ack = false;
            state.waiting_for_telemetry = false;
            uart_write_str(&mut uart, "Timeout waiting for ACK\r\n").await;
        }
        if state.waiting_for_telemetry
            && (Instant::now().as_millis() - state.last_command_time.as_millis())
                > TELEMETRY_TIMEOUT_MS
        {
            state.waiting_for_telemetry = false;
            uart_write_str(&mut uart, "Timeout waiting for telemetry\r\n").await;
        }

        // Приём LoRa
        match embassy_time::with_timeout(Duration::from_millis(500), lora.rx(&rx_pkt, &mut rx_buf))
            .await
        {
            Ok(Ok((len, _sig))) if len >= 1 => {
                handle_rx(
                    &mut lora,
                    &mdltn,
                    &mut tx_pkt,
                    &rx_pkt,
                    &mut state,
                    &mut uart,
                    &mut rx_buf,
                    len,
                )
                .await;
                start_rx(&mut lora, &mdltn, &rx_pkt).await;
            }
            _ => {}
        }

        // Чтение UART (с таймаутом 10 мс)
        let mut byte = [0u8];
        if embassy_time::with_timeout(Duration::from_millis(10), uart.read(&mut byte))
            .await
            .is_ok()
        {
            match byte[0] {
                b'1' => {
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x01,
                        &[],
                    )
                    .await
                }
                b'2' => {
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x02,
                        &[],
                    )
                    .await
                }
                b'3' => {
                    let params = [10u8, 0u8];
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x03,
                        &params,
                    )
                    .await;
                }
                b'4' => {
                    let params = [1u8];
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x04,
                        &params,
                    )
                    .await;
                }
                b'5' => {
                    let params = [0x1D, 0x01]; // 28.5°C
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x05,
                        &params,
                    )
                    .await;
                }
                b'6' => {
                    let params = [1u8];
                    send_ground_command(
                        &mut lora,
                        &mdltn,
                        &mut tx_pkt,
                        &rx_pkt,
                        &mut state,
                        &mut uart,
                        0x06,
                        &params,
                    )
                    .await;
                }
                b'9' => {
                    uart_write_str(&mut uart, "=== MENU ===\r\n1: Telemetry  2: Reset  3: Beacon interval\r\n4: Antenna  5: Setpoint  6: Relay  9: Menu\r\n").await;
                }
                _ => {}
            }
        }

        Timer::after(Duration::from_millis(5)).await;
    }
}
