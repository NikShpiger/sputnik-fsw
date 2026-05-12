#![no_std]
#![no_main]

#[path = "../led.rs"]
mod led;
#[path = "../protocol.rs"]
mod protocol;

// Заглушки датчиков
mod sensors {
    use defmt::info;
    pub fn read_temperatures() -> (f32, f32) {
        info!("sensors: reading temperatures (stub)");
        (25.0, 25.0)
    }
    pub fn read_battery_voltage() -> f32 {
        info!("sensors: reading battery voltage (stub)");
        4.2
    }
    pub fn set_heater_power(power: f32) {
        info!("sensors: set heater power to {=f32}", power);
    }
    pub fn deploy_antenna(deploy: bool) {
        info!("sensors: deploy antenna {=bool}", deploy);
    }
}

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals, spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{LoRa, mod_params::*, sx127x};
use protocol::*;
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQ: u32 = 433_000_000;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    EXTI0 => embassy_stm32::exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

struct SatelliteState {
    beacon_interval: Duration,
    last_beacon: Instant,
    temperature_setpoint: f32,
    antenna_deployed: bool,
    heater_power: f32,
    last_temp_batt1: f32,
    last_temp_batt2: f32,
    last_battery_voltage: f32,
}

impl SatelliteState {
    fn new() -> Self {
        Self {
            beacon_interval: Duration::from_secs(10),
            last_beacon: Instant::now(),
            temperature_setpoint: 28.0,
            antenna_deployed: false,
            heater_power: 0.0,
            last_temp_batt1: 0.0,
            last_temp_batt2: 0.0,
            last_battery_voltage: 0.0,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Satellite booting...");

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    let led = Output::new(p.PC13, Level::High, Speed::Low);
    let token = led::blinky(led).unwrap();
    spawner.spawn(token);

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

    let mut state = SatelliteState::new();
    let mut last_temp_read = Instant::now();
    let mut rx_buf = [0u8; 128];

    let _ = lora
        .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt)
        .await;

    loop {
        // ------------------ Чтение датчиков ------------------
        if last_temp_read.elapsed() >= Duration::from_millis(500) {
            last_temp_read = Instant::now();
            let (t1, t2) = sensors::read_temperatures();
            let vbat = sensors::read_battery_voltage();
            state.last_temp_batt1 = t1;
            state.last_temp_batt2 = t2;
            state.last_battery_voltage = vbat;
            let avg = (t1 + t2) / 2.0;
            let error = state.temperature_setpoint - avg;
            state.heater_power = (error * 80.0).clamp(0.0, 255.0);
            sensors::set_heater_power(state.heater_power);
            info!(
                "T1: {=f32}, T2: {=f32}, Vbat: {=f32}, Heater: {=f32}",
                t1, t2, vbat, state.heater_power
            );
        }

        // ------------------ Отправка маяка ------------------
        if state.last_beacon.elapsed() >= state.beacon_interval {
            state.last_beacon = Instant::now();
            let beacon = BeaconFrame {
                frame_control: (0x00 << 4) | FrameType::Beacon as u8,
                satellite_id: SATELLITE_ID,
                beacon_interval: state.beacon_interval.as_secs() as u16,
                timestamp: Instant::now().as_millis() as u32,
                reserved: 0,
            };
            let bytes = unsafe {
                core::slice::from_raw_parts(
                    &beacon as *const _ as *const u8,
                    core::mem::size_of::<BeaconFrame>(),
                )
            };
            match lora.prepare_for_tx(&mdltn, &mut tx_pkt, 15, bytes).await {
                Ok(()) => {
                    let _ = lora.tx().await;
                    info!("Beacon sent");
                }
                Err(e) => info!("Beacon prepare error: {:?}", e),
            }
            let _ = lora
                .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt)
                .await;
        }

        // ------------------ Приём ------------------
        match embassy_time::with_timeout(Duration::from_millis(500), lora.rx(&rx_pkt, &mut rx_buf))
            .await
        {
            Ok(Ok((len, sig))) if len >= 1 => {
                let frame_type = rx_buf[0] & 0x0F;
                let rssi = sig.rssi;
                let snr = sig.snr;
                info!(
                    "RX type=0x{:x} len={} rssi={} snr={}",
                    frame_type, len, rssi, snr
                );

                // Обрабатываем только GroundCommand
                if frame_type == FrameType::GroundCommand as u8 {
                    if (len as usize) >= core::mem::size_of::<GroundCommandFrame>() {
                        let dest = unsafe {
                            core::ptr::read_unaligned(rx_buf.as_ptr().add(1) as *const u32)
                        };
                        let src = unsafe {
                            core::ptr::read_unaligned(rx_buf.as_ptr().add(5) as *const u32)
                        };
                        let code = rx_buf[12]; // команда лежит после 12-байтового заголовка
                        let params = if (len as usize) > 13 {
                            &rx_buf[13..(len as usize)]
                        } else {
                            &[]
                        };
                        if dest == SATELLITE_ID || dest == BROADCAST_ADDR {
                            info!("Ground cmd 0x{:x} from 0x{:x}", code, src);

                            // ACK Received
                            let ack_received = CommandAckFrame {
                                header: FrameHeader {
                                    frame_type: FrameType::CommandAck as u8,
                                    dest_addr: src,
                                    src_addr: SATELLITE_ID,
                                    protocol_id: ProtocolId::CmdControl as u8,
                                    flags: 0,
                                    data_length: 2,
                                },
                                command_code: code,
                                result_code: CommandResultCode::Received as u8,
                            };
                            let ack_bytes = unsafe {
                                core::slice::from_raw_parts(
                                    &ack_received as *const _ as *const u8,
                                    core::mem::size_of::<CommandAckFrame>(),
                                )
                            };
                            let _ = lora
                                .prepare_for_tx(&mdltn, &mut tx_pkt, 15, ack_bytes)
                                .await;
                            let _ = lora.tx().await;
                            let _ = lora
                                .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt)
                                .await;

                            // Выполнение команды
                            let success = execute_ground_command(&mut state, code, params).await;
                            if success && code == 0x01 {
                                // Отправка телеметрии
                                let telemetry = SatTelemetryFrame {
                                    header: FrameHeader {
                                        frame_type: FrameType::SatTelemetry as u8,
                                        dest_addr: src,
                                        src_addr: SATELLITE_ID,
                                        protocol_id: ProtocolId::SatTelemetry as u8,
                                        flags: 0,
                                        data_length: (core::mem::size_of::<SatTelemetryFrame>()
                                            - core::mem::size_of::<FrameHeader>())
                                            as u8,
                                    },
                                    temp_batt1: state.last_temp_batt1,
                                    temp_batt2: state.last_temp_batt2,
                                    battery_voltage: state.last_battery_voltage,
                                    heater_power: state.heater_power,
                                    setpoint: state.temperature_setpoint,
                                    uptime: Instant::now().as_millis() as u32,
                                    relay_mode: 0,    // ретрансляции нет
                                    storage_count: 0, // буфера нет
                                };
                                let tel_bytes = unsafe {
                                    core::slice::from_raw_parts(
                                        &telemetry as *const _ as *const u8,
                                        core::mem::size_of::<SatTelemetryFrame>(),
                                    )
                                };
                                let _ = lora
                                    .prepare_for_tx(&mdltn, &mut tx_pkt, 15, tel_bytes)
                                    .await;
                                let _ = lora.tx().await;
                                let _ = lora
                                    .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt)
                                    .await;
                                info!("Telemetry sent");
                            }

                            // ACK Success/Error
                            let result = if success {
                                CommandResultCode::Success as u8
                            } else {
                                CommandResultCode::Error as u8
                            };
                            let ack_final = CommandAckFrame {
                                header: FrameHeader {
                                    frame_type: FrameType::CommandAck as u8,
                                    dest_addr: src,
                                    src_addr: SATELLITE_ID,
                                    protocol_id: ProtocolId::CmdControl as u8,
                                    flags: 0,
                                    data_length: 2,
                                },
                                command_code: code,
                                result_code: result,
                            };
                            let ack_final_bytes = unsafe {
                                core::slice::from_raw_parts(
                                    &ack_final as *const _ as *const u8,
                                    core::mem::size_of::<CommandAckFrame>(),
                                )
                            };
                            let _ = lora
                                .prepare_for_tx(&mdltn, &mut tx_pkt, 15, ack_final_bytes)
                                .await;
                            let _ = lora.tx().await;
                            let _ = lora
                                .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pkt)
                                .await;
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

async fn execute_ground_command(state: &mut SatelliteState, code: u8, params: &[u8]) -> bool {
    match code {
        0x01 => {
            info!("Get Telemetry");
            true
        }
        0x02 => {
            info!("Reset – rebooting in 100ms");
            Timer::after(Duration::from_millis(100)).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        0x03 => {
            if params.len() >= 2 {
                let secs = u16::from_le_bytes([params[0], params[1]]);
                if secs >= 5 {
                    state.beacon_interval = Duration::from_secs(secs as u64);
                    info!("Beacon interval set to {}s", secs);
                    return true;
                }
            }
            false
        }
        0x04 => {
            if let Some(&deploy) = params.first() {
                state.antenna_deployed = deploy != 0;
                sensors::deploy_antenna(state.antenna_deployed);
                return true;
            }
            false
        }
        0x05 => {
            if params.len() >= 2 {
                let raw = i16::from_le_bytes([params[0], params[1]]);
                let setpoint = raw as f32 / 10.0;
                if setpoint >= -20.0 && setpoint <= 80.0 {
                    state.temperature_setpoint = setpoint;
                    info!("Setpoint: {=f32}", setpoint);
                    return true;
                }
            }
            false
        }
        0x06 => {
            // Режим ретрансляции заглушен
            info!("Relay mode command ignored (not implemented)");
            false
        }
        _ => false,
    }
}
