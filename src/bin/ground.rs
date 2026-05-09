//! Наземная станция: постоянно слушает эфир и выводит принятые маяки.
//! Частота 433 МГц, модуль SX1278, SPI2 (пины PB12..PB15, PB1, PB0).

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals, spi};
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{LoRa, mod_params::*, sx127x};
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQ: u32 = 433_000_000;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    // ---- Инициализация LoRa (аналогично спутнику) ----
    let nss = Output::new(p.PB12, Level::High, Speed::Low);
    let reset = Output::new(p.PB1, Level::High, Speed::Low);
    let irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs);

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
        tx_boost: false, // приёмнику усиление передачи не нужно
    };
    let iv = GenericSx127xInterfaceVariant::new(reset, irq, None, None).unwrap();
    let mut lora = LoRa::new(Sx127x::new(spi_dev, iv, lora_cfg), false, Delay)
        .await
        .unwrap();

    // Параметры модуляции
    let mdltn_params = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQ,
        )
        .unwrap();

    // Настройка непрерывного приёма
    let rx_pkt_params = lora
        .create_rx_packet_params(4, false, 255, true, false, &mdltn_params)
        .unwrap();
    lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
        .unwrap();

    let mut buf = [0u8; 128]; // буфер для приёма

    loop {
        match lora.rx(&rx_pkt_params, &mut buf).await {
            Ok((len, _)) if len >= 6 => {
                // Первые 4 байта – ID спутника, следующие 2 – младшая часть времени
                let id = u32::from_le_bytes(buf[0..4].try_into().unwrap());
                let ts = (buf[4] as u32) | ((buf[5] as u32) << 8);
                info!("Beacon from {} at {}ms", id, ts);
            }
            Ok(_) => {
                // Пакет слишком короткий или не маяк – игнорируем
            }
            Err(e) => info!("RX error: {:?}", e),
        }
    }
}
