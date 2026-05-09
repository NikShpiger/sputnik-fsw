//! Спутник: мигает светодиодом PC13 и каждые 10 секунд отправляет LoRa-маяк.
//! Частота 433 МГц, модуль SX1278, SPI2 (пины PB12..PB15, PB1, PB0).

#![no_std]
#![no_main]

// Подключаем модуль светодиода из папки src
#[path = "../led.rs"]
mod led;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals, spi};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{LoRa, mod_params::*, sx127x};
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQ: u32 = 433_000_000; // рабочая частота

// Привязываем прерывания DMA (SPI2 RX/TX) и внешнее прерывание (DIO0)
bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Настройка тактирования (HSI 8 МГц)
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    // ---- Задача мигания светодиодом (работает параллельно) ----
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(led::blinky(led).unwrap());

    // ---- Инициализация LoRa (SPI2, как в вашем рабочем примере) ----
    let nss = Output::new(p.PB12, Level::High, Speed::Low); // NSS / CS
    let reset = Output::new(p.PB1, Level::High, Speed::Low); // RESET
    let irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs); // DIO0

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = khz(200); // 200 кГц
    let spi = spi::Spi::new(
        p.SPI2, p.PB13, // SCK
        p.PB15, // MOSI
        p.PB14, // MISO
        p.DMA1_CH5, p.DMA1_CH4, Irqs, spi_cfg,
    );
    let spi_dev = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    let lora_cfg = sx127x::Config {
        chip: Sx1276, // SX1278 работает как SX1276
        tcxo_used: false,
        rx_boost: false,
        tx_boost: true, // усиление передачи включено
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
    let mut tx_params = lora
        .create_tx_packet_params(4, false, true, false, &mdltn_params)
        .unwrap();

    let mut last_beacon = embassy_time::Instant::now();

    loop {
        // Отправляем маяк каждые 10 секунд
        if last_beacon.elapsed().as_secs() >= 10 {
            last_beacon = embassy_time::Instant::now();
            let ts = embassy_time::Instant::now().as_millis() as u32;

            // Формируем пакет: 4 байта ID спутника + 2 байта времени
            let mut pkt = [0u8; 6];
            pkt[0..4].copy_from_slice(&0x00000001u32.to_le_bytes());
            pkt[4..6].copy_from_slice(&ts.to_le_bytes()[..2]);

            // Передача: подготовка -> отправка
            match lora
                .prepare_for_tx(&mdltn_params, &mut tx_params, 15, &pkt)
                .await
            {
                Ok(()) => match lora.tx().await {
                    Ok(()) => info!("Beacon sent: ID=1, ts={}", ts),
                    Err(e) => info!("Beacon TX error: {:?}", e),
                },
                Err(e) => info!("Beacon prepare error: {:?}", e),
            }
        }
        Timer::after_millis(100).await; // небольшая пауза, чтобы не нагружать цикл
    }
}
