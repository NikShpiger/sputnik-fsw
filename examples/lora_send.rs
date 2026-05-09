//! LoRa P2P send with multitasking (blinky + TX)
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals, spi};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{Sx127x, Sx1276};
use lora_phy::{mod_params::*, sx127x};
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQUENCY_IN_HZ: u32 = 433_000_000;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

// Новая задача: мигает светодиодом на PC13
#[embassy_executor::task]
async fn blinky(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after_millis(500).await;
        led.set_high();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // spawner теперь используется
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    // Светодиод для задачи blinky
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(blinky(led).unwrap()); // разворачиваем SpawnToken и запускаем

    // --- Настройка LoRa (ваши пины SPI2) ---
    let nss = Output::new(p.PB12, Level::High, Speed::Low);
    let reset = Output::new(p.PB1, Level::High, Speed::Low);
    let irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs);

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = khz(200);
    let spi = spi::Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH5, p.DMA1_CH4, Irqs, spi_cfg,
    );
    let spi = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    let lora_config = sx127x::Config {
        chip: Sx1276,
        tcxo_used: false,
        rx_boost: false,
        tx_boost: true,
    };
    let iv = GenericSx127xInterfaceVariant::new(reset, irq, None, None).unwrap();
    let mut lora = LoRa::new(Sx127x::new(spi, iv, lora_config), false, Delay)
        .await
        .unwrap();

    let mdltn_params = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_IN_HZ,
        )
        .unwrap();

    let mut tx_pkt_params = lora
        .create_tx_packet_params(4, false, true, false, &mdltn_params)
        .unwrap();

    let buffer = [0x01u8, 0x02u8, 0x03u8];

    loop {
        match lora
            .prepare_for_tx(&mdltn_params, &mut tx_pkt_params, 15, &buffer)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }

        match lora.tx().await {
            Ok(()) => info!("TX DONE"),
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }

        match lora.sleep(false).await {
            Ok(()) => info!("Sleep successful"),
            Err(err) => info!("Sleep unsuccessful = {}", err),
        }

        Timer::after_secs(10).await;
    }
}
