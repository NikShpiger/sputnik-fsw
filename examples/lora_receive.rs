//! LoRa P2P receive for STM32F103C8T6 with SPI2
//! Connections: SCK=PB13, MISO=PB14, MOSI=PB15, NSS=PB12, RESET=PB1, DIO0=PB0
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
use lora_phy::mod_params::*;
use lora_phy::sx127x::{Sx127x, Sx1276};
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQUENCY_IN_HZ: u32 = 433_000_000;

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

    // Пины для SPI2
    let nss = Output::new(p.PB12, Level::High, Speed::Low);
    let reset = Output::new(p.PB1, Level::High, Speed::Low);
    let irq = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs);

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = khz(200);
    let spi = spi::Spi::new(
        p.SPI2, p.PB13, // SCK
        p.PB15, // MOSI
        p.PB14, // MISO
        p.DMA1_CH5, p.DMA1_CH4, Irqs, spi_cfg,
    );
    let spi = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    let lora_config = lora_phy::sx127x::Config {
        chip: Sx1276, // можно Sx1278
        tcxo_used: false,
        rx_boost: false,
        tx_boost: false,
    };
    let iv = GenericSx127xInterfaceVariant::new(reset, irq, None, None).unwrap();
    let mut lora = LoRa::new(Sx127x::new(spi, iv, lora_config), false, Delay)
        .await
        .unwrap();

    // Индикатор успешного приёма – штатный светодиод PC13
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    let mut receiving_buffer = [00u8; 100];

    let mdltn_params = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_IN_HZ,
        )
        .unwrap();

    let rx_pkt_params = lora
        .create_rx_packet_params(
            4,
            false,
            receiving_buffer.len() as u8,
            true,
            false,
            &mdltn_params,
        )
        .unwrap();

    lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
        .unwrap();

    loop {
        receiving_buffer = [00u8; 100];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _)) => {
                if received_len == 3
                    && receiving_buffer[0] == 0x01u8
                    && receiving_buffer[1] == 0x02u8
                    && receiving_buffer[2] == 0x03u8
                {
                    info!("rx successful");
                    led.set_low(); // включить светодиод
                    Timer::after_secs(5).await;
                    led.set_high(); // выключить
                } else {
                    info!("rx unknown packet");
                }
            }
            Err(err) => info!("rx unsuccessful = {}", err),
        }
    }
}
