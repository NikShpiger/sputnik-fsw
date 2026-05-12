// Файл: src/bin/sensors.rs

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::adc::{Adc, SampleTime, Resolution};
use embassy_stm32::pwm::{Pwm, Channel, Frequency};
use embassy_stm32::gpio::{Level, Output, Speed};
use embedded_hal_async::delay::DelayNs;

// Сторонние крейты для OneWire и DS18B20
use ds18b20::{Ds18b20, Resolution as DsResolution};
use one_wire_bus::OneWire;

// ---------- ПАРАМЕТРЫ (копия из вашего скетча) ----------
const CAL_OFFSET_BATT1: f32 = 1.3;
const CAL_OFFSET_BATT2: f32 = 2.3;
const VOLTAGE_DIVIDER_RATIO: f32 = 1.373;
const VREF: f32 = 3.3;

const PID_KP: f32 = 80.0;
const PID_KI: f32 = 1.5;
const PID_KD: f32 = 0.2;
const PID_OUTPUT_MIN: f32 = 0.0;
const PID_OUTPUT_MAX: f32 = 255.0;

/// Полная структура состояния датчиков и регулятора
pub struct Sensors {
    temp1: Ds18b20<
        OneWire<'static, embassy_stm32::peripherals::PA11>,
        embassy_time::Delay,
    >,
    temp2: Ds18b20<
        OneWire<'static, embassy_stm32::peripherals::PA12>,
        embassy_time::Delay,
    >,
    adc: Adc<'static, embassy_stm32::peripherals::ADC1>,
    adc_pin: embassy_stm32::gpio::PA9,
    heater: Pwm<'static, embassy_stm32::peripherals::TIM1>,
    antenna: Output<'static, embassy_stm32::gpio::PA3>,
    pid: PidController,
    last_temp_reading: Instant,
    last_valid_temp1: f32,
    last_valid_temp2: f32,
    setpoint: f32,
    heater_power: f32,
    battery_voltage: f32,
}

/// Простейший PID–регулятор (без сторонних крейтов)
struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    prev_time: Option<Instant>,
    output_min: f32,
    output_max: f32,
}

impl PidController {
    fn new(kp: f32, ki: f32, kd: f32, output_min: f32, output_max: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            prev_time: None,
            output_min,
            output_max,
        }
    }

    fn calculate(&mut self, setpoint: f32, current: f32, now: Instant) -> f32 {
        let error = setpoint - current;
        let dt = self.prev_time.map(|t| (now - t).as_millis() as f32 / 1000.0).unwrap_or(0.5);

        // Пропорциональная часть
        let p = self.kp * error;

        // Интегральная часть с антивиндом
        self.integral += error * dt;
        // Простейшее ограничение: если выход уже на пределе, не накапливаем интеграл
        let i = self.ki * self.integral;

        // Дифференциальная часть
        let d = if dt > 0.0 {
            self.kd * (error - self.prev_error) / dt
        } else {
            0.0
        };

        let mut output = p + i + d;
        output = output.clamp(self.output_min, self.output_max);

        self.prev_error = error;
        self.prev_time = Some(now);

        // Защита интеграла от верхнего и нижнего насыщения
        if output >= self.output_max && error > 0.0 {
            self.integral -= error * dt;
        } else if output <= self.output_min && error < 0.0 {
            self.integral -= error * dt;
        }

        output
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_time = None;
    }
}

impl Sensors {
    pub async fn new(
        p: embassy_stm32::init(???) // здесь будет передана структура Peripherals
    ) -> Result<Self, ...> {
        // На практике лучше принимать через аргументы все нужные пины и периферию.
        // Мы используем unsafe получения PA11, PA12, PA8, PA3, PA9, ADC1, TIM1.
        // Это временный упрощённый конструктор для демонстрации.

        let owb1 = OneWire::new(unsafe { embassy_stm32::peripherals::PA11::steal() }).unwrap();
        let owb2 = OneWire::new(unsafe { embassy_stm32::peripherals::PA12::steal() }).unwrap();
        let mut temp1 = Ds18b20::new(owb1).unwrap();
        let mut temp2 = Ds18b20::new(owb2).unwrap();
        temp1.set_resolution(DsResolution::Bits10).ok();
        temp2.set_resolution(DsResolution::Bits10).ok();

        let mut adc = Adc::new(unsafe { embassy_stm32::peripherals::ADC1::steal() }, /* interrupts */, Default::default());
        let adc_pin = unsafe { embassy_stm32::gpio::PA9::steal() };

        let pwm = Pwm::new(
            unsafe { embassy_stm32::peripherals::TIM1::steal() },
            /* interrupts */,
            embassy_stm32::pwm::Config {
                frequency: Frequency::Hz(1000),
                ..Default::default()
            },
        );
        let mut heater = pwm.channel(Channel::Ch1, unsafe { embassy_stm32::gpio::PA8::steal() }).unwrap();
        heater.set_duty(0);

        let antenna = Output::new(unsafe { embassy_stm32::gpio::PA3::steal() }, Level::Low, Speed::Low);

        Ok(Self {
            temp1,
            temp2,
            adc,
            adc_pin,
            heater,
            antenna,
            pid: PidController::new(PID_KP, PID_KI, PID_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX),
            last_temp_reading: Instant::now(),
            last_valid_temp1: 0.0,
            last_valid_temp2: 0.0,
            setpoint: 28.0,
            heater_power: 0.0,
            battery_voltage: 0.0,
        })
    }

    /// Выполнить полный цикл: опрос датчиков, обновление PID, ШИМ, измерение напряжения
    pub async fn update(&mut self) {
        // 1. Запрос конверсии
        self.temp1.start_measurement().await.ok();
        self.temp2.start_measurement().await.ok();
        Timer::after(Duration::from_millis(200)).await;  // для 10 бит – 187.5 мс

        // 2. Считывание сырых значений
        let raw1 = self.temp1.read_temperature().await.ok().and_then(|r| r);
        let raw2 = self.temp2.read_temperature().await.ok().and_then(|r| r);

        // 3. Калибровка и проверка валидности
        if let Some(t) = raw1 {
            let cal = t + CAL_OFFSET_BATT1;
            if is_valid_temp(cal) {
                self.last_valid_temp1 = cal;
            }
        }
        if let Some(t) = raw2 {
            let cal = t + CAL_OFFSET_BATT2;
            if is_valid_temp(cal) {
                self.last_valid_temp2 = cal;
            }
        }

        // 4. Средняя температура
        let avg_temp = (self.last_valid_temp1 + self.last_valid_temp2) / 2.0;

        // 5. PID
        let power = self.pid.calculate(self.setpoint, avg_temp, Instant::now());
        self.heater_power = power.clamp(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        let duty = self.heater_power / PID_OUTPUT_MAX;  // 0 .. 1
        self.heater.set_duty(duty);

        // 6. Измерение напряжения батареи
        self.battery_voltage = self.read_battery_voltage().await;

        self.last_temp_reading = Instant::now();
    }

    async fn read_battery_voltage(&mut self) -> f32 {
        const SAMPLES: usize = 10;
        let mut sum: u32 = 0;
        for _ in 0..SAMPLES {
            let val = self.adc.read(&mut self.adc_pin).await.unwrap();
            sum += val as u32;
            Timer::after(Duration::from_millis(1)).await;
        }
        let avg = (sum as f32) / SAMPLES as f32;
        let pin_voltage = (avg / 4095.0) * VREF;
        pin_voltage * VOLTAGE_DIVIDER_RATIO
    }

    pub fn set_setpoint(&mut self, sp: f32) {
        self.setpoint = sp;
        self.pid.reset();  // сброс интеграла при смене уставки
    }

    pub fn set_antenna(&mut self, deploy: bool) {
        if deploy {
            self.antenna.set_high();
        } else {
            self.antenna.set_low();
        }
    }

    /// Возвращает последние измеренные значения для телеметрии
    pub fn telemetry(&self) -> (f32, f32, f32, f32, f32) {
        (
            self.last_valid_temp1,
            self.last_valid_temp2,
            self.battery_voltage,
            self.heater_power,
            self.setpoint,
        )
    }
}

fn is_valid_temp(t: f32) -> bool {
    t > -55.0 && t < 125.0 && (t - (-127.0)).abs() > 0.01 && (t - 85.0).abs() > 0.01
}
