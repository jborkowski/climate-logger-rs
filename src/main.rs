#![no_std]
#![no_main]

use esp32_hal::clock::*;
use esp32_hal::spi::SpiMode;
use esp32_hal::{
    i2c::I2C, pac::Peripherals, prelude::*, timer::TimerGroup, Delay, Rtc, IO, spi::Spi,
};
use esp_backtrace as _;
use esp_println::println;
use profont::PROFONT_12_POINT;

use esp32_hal::prelude::_fugit_RateExtU32;

use dht_sensor::*;

use ssd1306;
use ssd1306::prelude::*;
use ssd1306::rotation::DisplayRotation;

use ssd1306::{
    mode::DisplayConfig,
    I2CDisplayInterface, Ssd1306,
};

use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use heapless::String;

use ds1307::{DateTimeAccess, Ds1307};

use embedded_sdmmc::*;

#[xtensa_lx_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    let mut gpio4 = io.pins.gpio4.into_open_drain_output();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio33,
        io.pins.gpio32,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .unwrap();

    let i2c_bus = shared_bus::BusManagerSimple::new(i2c);

    // init Display
    let display_interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
    let mut display = Ssd1306::new(
        display_interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display.init().unwrap();

    let text_style = MonoTextStyle::new(&PROFONT_12_POINT, BinaryColor::On);

    display.flush().unwrap();

    // init RTC
    let mut rtc = Ds1307::new(i2c_bus.acquire_i2c());
    // let datetime = NaiveDate::from_ymd(2022, 10, 9).and_hms(18, 35, 10);
    // rtc.set_datetime(&datetime).unwrap();

    // init SDMMC
    let sclk = io.pins.gpio18;
    let miso = io.pins.gpio19;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio5.into_push_pull_output();

    let spi = Spi::new_no_cs(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        400u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );


    let spi_dev = SdMmcSpi::new(spi, cs);
    let mut sd_controller = Controller::new(spi_dev, FakeTime {});

    println!("Init SD card...");
    let card = sd_controller.device().init().unwrap();
    println!("Card size {}", sd_controller.device().card_size_bytes().unwrap());

    loop {
        match dht22::Reading::read(&mut delay, &mut gpio4) {
            Ok(dht22::Reading {
                temperature,
                relative_humidity,
            }) => {
                let mut deg_string: String<32> = String::from(temperature as i32);
                deg_string.push_str("°").unwrap();

                let mut hum_string: String<32> = String::from(relative_humidity as i32);
                hum_string.push_str("% RHr").unwrap();
                display.clear();

                Text::with_baseline(
                    "Temperature: ",
                    Point::new(0, 16),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                Text::with_baseline(&deg_string, Point::new(97, 16), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline("Humidity: ", Point::new(0, 32), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                Text::with_baseline(&hum_string, Point::new(73, 32), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                // let mut datetime = String::new();

                // write!(&datetime, "{}", rtc.datetime().unwrap());

                // Text::with_baseline(&datetime, Point::new(0, 0), text_style, Baseline::Top)
                //     .draw(&mut display)
                //     .unwrap();

                display.flush().unwrap();

                println!("{}", rtc.datetime().unwrap());

                println!("{}°, {}% RH", temperature, relative_humidity)
            }
            Err(e) => println!("Error {:?}", e),
        }

        delay.delay_ms(500_u16);
    }
}


struct FakeTime;

impl TimeSource for FakeTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(1019, 11, 24, 3, 40, 31).unwrap()
    }
}
