#![no_std]
#![no_main]

use esp32_hal::clock::*;
use esp32_hal::pac::I2C0;
use esp32_hal::{gpio, i2c::I2C, pac::Peripherals, prelude::*, timer::TimerGroup, Delay, Rtc, IO};
use esp_backtrace as _;
use esp_hal_common::{system::PeripheralClockControl, Unknown};
use esp_println::println;
use profont::PROFONT_12_POINT;

use esp32_hal::prelude::_fugit_RateExtU32;

use dht_sensor::*;

use ssd1306;
use ssd1306::prelude::*;
use ssd1306::rotation::DisplayRotation;

use ssd1306::{
    mode::{BufferedGraphicsMode, DisplayConfig},
    I2CDisplayInterface, Ssd1306,
};

use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Baseline, Text},
};

use heapless::String;

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

    let mut display = init_display(
        peripherals.I2C0,
        io.pins.gpio33,
        io.pins.gpio32,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let text_style = MonoTextStyle::new(&PROFONT_12_POINT, BinaryColor::On);

    display.flush().unwrap();

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

                Text::with_baseline("Temperature: ", Point::new(0, 16), text_style, Baseline::Top)
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

                display.flush().unwrap();

                println!("{}°, {}% RH", temperature, relative_humidity)
            }
            Err(e) => println!("Error {:?}", e),
        }

        delay.delay_ms(500_u16);
    }
}

fn init_display(
    i2c: I2C0,
    sda: gpio::Gpio33<Unknown>,
    scl: gpio::Gpio32<Unknown>,
    pcc: &mut PeripheralClockControl,
    clocks: &Clocks,
) -> Ssd1306<
    I2CInterface<I2C<I2C0>>,
    ssd1306::prelude::DisplaySize128x64,
    BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
> {
    let i2c = I2C::new(i2c, sda, scl, 100u32.kHz(), pcc, clocks).unwrap();

    let display_interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(
        display_interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display.init().unwrap();

    return display;
}
