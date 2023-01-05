#![no_main]
#![no_std]


use hal::delay::DelayFromCountDownTimer;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use hal::timer::Timer;
use stm32g4xx_hal as hal;
use cortex_m_semihosting::hprintln;
use cortex_m_rt::entry;
use log::info;
use hal::serial;
use stm32g4xx_hal::serial::FullConfig;

#[macro_use]
mod utils;

#[entry]
fn uart() -> ! {
    utils::logger::init();

    hprintln!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi());

    let gpioa = dp.GPIOA.split(&mut rcc);

    hprintln!("Init SYST delay");
    let mut delay_syst = cp.SYST.delay(&rcc.clocks);

    hprintln!("Init Timer2 delay");
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.ms()));

    //Configure the baud rate for USART communication

    let pin_tx = gpioa.pa2.into_alternate();
    let pin_rx = gpioa.pa3.into_alternate();

    let mut usart = serial::usart::SerialExt::usart(
        dp.USART2,
        (pin_tx),
        (pin_rx),
        serial::FullConfig::default(),
        (&mut rcc),
    ).unwrap();

    loop {
        hprintln!("Begin of loop");
        delay_tim2.delay_ms(1000 as u16);
        hprintln!("Sent data");
        usart.write(6).unwrap();
    }
}
