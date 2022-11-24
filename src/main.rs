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

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    hprintln!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi());

    hprintln!("Init Led");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    hprintln!("Init SYST delay");
    let mut delay_syst = cp.SYST.delay(&rcc.clocks);

    hprintln!("Init Timer2 delay");
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.ms()));

    loop {

        //led.set_low();
        hprintln!("Toggle");
        led.toggle().unwrap();
        hprintln!("SYST delay");
        delay_syst.delay(1000.ms());
        hprintln!("Toggle");
        led.toggle().unwrap();
        hprintln!("TIM2 delay");
        delay_tim2.delay_ms(1000_u16);
    }
}
