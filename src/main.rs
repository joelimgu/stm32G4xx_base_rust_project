#![no_main]
#![no_std]


use core::borrow::BorrowMut;
use stm32g4xx_hal::{gpio::{gpioc, ExtiPin, GpioExt, Input, PullDown, SignalEdge}, rcc::RccExt, stm32, stm32::{interrupt, Interrupt}, stm32g4, syscfg::SysCfgExt};

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::{asm::wfi, interrupt::Mutex};

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32g4xx_hal::gpio::gpioa::{PA5, PA9};
use stm32g4xx_hal::gpio::{Output, PushPull};
use stm32g4xx_hal::time::U32Ext;

use stm32g4xx_hal::timer::{Event, Timer};
use panic_halt as _;
use stm32g4xx_hal::stm32::NVIC;

type ButtonPin = gpioc::PC13<Input<PullDown>>;

// Make LED pin globally available
static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static G_LED_ON: AtomicBool = AtomicBool::new(true);

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the button is pressed

unsafe fn clear_tim2interrupt_bit() {
    (*stm32g4::stm32g431::TIM2::ptr())
        .sr
        .write(|w| w.uif().clear_bit());

}
static LED: Mutex<RefCell<Option<PA9<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        let mut option = LED.borrow(&cs).borrow_mut().take();
        match option {
            None => {}
            Some(mut led) => {
                led.set_low().unwrap();
            }
        }

    });
     unsafe { clear_tim2interrupt_bit() }
}
        #[entry]
fn main() -> ! {
    let mut dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain();

    // Configure PA5 pin to blink LED
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa9.into_push_pull_output();
    led.set_high().unwrap();
    cortex_m::interrupt::free(|cs| unsafe {
        LED
        .borrow(&cs)
        .replace(Some(led));
    });


            let mut tim2 = Timer::new(dp.TIM2, &rcc.clocks).start_count_down(1.hz());
            tim2.listen(Event::TimeOut);
            tim2.release();
            unsafe{NVIC::unmask(Interrupt::TIM2)}
    // Move the pin into our global storage

    //let mut delay = cp.SYST.delay(&rcc.clocks);

    loop {
        // wfi();
        //
        // if G_LED_ON.load(Ordering::Relaxed) {
        //     led.set_high().unwrap();
        // } else {
        //     led.set_low().unwrap();
        // }
    }
}
