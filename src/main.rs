#![no_main]
#![no_std]


use core::borrow::BorrowMut;
use stm32g4xx_hal::{gpio::{gpioc, ExtiPin, GpioExt, Input, PullDown, SignalEdge}, rcc::RccExt, stm32, stm32::{interrupt, Interrupt}, stm32g4, syscfg::SysCfgExt};

use core::cell::RefCell;
use core::ptr::null;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::{asm::wfi, interrupt::Mutex};

use cortex_m_rt::entry;
use cortex_m_semihosting::{hprint, hprintln};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
//use embedded_hal::prelude::_embedded_hal_timer_CountDown;
use stm32g4xx_hal::gpio::gpioa::{PA11, PA5, PA6, PA9};
use stm32g4xx_hal::gpio::{Alternate, Output, PushPull};
use stm32g4xx_hal::time::U32Ext;

use stm32g4xx_hal::timer::{Event, Timer};
use panic_halt as _;
use stm32g4xx_hal::gpio::gpiob::{PB3, PB4, PB5};
use stm32g4xx_hal::stm32::{NVIC, TIM3};
use stm32g4xx_hal::stm32::adc1::jsqr::JEXTSEL_A::TIM3_CC1;
use stm32g4xx_hal::stm32::rcc::apb1enr1::TIM3EN_R;


// Make LED pin globally available
static G_LED_ON: AtomicBool = AtomicBool::new(true);


unsafe fn clear_tim2interrupt_bit() {
    (*stm32g4::stm32g431::TIM2::ptr())
        .sr
        //.write(|w| w.uif().clear_bit());
        .write(|w| w.uif().bit(false));

}
static LED: Mutex<RefCell<Option<PB4<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
//interruption pour éteindre la LED
#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        let mut option = LED.borrow(&cs);
        match option.borrow_mut().as_mut()  {
            None => {hprintln!("ERR");}
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

    dp.RCC.apb1enr1.write(|w| unsafe {
        w.tim3en().set_bit()
    });

    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain();


    //Fclock=170Mhz

    //PSC+1=fck/fcmpt=170Mhz/1Mhz=170
    //ARR=40000

    dp.TIM3.arr.write(|w| unsafe {
        w.bits(40000)
    });
    dp.TIM3.psc.write(|w| unsafe {
        w.bits(169)
    });

    //Configuration du capture compare
    // . Select the proper tim_tix_in[15:0] source (internal or external) with the TI1SEL[3:0] bits
    // in the TIMx_TISEL register.
    dp.TIM3.tisel.write(|w| unsafe {
        w.ti2sel().bits(0000)// tim_ti2_in0: TIMx_CH2
    });

    //     2. Select the active input: TIMx_CCR1 must be linked to the tim_ti1 input, so write the
    // CC1S bits to 01 in the TIMx_CCMR1 register. As soon as CC1S becomes different
    // from 00, the channel is configured in input and the TIMx_CCR1 register becomes read-
    //     only.
    dp.TIM3.ccmr1_input().write(|w| unsafe {
        w.cc1s().bits(10)//CC1 channel is configured as input, tim_ic1 is mapped on tim_ti2
    });
    //         3. Program the needed input filter duration in relation with the signal connected to the
    //         timer (when the input is one of the tim_tix (ICxF bits in the TIMx_CCMRx register). Let’s
    //        imagine that, when toggling, the input signal is not stable during at must 5 internal clock
    //        cycles. We must program a filter duration longer than these 5 clock cycles. We can
    //        validate a transition on tim_ti1 when 8 consecutive samples with the new level have
    //        been detected (sampled at fDTS frequency). Then write IC1F bits to 0011 in the
    //        TIMx_CCMR1 register.

    //         4. Select the edge of the active transition on the tim_ti1 channel by writing the CC1P and
    //        CC1NP and CC1NP bits to 000 in the TIMx_CCER register (rising edge in this case).
    dp.TIM3.ccer.write(|w| {//capture compare register
        w.
            cc1p().bit(true)
            .cc1np().bit(true)
            .cc1e().bit(true)// 6. Enable capture from the counter into the capture register by setting the CC1E bit in the
        //        TIMx_CCER register.
    });

    //         7. If needed, enable the related interrupt request by setting the CC1IE bit in the
    //        TIMx_DIER register, and/or the DMA request by setting the CC1DE bit in the
    //        TIMx_DIER register.
    dp.TIM3.dier.write(|w| unsafe {
        w
            .cc1ie().bit(true)
            .cc1de().bit(true)
    });

     dp.TIM3.cr1.write(|w| unsafe {
         w.cen().bit(true)
    });//activation compteur


    // Configure PA5 pin to blink LED
   // let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb4.into_push_pull_output();
    let mut capture: PB5<Alternate<2>> = gpiob.pb5.into_alternate();
    led.set_high().unwrap();
    cortex_m::interrupt::free(|cs| unsafe {
        LED
            .borrow(&cs)
            .replace(Some(led));
    });

    //demarrage du compteur
    let mut tim2 = Timer::new(dp.TIM2, &rcc.clocks).start_count_down(100.khz());


    tim2.listen(Event::TimeOut);
    unsafe { NVIC::unmask(Interrupt::TIM2) }
    let mut rise:bool =true;

    loop {
        if unsafe { (*stm32g4::stm32g431::TIM3::ptr()).sr.read().cc1if().bits() }&&rise
        {
            unsafe { (*stm32g4::stm32g431::TIM3::ptr()).cnt.reset() };
            //hprintln!("{}",unsafe{(*stm32g4::stm32g431::TIM3::ptr()).cnt.read().bits()});
            rise=false;
        }
        if unsafe { (*stm32g4::stm32g431::TIM3::ptr()).sr.read().cc1of().bits() }
        {
            let mut distance = unsafe { (*stm32g4::stm32g431::TIM3::ptr()).ccr1.read().bits() };
            //let mut distance = unsafe { (*stm32g4::stm32g431::TIM3::ptr()).cnt.read().bits() };

            unsafe { (*stm32g4::stm32g431::TIM3::ptr()).sr.write(|w| w.cc1of().bit(false)) };
            unsafe { (*stm32g4::stm32g431::TIM3::ptr()).sr.write(|w| w.cc1if().bit(false)) };
            //dp.TIM3.cr1.write(|w|unsafe{w.cen().bit(false)});
            hprintln!("distance={}cm",unsafe{distance/58});
            rise=true;

            cortex_m::interrupt::free(|cs| {
                let mut option = LED.borrow(&cs);
                match option.borrow_mut().as_mut() {
                    None => { hprintln!("ERR"); }
                    Some(mut led) => {
                        led.set_high().unwrap();
                    }
                }
            });
            tim2.start(100.khz());
            tim2.listen(Event::TimeOut);
            //unsafe { NVIC::unmask(Interrupt::TIM2) }
            //hprintln!("listen");
        }
    }
}
