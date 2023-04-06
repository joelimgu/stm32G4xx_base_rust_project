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
use stm32g4xx_hal::gpio::gpioa::{PA11, PA5, PA6, PA9};
use stm32g4xx_hal::gpio::{Alternate, Output, PushPull};
use stm32g4xx_hal::time::U32Ext;

use stm32g4xx_hal::timer::{Event, Timer};
use panic_halt as _;
use stm32g4xx_hal::gpio::gpiob::{PB3, PB4, PB5};
use stm32g4xx_hal::stm32::{NVIC, TIM3};
use stm32g4xx_hal::stm32::rcc::apb1enr1::TIM3EN_R;


// Make LED pin globally available
static G_LED_ON: AtomicBool = AtomicBool::new(true);

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the button is pressed

unsafe fn clear_tim2interrupt_bit() {
    (*stm32g4::stm32g431::TIM2::ptr())
        .sr
        .write(|w| w.uif().clear_bit());

}
static LED: Mutex<RefCell<Option<PA9<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));


//interruption pour éteindre la LED
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

    dp.RCC.apb2enr.write(|w|unsafe{
        w.tim1en().bit(true)
    });

    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain();



    //Fclock=170Mhz
    //periode timer=periode Horloge*(PSC+1)*(ARR+1)
    //periode timer * fhorloge=(PSC+1)*(ARR+1)
    //1us*170*MHz=170=(PSC+1)*(ARR+1)
    //je prends PSC=16 ARR=9

    dp.TIM3.arr.write(|w| unsafe{
        w.bits(9)
    });
    dp.TIM3.psc.write(|w|unsafe{
        w.bits(16)
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
    // timer (when the input is one of the tim_tix (ICxF bits in the TIMx_CCMRx register). Let’s
    //        imagine that, when toggling, the input signal is not stable during at must 5 internal clock
    //        cycles. We must program a filter duration longer than these 5 clock cycles. We can
    //        validate a transition on tim_ti1 when 8 consecutive samples with the new level have
    //        been detected (sampled at fDTS frequency). Then write IC1F bits to 0011 in the
    //        TIMx_CCMR1 register.

    //         4. Select the edge of the active transition on the tim_ti1 channel by writing the CC1P and
    //        CC1NP and CC1NP bits to 000 in the TIMx_CCER register (rising edge in this case).
    dp.TIM3.ccer.write(|w| {//capture compare register
        w.cc1p().bit(true)
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

    //clear les flags
    dp.TIM3.sr.write(|w| unsafe {
        w.cc1if().clear_bit()
            .cc1of().clear_bit()
    });

    dp.TIM3.cr1.write(|w| unsafe {
        w.cen().bit(true)
    });//activation compteur


    // Configure PA5 pin to blink LED
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob=dp.GPIOB.split(&mut rcc);
    let mut led = gpioa.pa9.into_push_pull_output();
    let mut capture: PB5<Alternate<2>> = gpiob.pb5.into_alternate();
    led.set_high().unwrap();
    cortex_m::interrupt::free(|cs| unsafe {
        LED
        .borrow(&cs)
        .replace(Some(led));
    });

    //demarrage du compteur
    let mut tim2 = Timer::new(dp.TIM2, &rcc.clocks).start_count_down(1.hz());

    let mut val1=dp.TIM3.ccr1.read().bits();
    let mut val2=dp.TIM3.ccr1.read().bits();
    let mut result= val2-val1;
            //let mut tim3=Timer::new(dp.TIM3, &rcc.clocks).start_count_down(1.hz());//40 ms=25hz    38ms quand il n'y a pas d'obstacle
            tim2.listen(Event::TimeOut);

            unsafe{NVIC::unmask(Interrupt::TIM2)}
    // Move the pin into our global storage

    //let mut delay = cp.SYST.delay(&rcc.clocks);



    loop {
        let start=unsafe{(*stm32g4::stm32g431::TIM3::ptr()).sr.read().cc1if().bits()};
        let end=unsafe{(*stm32g4::stm32g431::TIM3::ptr()).sr.read().cc1of().bits()};

        let mut distance = unsafe {
            (*stm32g4::stm32g431::TIM3::ptr())
                .cnt
                .read().bits()
        };
        if start {
           let mut val1=unsafe{(*stm32g4::stm32g431::TIM3::ptr()).ccr1.read().bits()};
            hprintln!("val1={}",val1);
        };
        hprintln!("distance={}",distance);
        unsafe{(*stm32g4::stm32g431::TIM3::ptr()).sr.write( |w|w.cc1if().clear_bit())};
       if end {
           let mut val2=unsafe{(*stm32g4::stm32g431::TIM3::ptr()).ccr1.read().bits()};
           hprintln!("val2={}",val2);
            let mut result= val2-val1;
           hprintln!("result={}",result);
           hprintln!("distance={}",distance);
           unsafe{(*stm32g4::stm32g431::TIM3::ptr()).sr.write( |w| w.cc1of().clear_bit())};
       };
        // hprintln!("distance={}",distance);
        // hprintln!("result={}",result);
       // dp.RCC.apb2enr.write(|w| unsafe {
         //   w.});//activation compteur)
    }
}
