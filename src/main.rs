#![no_main]
#![no_std]

use stm32g4xx_hal as hal;
use cortex_m_semihosting::hprintln;
use cortex_m_rt::entry;
use log::info;
use hal::serial::FullConfig;
use hal::serial::usart::{Rx, Tx};
use hal::delay::DelayFromCountDownTimer;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use hal::timer::Timer;
use hal::interrupt;

use drs_0x01::builder::HerkulexMessage;
use embedded_hal::serial::{Read, Write};
use hal::nb::block;
use cortex_m;
use stm32g4xx_hal::gpio::Alternate;
use stm32g4xx_hal::gpio::gpioa::{PA2, PA3};
use stm32g4xx_hal::serial::{NoDMA, Serial, usart};
use stm32g4xx_hal::stm32::{Interrupt, USART2};

#[macro_use]
mod utils;

const BUFF_SIZE: usize = 20;
static mut BUFF_INDEX: usize = 0;
static mut BUFFER: &mut [u8; BUFF_SIZE] = &mut [0; BUFF_SIZE];
type USART_Tx = Tx<stm32::USART2, PA2<Alternate<7>>, NoDMA>;
type USART_Rx = Rx<stm32::USART2, PA3<Alternate<7>>, NoDMA>;

static mut Rx: Option<USART_Rx> = None;


/// A communication for the USART
pub struct Communication<'a> {
    tx: &'a mut USART_Tx,
}

/// A trait that implements the communication with a servo.
pub trait HerkulexCommunication {
    /// Send a message to the Herkulex servo
    fn send_message(&mut self, msg: HerkulexMessage);

    /// Receive a message from the Herkulex servo
    fn read_message(&self) -> [u8; BUFF_SIZE];
}

impl<'a> Communication<'a> {
    /// Create a new communication with Tx and Rx
    pub fn new(tx: &'a mut USART_Tx, mut rx: USART_Rx) -> Communication<'a> {
        let comm = Communication { tx };

        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        rx.listen();
        cortex_m::interrupt::free(|_| unsafe {
            Rx.replace(rx);
        });
        comm
    }
}

impl<'a> HerkulexCommunication for Communication<'a> {
    /// Send a message to the Herkulex servo
    fn send_message(&mut self, msg: HerkulexMessage) {
        for b in &msg {
            hal::block!(self.tx.write(*b)).unwrap();
        }
    }
    /// Receive a message from the Herkulex servo
    ///
    /// Can be stuck in the interruption
    fn read_message(&self) -> [u8; BUFF_SIZE] {
        cortex_m::asm::wfi(); // Can stay here forever, should add a countdown timer

        let mut received_message: [u8; BUFF_SIZE] = [0; BUFF_SIZE];
        for i in 0..BUFF_SIZE {
            unsafe {
                received_message[i] = BUFFER[i];
            }
        }
        received_message
    }
}

#[interrupt]
fn USART2() {
    // When a packet is received, there is at least 3 bytes : header, header, packet size
    let mut packet_size = 3;

    unsafe {
        cortex_m::interrupt::free(|_| {
            if let Some(rx) = Rx.as_mut() {
                // If it received a packet and we have not read it entirely yet
                while rx.is_rxne() || BUFF_INDEX < packet_size {
                    // Read the byte
                    if let Ok(w) = rx.read() {
                        BUFFER[BUFF_INDEX] = w; // Fill the buffer

                        // If we read the packet size in the received packet
                        // it updates our packet_size to read all bytes from the received packet
                        if BUFF_INDEX == 2 {
                            packet_size = w as usize;
                        }

                        // If the buffer is full, it rewrites at the beginning
                        if BUFF_INDEX >= BUFF_SIZE - 1 {
                            BUFF_INDEX = 0;
                        }

                        BUFF_INDEX += 1;
                    }
                }
                BUFF_INDEX = 0;
            };
        })
    }
}

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

    let mut usart = usart::Serial::usart2(
        dp.USART2,
        (pin_tx),
        (pin_rx),
        FullConfig::default(),
        (&mut rcc),
    ).unwrap();

    loop {
        hprintln!("Begin of loop");
        delay_tim2.delay_ms(1000 as u16);
        hprintln!("Sent data");
        usart.write(6).unwrap();
    }
}
