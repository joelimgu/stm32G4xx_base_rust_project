#![no_main]
#![no_std]

use crate::hal::{
    fdcan::{
        config::NominalBitTiming,
        filter::{StandardFilter, StandardFilterSlot},
        frame::{FrameFormat, TxFrameHeader},
        id::StandardId,
        FdCan,
    },
    gpio::{GpioExt as _, Speed},
    nb::block,
    rcc::{Config, RccExt, SysClockSrc},
    stm32::Peripherals,
    time::U32Ext,
};
use stm32g4xx_hal as hal;

use core::num::{NonZeroU16, NonZeroU8};

use cortex_m_rt::entry;

//use log::hprintln;

use cortex_m_semihosting::hprintln;
use stm32g4xx_hal::fdcan::config::GlobalFilter;
use stm32g4xx_hal::fdcan::filter::{Action, Filter, FilterType};


#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    hprintln!("Start");

    // APB1 (HSI): 16MHz, Bit rate: 125kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.hprintln/
    // TODO: use the can_bit_timings crate
    let btr = NominalBitTiming {
        prescaler: NonZeroU16::new(8).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    hprintln!("Init Clocks");

    let dp = Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::new(SysClockSrc::HSI));

    hprintln!("Split GPIO");

    let gpioa = dp.GPIOA.split(&mut rcc);

    let can1 = {
        //hprintln!("Init CAN 1");
        let rx = gpioa.pa11.into_alternate().set_speed(Speed::VeryHigh);
        let tx = gpioa.pa12.into_alternate().set_speed(Speed::VeryHigh);

        //hprintln!("-- Create CAN 1 instance");
        let can = FdCan::new(dp.FDCAN1, tx, rx, &rcc);

        //hprintln!("-- Set CAN 1 in Config Mode");
        let mut can = can.into_config_mode();
        can.set_protocol_exception_handling(false);

        //hprintln!("-- Configure nominal timing");
        can.set_nominal_bit_timing(btr);

        global_filter =

        can.set_global_filter(GlobalFilter::reject_all());

        let filtre = StandardFilter {
            filter: FilterType::DedicatedSingle(StandardId::new(4).unwrap()),
            action: Action::StoreInFifo0,
        };

        //hprintln!("-- Configure Filters");
        can.set_standard_filter(
            StandardFilterSlot::_0,
            filtre,
        );

        hprintln!("-- Current Config: {:#?}", can.get_config());

      //  hprintln!("-- Set CAN1 in to normal mode");
        // can.into_external_loopback()
        can.into_normal()
    };

    let mut can = can1;

    //hprintln!("Create Message Data");
    let mut buffer = [0xAAAAAAAA, 0xFFFFFFFF, 0x0, 0x0, 0x0, 0x0];
    //hprintln!("Create Message Header");
    // let header = TxFrameHeader {
    //     len: 2 * 4,
    //     id: StandardId::new(0x1).unwrap().into(),
    //     frame_format: FrameFormat::Standard,
    //     bit_rate_switching: false,
    //     marker: None,
    // };
    //hprintln!("Initial Header: {:#X?}", &header);

    //hprintln!("Transmit initial message");
    // block!(can.transmit(header, &mut |b| {
    //     let len = b.len();
    //     b[..len].clone_from_slice(&buffer[..len]);
    // },))
    //     .unwrap();

    loop {
        if let Ok(rxheader) = block!(can.receive0(&mut |h, b| {
            hprintln!("Received Header: {:#X?}", &h);
            hprintln!("received data: {:X?}", &b);

            for (i, d) in b.iter().enumerate() {
                buffer[i] = *d;
            }
            h
        })) {
            block!(
                can.transmit(rxheader.unwrap().to_tx_header(None), &mut |b| {
                    let len = b.len();
                    b[..len].clone_from_slice(&buffer[..len]);
                    hprintln!("Transmit: {:X?}", b);
                })
            )
                .unwrap();
        }
    }
}