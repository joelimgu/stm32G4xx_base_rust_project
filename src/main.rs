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
use hal::prelude::*;
use core::num::{NonZeroU16, NonZeroU8};

use cortex_m_rt::entry;

//use log::hprintln;
use panic_halt as _;

use cortex_m_semihosting::hprintln;
use embedded_hal::can::Id;
use stm32g4xx_hal::fdcan::config::{ClockDivider, DataBitTiming, FdCanConfig, FrameTransmissionConfig, GlobalFilter, Interrupts, TimestampSource};
use stm32g4xx_hal::fdcan::filter::{Action, Filter, FilterType};
use stm32g4xx_hal::fdcan::id::Id::Standard;


#[macro_use]
mod utils;


unsafe fn get_can_id_from_buff(b: &&[u32]) -> u32 {
    let b = *b;
    let id = *&b as *const _;
    let ptr = id as *const () as usize;
    let header = ptr - 2*4; // two times the register size(4 bytes)
    let ptr2 = header as *const u32;
    // buena suerte
    let id2 = unsafe { *(header as *const u32) };
    // equivalent to 00011111111111000000000000000000
    // i.e the first 11 bits of the ID as we only want the standard ID
    // pag 1961 of rm0440-stm32
    let m_id = (id2 & 0x1FFC0000) >> 18;
    m_id
}

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

        let configCAN = FdCanConfig {
            nbtr: btr,
            dbtr: DataBitTiming::default(),
            automatic_retransmit: true,
            transmit_pause: true,
            frame_transmit: FrameTransmissionConfig::ClassicCanOnly,
            non_iso_mode: false,
            edge_filtering: false,
            protocol_exception_handling: false,
            clock_divider: ClockDivider::_1,
            interrupt_line_config: Interrupts::none(),
            timestamp_source: TimestampSource::None,
            global_filter: GlobalFilter::default(),
        };

        //can.set_protocol_exception_handling(false);
        //can.set_non_iso_mode(false);

        //hprintln!("-- Configure nominal timing");
        //can.set_nominal_bit_timing(btr);


       // global_filter =

        //can.set_global_filter(GlobalFilter::reject_all());

        can.apply_config(configCAN);

        let filtre = StandardFilter {
            filter: FilterType::DedicatedSingle(StandardId::new(0x2).unwrap()),
            action: Action::StoreInFifo0,
        };

        //hprintln!("-- Configure Filters");
        can.set_standard_filter(
            StandardFilterSlot::_0,
            filtre,
        );

        // hprintln!("-- Current Config: {:#?}", can.get_config());

      //  hprintln!("-- Set CAN1 in to normal mode");
        // can.into_external_loopback()
        can.into_normal()
    };

    let mut can = can1;

    //hprintln!("Create Message Data");
    let mut buffer = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0];
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
    let mut id = 0;
    let mut buff_len = 0;
    if let Ok(rx_header) = block!(can.receive0(&mut |h, b| {
            unsafe {
                hprintln!("Received ID: {:#X?}", get_can_id_from_buff(&b));
            }
            id = unsafe { get_can_id_from_buff(&b) };
            hprintln!("received data: {:X?}", &b);

            // unsafe {
            //     hprintln!("ID: {}", *((0x4000A400+0x00B0_u32).as_ptr()));
            // }
            // buffer[-2]
            // buffer_len = 0;
            for (i, d) in b.iter().enumerate() {
                buffer[i] = *d;
                // buffer_len += 1;
            }
            h
        })) {
        const taille: u8 = 1;
        let id_origin = id & 0xF;
        let id_destination = (id >> 4) & 0xF;
        let prio = id >> 8;
        let new_id = (prio << 8) | (id_origin << 4) | id_destination;
        let tx_frame_header = TxFrameHeader {
            len: taille,
            frame_format: FrameFormat::Standard,
            id: Standard(StandardId::new(new_id as u16).unwrap()).into(),
            bit_rate_switching: false,
            marker: None,
        };
        // let mess: [u32; taille as usize] = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0];
        block!(
                can.transmit(tx_frame_header, &mut |b| {
                    let len = b.len();
                    b[..len].clone_from_slice(&buffer[..(len)]);
                    hprintln!("Transmit: ID {:X?}: {:X?}", new_id, b);
                })
            )
            .unwrap();
    }


    }
}
