// #![deny(warnings)]
// #![deny(unsafe_code)]
#![no_main]
#![no_std]

mod dpwmmin_table;
mod can_api;

use bincode::error::DecodeError;
use core::f32::consts::PI;
use core::mem::MaybeUninit;
use core::num::{NonZeroU16, NonZeroU8};
use core::ops::Rem;
use cortex_m::delay::Delay;
use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;
extern crate alloc;
extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::rcc::PllMDiv::{DIV_4, DIV_8};
use stm32g4xx_hal::rcc::PllNMul::MUL_75;
use stm32g4xx_hal::rcc::PllRDiv::DIV_2;
use stm32g4xx_hal::rcc::PllSrc::HSI;
use stm32g4xx_hal::rcc::{Config, FdCanClockSource, PllConfig, PllPDiv, Rcc, SysClockSrc};

use crate::can_api::{ApiEncodeDecode, AqueductResponseFrame};
use fdcan::config::{DataBitTiming, NominalBitTiming};
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use fdcan::frame::{FrameFormat, RxFrameInfo, TxFrameHeader};
use fdcan::id::Id::Standard;
use fdcan::id::{Id, StandardId};
use hal::adc::AdcClaim;
use hal::dma::{config::DmaConfig, stream::DMAExt};
use stm32g4xx_hal::adc::config::{
    Clock, ClockMode, Continuous, Dma, SampleTime, Sequence, TriggerMode,
};
use stm32g4xx_hal::adc::{ClockSource, Vref};
use stm32g4xx_hal::dma::TransferExt;
use stm32g4xx_hal::signature::VrefCal;
// use stm32g4xx_hal::gpio::Speed;
use embedded_alloc::LlffHeap as Heap;
use fdcan::ReceiveOverrun;
use hal::pwm::PwmAdvExt;
use stm32g4xx_hal::comparator::{self, ComparatorExt, ComparatorSplit};
use stm32g4xx_hal::dac::{Dac1IntSig1, Dac1IntSig2, Dac2IntSig1, DacExt, DacOut};

use crate::dpwmmin_table::ASSINUS_TABLE;
use num_traits::real::Real;
use stm32g4xx_hal::adc::config::ExternalTrigger12::{Tim_1_cc_1, Tim_1_trgo};
use stm32g4xx_hal::gpio::Speed;
use stm32g4xx_hal::pwm::Polarity;
use stm32g4xx_hal::pwr::{PowerConfiguration, PwrExt};
use stm32g4xx_hal::rcc::PllPDiv::DIV_9;
use stm32g4xx_hal::time::{ExtU32, NanoSecond, RateExtU32};

fn configure_clock(rcc: Rcc, pwr_cfg: PowerConfiguration) -> Rcc {
    let mut pll_config = PllConfig::default();
    pll_config.m = DIV_4;
    pll_config.mux = HSI;
    pll_config.n = MUL_75;
    pll_config.r = Some(DIV_2);
    // pll_config.p = Some(PllPDiv::DIV_28);

    let rcc = rcc.freeze(
        Config::new(SysClockSrc::PLL)
            .pll_cfg(pll_config)
            .fdcan_src(FdCanClockSource::PCLK),
        pwr_cfg,
    );
    rcc
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    const HEAP_SIZE: usize = 512;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let _cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    dp.RCC.apb2enr.write(|w| w.syscfgen().set_bit());

    let pwr = dp.PWR.constrain();
    let pwr_cfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut rcc = configure_clock(rcc, pwr_cfg);

    let mut delay = Delay::new(_cp.SYST, rcc.clocks.ahb_clk.to_Hz());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpioc.pc6.into_push_pull_output();

    let mut can = {
        let rx = gpioa.pa11.into_alternate::<9u8>(); //.set_speed(Speed::VeryHigh);
        let tx = gpiob.pb9.into_alternate::<9u8>(); //.set_speed(Speed::VeryHigh);

        let mut can = dp.FDCAN1.fdcan(tx, rx, &rcc);

        can.set_protocol_exception_handling(false);

        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(10).unwrap(),
            seg2: NonZeroU8::new(4).unwrap(),
            sync_jump_width: NonZeroU8::new(3).unwrap(),
        };

        can.set_nominal_bit_timing(btr);

        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        can.into_normal()
    };

    let pin_out1 = gpioa.pa8.into_alternate::<6u8>();
    let pin_out2 = gpioa.pa9.into_alternate::<6u8>();
    let pin_out3 = gpioa.pa10.into_alternate::<6u8>();

    let pin_out1n = gpioc.pc13.into_alternate::<4u8>();
    let pin_out2n = gpioa.pa12.into_alternate::<6u8>();
    let pin_out3n = gpiob.pb15.into_alternate::<4u8>();

    let shunt1_voltage_p = gpioa.pa1.into_analog();
    let shunt1_voltage_n = gpioa.pa3.into_analog();
    let shunt1_voltage_out = gpioa.pa2.into_analog();

    let shunt2_voltage_out = gpioa.pa6;

    let shunt3_voltage_p = gpiob.pb0.into_analog();
    let shunt3_voltage_n = gpiob.pb2.into_analog();
    let shunt3_voltage_out = gpiob.pb1.into_analog();

    dp.OPAMP.opamp1_csr.write(|w| {
        w.vp_sel()
            .vinp0()
            .vm_sel()
            .pga()
            .pga_gain()
            .gain16_input_vinm0()
            .opaintoen()
            .output_pin()
            .opaen()
            .enabled()
    });

    dp.OPAMP.opamp3_csr.write(|w| {
        w.vp_sel()
            .vinp0()
            .vm_sel()
            .pga()
            .pga_gain()
            .gain16_input_vinm0()
            .opaintoen()
            .output_pin()
            .opaen()
            .enabled()
    });

    let dac1 = dp.DAC1.constrain((gpioa.pa4, Dac1IntSig1), &mut rcc);
    let mut dac1 = dac1.enable();

    dac1.set_value(850);

    let (comp1, comp2, comp3, comp4, ..) = dp.COMP.split(&mut rcc);

    let comp1 = comp1
        .comparator(
            &shunt1_voltage_p,
            &dac1,
            comparator::Config::default().hysteresis(comparator::Hysteresis::None),
            &rcc.clocks,
        )
        .enable();

    // let comp2 = comp2.comparator(
    //     &shunt2_voltage_p,
    //     &dac2,
    //     comparator::Config::default().hysteresis(comparator::Hysteresis::None),
    //     &rcc.clocks,
    // );

    let comp4 = comp4
        .comparator(
            &shunt3_voltage_p,
            &dac1,
            comparator::Config::default().hysteresis(comparator::Hysteresis::None),
            &rcc.clocks,
        )
        .enable();

    let pins = (pin_out1, pin_out2, pin_out3);

    let (mut tim1_control, (c1, c2, c3)) = dp
        .TIM1
        .pwm_advanced(pins, &mut rcc)
        .frequency(50.kHz())
        .center_aligned()
        .with_deadtime(200.nanos())
        .finalize();

    let tim1_hack = unsafe { &*stm32::TIM1::ptr() };

    let mut c1 = c1.into_complementary(pin_out1n);
    let mut c2 = c2.into_complementary(pin_out2n);
    let mut c3 = c3.into_complementary(pin_out3n);

    c1.set_duty(0);
    c2.set_duty(0);
    c3.set_duty(0);

    delay.delay_ms(100);

    unsafe {
        tim1_hack.bdtr.modify(|r, w| {
            w.bkf()
                .bits(2)
                .aoe()
                .clear_bit()
                .bke()
                .set_bit()
                .bkp()
                .set_bit()
                .moe()
                .set_bit()
        });
        tim1_hack
            .af1
            .write(|w| w.bkcmp1e().set_bit().bkcmp1p().clear_bit());
    }

    delay.delay_ms(100);

    unsafe { tim1_hack.bdtr.modify(|r, w| w.moe().set_bit()) };

    let vcc_voltage = gpioa.pa0.into_analog();
    let _ntc_voltage = gpiob.pb14.into_analog();

    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .half_transfer_interrupt(true)
        .circular_buffer(true)
        .memory_increment(true);

    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    adc.enable_vref(&dp.ADC12_COMMON);
    adc.set_external_trigger((TriggerMode::FallingEdge, Tim_1_trgo));
    adc.set_clock_mode(ClockMode::Synchronous_Div_2);
    adc.set_clock(Clock::Div_2);
    adc.reset_sequence();
    adc.configure_channel(&Vref, Sequence::One, SampleTime::Cycles_6_5);
    adc.configure_channel(&vcc_voltage, Sequence::Two, SampleTime::Cycles_6_5);
    adc.configure_channel(&shunt1_voltage_p, Sequence::Three, SampleTime::Cycles_6_5);
    adc.configure_channel(&shunt3_voltage_out, Sequence::Four, SampleTime::Cycles_6_5);

    let first_buffer = cortex_m::singleton!(: [u16; 100] = [0; 100]).unwrap();
    let mut transfer = streams.0.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(Dma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    //enable update trigger
    unsafe { tim1_hack.cr2.modify(|r, w| w.mms().bits(0b0111)) }

    let mut vdda = 0u16; // mV
                         //let mut _ntc_temp = 0f32;
    let mut vcc = 0u16; // mV
    let mut current1 = 0i32; // mA
    let mut current3 = 0i32;

    let mut counter = 0u32;
    let mut flag = false;

    let mut phase_orientation = 0u16;

    fn assinus(orientation: u16) -> f32 {
        if orientation > 4095 {
            0_f32
        } else if orientation < 1366 {
            ASSINUS_TABLE[orientation as usize]
        } else if (4096 - orientation) < 1366 {
            ASSINUS_TABLE[4096_usize - orientation as usize]
        } else {
            0_f32
        }
    }

    fn phase_pwm(orientation: u16, max_pwm: u16, torque: f32) -> u16 {
        if torque > 1_f32 || torque < 0_f32 {
            0_u16
        } else {
            let mut value: f32 = assinus(orientation);

            if value != 0_f32 {
                value = value * 0.25_f32 * torque;
            };

            (value * max_pwm as f32) as u16
        }
    }

    led.set_high();

    unsafe {
        tim1_hack.ccr4().write(|w| w.ccr().bits(1400));
        tim1_hack.ccmr2_output().modify(|r, w| w.oc4m().pwm_mode1());
        tim1_hack.ccmr2_output().modify(|r, w| w.oc4pe().set_bit());
        tim1_hack.ccer.modify(|r, w| w.cc4e().set_bit());
    }

    c1.enable();
    c2.enable();
    c3.enable();

    let mut can_rx_buffer = [0u8; 8];
    let encoder_zero = 283u16;
    let poles_pairs = 7u16;

    let mut current_orientation = Some(0u16);
    let mut set_point = 0u16;
    let mut last_orientation = 0u16;
    let mut velocity = 0u16;
    let mut orientation_integral = 0i64;
    let max_orientation_integral = 2048_i64 * 5_i64 * 50_i64; // Max error max out integral in 2ms

    let p_gain: f32 = 1_f32 / 300_f32;
    let i_gain: f32 = 1_f32 / (max_orientation_integral as f32); // max i value = 1
    let d_gain: f32 = 0_f32;

    fn orientation_delta(o1: u16, o2: u16) -> i16 {
        let tmp: i16 = o2 as i16 - o1 as i16;
        if tmp > 2047_i16 {
            tmp - 4096_i16
        } else if tmp < -2048_i16 {
            tmp + 4096_i16
        } else {
            tmp
        }
    }

    let mut pid = move |current_orientation: u16, set_point: u16| -> f32 {
        let delta = orientation_delta(current_orientation, set_point);
        orientation_integral += delta as i64;
        if orientation_integral > max_orientation_integral {
            orientation_integral = max_orientation_integral
        }
        if orientation_integral < -max_orientation_integral {
            orientation_integral = -max_orientation_integral
        }

        let p = p_gain * delta as f32;
        let i = i_gain * orientation_integral as f32;

        //TODO: velocity and d term

        let mut ans = p + i;
        if ans > 1_f32 {
            1_f32
        } else if ans < -1_f32 {
            -1_f32
        } else {
            ans
        }
    };

    loop {
        if transfer.elements_available() >= 4 {
            counter += 1;

            if !tim1_hack.bdtr.read().moe().bit() {
                counter += 10
            }

            let mut b = [0u16; 4];
            transfer.read_exact(&mut b);

            match current_orientation {
                Some(current_orientation) => {
                    let mut torque = pid(current_orientation, set_point);
                    let phase_shift_half_pi = 1024;
                    phase_orientation = (4096 - current_orientation + encoder_zero) * poles_pairs;
                    if torque >= 0_f32 {
                        phase_orientation -= phase_shift_half_pi;
                    } else {
                        phase_orientation += phase_shift_half_pi;
                        torque = -torque;
                    }
                    phase_orientation = phase_orientation.rem(4096);


                    let max_duty = c1.get_max_duty();
                    let mut phase_orientation2 = phase_orientation + 4096 / 3;
                    let mut phase_orientation3 = phase_orientation + 8192 / 3;
                    if phase_orientation2 > 4095 {
                        phase_orientation2 -= 4096;
                    }
                    if phase_orientation3 > 4095 {
                        phase_orientation3 -= 4096;
                    }
                    let pwm1 = phase_pwm(phase_orientation, max_duty, torque);
                    let pwm2 = phase_pwm(phase_orientation2, max_duty, torque);
                    let pwm3 = phase_pwm(phase_orientation3, max_duty, torque);

                    c1.set_duty(pwm1);
                    c2.set_duty(pwm2);
                    c3.set_duty(pwm3);
                }
                None => {
                    c1.set_duty(0);
                    c2.set_duty(0);
                    c3.set_duty(0);
                }
            }

            if counter > 50000 {
                if flag {
                    led.set_high()
                } else {
                    led.set_low()
                };
                flag = !flag;

                counter = 0;

                vdda = (3000 * VrefCal::get().read() as u32 / b[0] as u32) as u16;
                vcc = Vref::sample_to_millivolts_ext(
                    b[1],
                    vdda as u32 * 187 / 18,
                    hal::adc::config::Resolution::Twelve,
                );
                // current1 = Vref::sample_to_millivolts_ext(
                //     b[2],
                //     vdda as u32,
                //     hal::adc::config::Resolution::Twelve,
                // ) as i32;

                current1 = b[2] as i32;

                current3 = Vref::sample_to_millivolts_ext(
                    b[3],
                    vdda as u32,
                    hal::adc::config::Resolution::Twelve,
                ) as i32;

                let divider = 220_i64 * 22_i64 + 220_i64 * 15_i64 + 22_i64 * 15_i64;

                let bias = 22_i64 * (15_i64 * vdda as i64) * 16_i64 / divider;

                // current1 = ((current1 as i64 - bias) * 1000_i64 * divider
                //     / (3_i64 * 16_i64 * 220_i64 * 22_i64)) as i32;
                //current3 = (current3 - bias) * (220 * 22) * 1000 / (divider * 3 * 16);

                let data1 = AqueductResponseFrame::ServoState {
                    vcc,
                    current1: current1,
                };

                let v = data1.api_encode().unwrap();
                let d = v.as_slice();

                let frame_header = TxFrameHeader {
                    len: v.len() as u8,
                    frame_format: FrameFormat::Standard,
                    id: Standard(StandardId::new(1).unwrap()),
                    bit_rate_switching: false,
                    marker: None,
                };
                let _ = can.transmit_preserve(frame_header, &d, &mut |_, _, _| {});
                // unwrap
            }
        }

        match can.receive0(&mut can_rx_buffer) {
            Ok(frame) => {
                let frame = frame.unwrap();
                match frame.id {
                    Standard(id) if id.as_raw() == 10 && frame.len >= 7 => {
                        match AqueductResponseFrame::api_decode(&can_rx_buffer) {
                            Ok(AqueductResponseFrame::State {
                                button,
                                pump,
                                weight,
                            }) => {
                                if weight >= 0 && weight <= 4096 {
                                    current_orientation = Some(weight as u16);
                                } else {
                                    current_orientation = None
                                };
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
            Err(_) => {}
        }
    }
}
