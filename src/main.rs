// #![deny(warnings)]
// #![deny(unsafe_code)]
#![no_main]
#![no_std]

mod can_api;
mod dpwmmin_table;

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
use stm32g4xx_hal::can::{Can, CanExt};
use stm32g4xx_hal::rcc::PllMDiv::DIV_4;
use stm32g4xx_hal::rcc::PllNMul::MUL_75;
use stm32g4xx_hal::rcc::PllRDiv::DIV_2;
use stm32g4xx_hal::rcc::PllSrc::HSI;
use stm32g4xx_hal::rcc::{Config, FdCanClockSource, PllConfig, Rcc, SysClockSrc};

use crate::can_api::*;
use fdcan::config::NominalBitTiming;
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use fdcan::frame::{FrameFormat, TxFrameHeader};
use fdcan::id::Id::Standard;
use fdcan::id::StandardId;
// use stm32g4xx_hal::gpio::Speed;
use embedded_alloc::LlffHeap as Heap;
use fdcan::{FdCan, NormalOperationMode};
use hal::hal_02::PwmPin;
use hal::pwm::PwmAdvExt;

use crate::dpwmmin_table::DPWMMIN_TABLE;
use stm32g4xx_hal::hal::spi;
use stm32g4xx_hal::pac::{FDCAN1, RCC};
use stm32g4xx_hal::pwr::{PowerConfiguration, PwrExt};
use stm32g4xx_hal::time::{ExtU32, RateExtU32};

fn configure_clock(rcc: Rcc, pwr_cfg: PowerConfiguration) -> Rcc {
    let pll_config = PllConfig {
        // 150 MHz
        m: DIV_4,
        mux: HSI,
        n: MUL_75,
        r: Some(DIV_2),
        ..PllConfig::default()
    };

    rcc.freeze(
        Config::new(SysClockSrc::PLL)
            .pll_cfg(pll_config)
            .fdcan_src(FdCanClockSource::PCLK),
        pwr_cfg,
    )
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

const PHASE_SHIFT_HALF_PI: u32 = 4096;

#[entry]
fn main() -> ! {
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let _cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    dp.RCC.apb2enr().write(|w| w.syscfgen().set_bit());

    let pwr = dp.PWR.constrain();
    let pwr_cfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut rcc = configure_clock(rcc, pwr_cfg);

    let _delay = Delay::new(_cp.SYST, rcc.clocks.ahb_clk.to_Hz());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let _gpioc = dp.GPIOC.split(&mut rcc);
    let gpiof = dp.GPIOF.split(&mut rcc);

    let mut can = {
        let rx = gpioa.pa11.into_alternate::<9u8>(); //.set_speed(Speed::VeryHigh);
        let tx = gpioa.pa12.into_alternate::<9u8>(); //.set_speed(Speed::VeryHigh);

        let mut can = dp.FDCAN1.fdcan(tx, rx, &mut rcc);

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

    // let mut driver_gain = gpiob.pb6.into_push_pull_output();
    // let mut driver_slew = gpiob.pb7.into_push_pull_output();
    // driver_gain.set_low().unwrap();
    // driver_slew.set_low().unwrap();

    let mut driver_nsleep = gpioa.pa3.into_push_pull_output();
    driver_nsleep.set_high();

    let pin_out1 = gpioa.pa8.into_alternate::<6u8>();
    let pin_out2 = gpioa.pa9.into_alternate::<6u8>();
    let pin_out3 = gpioa.pa10.into_alternate::<6u8>();

    let pin_out1n = gpioa.pa7.into_alternate::<6u8>();
    let pin_out2n = gpiob.pb0.into_alternate::<6u8>();
    let pin_out3n = gpiof.pf0.into_alternate::<6u8>();

    let pins = (pin_out1, pin_out2, pin_out3);

    let (_tim1_control, (c1, c2, c3)) = dp
        .TIM1
        .pwm_advanced(pins, &mut rcc)
        .frequency(50.kHz())
        .center_aligned()
        .with_deadtime(100.nanos())
        .finalize();

    // let tim1_hack = unsafe { &*stm32::TIM1::ptr() };

    let mut c1 = c1.into_complementary(pin_out1n);
    let mut c2 = c2.into_complementary(pin_out2n);
    let mut c3 = c3.into_complementary(pin_out3n);

    let max_duty = c1.get_max_duty() as f32;

    c1.set_duty(0);
    c2.set_duty(0);
    c3.set_duty(0);

    fn dpwmmin(orientation: u32) -> f32 {
        if orientation > 16383 {
            0_f32
        } else if orientation < 5462 {
            DPWMMIN_TABLE[orientation as usize]
        } else if (16384 - orientation) < 5462 {
            DPWMMIN_TABLE[16384_usize - orientation as usize]
        } else {
            0_f32
        }
    }

    fn phase_pwm(orientation: u32, max_pwm: u16, torque: f32) -> u16 {
        if !(0_f32..=1_f32).contains(&torque) {
            0_u16
        } else {
            let mut value: f32 = dpwmmin(orientation);

            if value != 0_f32 {
                value = value * 0.5_f32 * torque;
            };

            (value * max_pwm as f32) as u16
        }
    }

    c1.enable();
    c2.enable();
    c3.enable();

    // TIM2

    let rcc_hack = unsafe { &(*RCC::ptr()) };
    let sensor_read_timer = dp.TIM2;
    rcc_hack.apb1enr1().modify(|_, w| w.tim2en().set_bit());
    sensor_read_timer.dier().modify(|_, w| w.uie().set_bit());
    sensor_read_timer
        .psc()
        .modify(|_, w| unsafe { w.psc().bits(0) });
    sensor_read_timer
        .arr()
        .modify(|_, w| unsafe { w.arr().bits(15000 - 1) }); //10 kHz

    let sclk = gpiob.pb3.into_alternate();
    let miso = gpiob.pb4.into_alternate();
    let mosi = gpiob.pb5.into_alternate();
    let mut cs_pin = gpioa.pa4.into_push_pull_output();
    cs_pin.set_high();

    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_2, 4.MHz(), &mut rcc);

    let mut can_rx_buffer = [0u8; 8];

    let mut master_address = 0u16;
    let mut slave_address = 0u16;

    let mut chip_id1_received = false;
    let mut chip_id2_received = false;

    let uid_address: *const u8 = 0x1FFF_7590 as *const u8;
    let uid = unsafe { core::ptr::read_volatile(uid_address as *const [u8; 12]) };

    let chip_id1: [u8; 6] = uid[0..6].try_into().unwrap();
    let chip_id2: [u8; 6] = uid[6..12].try_into().unwrap();

    let mut data_rate: u32 = 1;
    let mut duty_cycle_limit = 1.0f32;
    let mut reverse_motor = false;
    let mut velocity_iir_filter_gain = 0.97_f32;
    let mut inverse_velocity_iir_filter_gain = 1.0 - velocity_iir_filter_gain;

    let mut position_low_pass_gain = 1.0f32;
    let mut filtered_set_point = 0u16;
    let mut filtered_set_point_f32 = 0_f32;

    let mut encoder_zero = 0u32; //950u32;
    let mut pole_pairs = 7u32;

    let mut current_command: ServoCommandFrame = ServoCommandFrame::Disable;
    let mut current_orientation = Some(0u16);
    let mut orientation_processed = true;

    let mut previous_orientation = 0u16;
    let mut velocity = 0f32;
    let current = 0i16;
    let mut position_integral = 0i64;
    let mut max_position_integral = 8192_i64 * 2000;

    let mut max_velocity: f32 = 1_f32;
    let mut velocity_integral = 0_f32;
    let mut velocity_integral_cycles_to_max_out: u16 = 2000;
    let mut max_velocity_integral: f32 = max_velocity * velocity_integral_cycles_to_max_out as f32;

    let mut position_p_gain: f32 = 1_f32 / 500_f32;
    let mut position_i_gain: f32 = 1_f32 / (max_position_integral as f32); // 1_f32 / (max_orientation_integral as f32); // max i value = 1
    let mut velocity_p_gain: f32 = 1_f32 / 20_f32;
    let mut velocity_i_gain: f32 = 1_f32;

    fn orientation_delta(o1: u16, o2: u16) -> i32 {
        let tmp: i32 = o2 as i32 - o1 as i32;
        if tmp > 8191_i32 {
            tmp - 16384_i32
        } else if tmp < -8192_i32 {
            tmp + 16384_i32
        } else {
            tmp
        }
    }

    macro_rules! pid {
        ($current_orientation:expr,$set_point:expr) => {{
            {
                let setpoint_delta = orientation_delta(filtered_set_point, $set_point);
                let setpoint_increment_f32 =
                    (setpoint_delta as f32) * position_low_pass_gain + filtered_set_point_f32;
                let setpoit_increment = setpoint_increment_f32 as i16;
                filtered_set_point_f32 = setpoint_increment_f32 - setpoit_increment as f32;

                let new_filtered_set_point = filtered_set_point as i16 + setpoit_increment;

                if new_filtered_set_point < 0 {
                    filtered_set_point = (new_filtered_set_point + 16384_i16) as u16;
                } else {
                    filtered_set_point = new_filtered_set_point.rem_euclid(16384_i16) as u16;
                };
            }

            let delta = orientation_delta($current_orientation, filtered_set_point);
            position_integral += delta as i64;
            position_integral =
                position_integral.clamp(-max_position_integral, max_position_integral);

            let p = position_p_gain * delta as f32;
            let i = position_i_gain * position_integral as f32;

            let velocity_delta = (p + i).clamp(-max_velocity, max_velocity) - velocity;
            velocity_integral += velocity_delta;
            velocity_integral =
                velocity_integral.clamp(-max_velocity_integral, max_velocity_integral);

            let p = velocity_p_gain * velocity_delta;
            let i = velocity_i_gain * velocity_integral;

            (p + i).clamp(-1.0, 1.0)
        }};
    }

    trait ApiTransmitter {
        fn can_transmit<T: ApiEncodeDecode>(
            &mut self,
            general_channel: bool,
            slave_address: u16,
            data: &T,
        );
    }
    impl ApiTransmitter for FdCan<Can<FDCAN1>, NormalOperationMode> {
        fn can_transmit<T: ApiEncodeDecode>(
            &mut self,
            general_channel: bool,
            slave_address: u16,
            data: &T,
        ) {
            if general_channel || (slave_address > 1 && slave_address < 2048) {
                let mut address = slave_address;
                if general_channel {
                    address = 1
                }

                let data_vec = data.api_encode().unwrap();
                let data = data_vec.as_slice();

                self.transmit(
                    TxFrameHeader {
                        len: data.len() as u8,
                        frame_format: FrameFormat::Standard,
                        id: Standard(StandardId::new(address).unwrap()),
                        bit_rate_switching: false,
                        marker: None,
                    },
                    data,
                )
                .unwrap();
            }
        }
    }
    sensor_read_timer.cr1().modify(|_, w| w.cen().set_bit());

    let mut cs_pin_set_time = None;

    let mut counter = 0u32;

    loop {
        if sensor_read_timer.sr().read().uif().bit_is_set() {
            sensor_read_timer.sr().modify(|_, w| w.uif().clear_bit());
            cs_pin.set_low();
            cs_pin_set_time = Some(sensor_read_timer.cnt().read().cnt().bits());
        }

        match cs_pin_set_time {
            None => {}
            Some(t) if sensor_read_timer.cnt().read().cnt().bits() > t + 30 => {
                //TODO: maybe use interrupts here instead
                //200ns @ 150MHz
                cs_pin_set_time = None;
                let mut buffer = [0u8; 2];
                spi.transfer_in_place(buffer.as_mut_slice()).unwrap();
                current_orientation =
                    Some((((buffer[0] & 0b01111111u8) as u16) << 7) | (buffer[1] as u16 >> 1));
                cs_pin.set_high();

                let delta = orientation_delta(previous_orientation, current_orientation.unwrap());
                previous_orientation = current_orientation.unwrap();
                velocity = velocity * velocity_iir_filter_gain
                    + inverse_velocity_iir_filter_gain * delta as f32;
                orientation_processed = false;

                if counter > 10000 / data_rate {
                    counter = 0;
                    let data = current_orientation
                        .map(|position| ServoResponseFrame::State {
                            sensor_detected: true,
                            position,
                            velocity: (velocity / 0.0016).clamp(i16::MIN as f32, i16::MAX as f32)
                                as i16, // 1024 = 1rps
                            current,
                        })
                        .unwrap_or(ServoResponseFrame::State {
                            sensor_detected: false,
                            position: 0,
                            velocity: 0,
                            current: 0,
                        });

                    can.can_transmit(false, slave_address, &data);
                } else {
                    counter += 1;
                }
            }
            Some(_) => {}
        }

        if !orientation_processed {
            orientation_processed = true;
            match current_orientation {
                Some(current_orientation) => match current_command {
                    ServoCommandFrame::Brake => {
                        c1.set_duty(0);
                        c2.set_duty(0);
                        c3.set_duty(0);
                        c1.enable();
                        c2.enable();
                        c3.enable();
                    }
                    ServoCommandFrame::BrushedForceDutyCycle { .. }
                    | ServoCommandFrame::BrushedHoldPosition { .. } => {
                        let torque = match current_command {
                            ServoCommandFrame::BrushedForceDutyCycle { duty_cycle } => duty_cycle,
                            ServoCommandFrame::BrushedHoldPosition { position } => {
                                pid!(current_orientation, position)
                            }
                            _ => 0.0,
                        }
                        .clamp(-1.0, 1.0)
                            * duty_cycle_limit;

                        let (d1, d2) = if (torque > 0.0) ^ reverse_motor {
                            (0, (torque * max_duty).abs() as u16)
                        } else {
                            ((torque * max_duty).abs() as u16, 0)
                        };
                        c3.disable();
                        c1.set_duty(d1);
                        c2.set_duty(d2);
                        c1.enable();
                        c2.enable();
                    }
                    ServoCommandFrame::BrushlessHoldPosition { .. }
                    | ServoCommandFrame::BrushlessForcePosition { .. } => {
                        let (phase_orientation, torque) = match current_command {
                            ServoCommandFrame::BrushlessForcePosition { position, power } => {
                                if position < 16384 {
                                    (Some(position as u32), power.clamp(0.0, 1.0))
                                } else {
                                    (None, 0.0)
                                }
                            }
                            ServoCommandFrame::BrushlessHoldPosition { position } => {
                                let mut torque = pid!(current_orientation, position);
                                let mut phase_orientation_raw =
                                    (16384 + current_orientation as u32 - encoder_zero)
                                        * pole_pairs;
                                if torque >= 0_f32 {
                                    phase_orientation_raw += PHASE_SHIFT_HALF_PI;
                                } else {
                                    phase_orientation_raw -= PHASE_SHIFT_HALF_PI;
                                    torque = -torque;
                                }
                                (Some(phase_orientation_raw.rem(16384)), torque)
                            }
                            _ => (None, 0.0),
                        };

                        match phase_orientation {
                            None => {
                                c1.disable();
                                c2.disable();
                                c3.disable();
                            }
                            Some(phase_orientation) => {
                                let max_duty = c1.get_max_duty();
                                let mut phase_orientation2 = phase_orientation + 16384 / 3;
                                let mut phase_orientation3 = phase_orientation + 16384 * 2 / 3;
                                if phase_orientation2 > 16383 {
                                    phase_orientation2 -= 16384;
                                }
                                if phase_orientation3 > 16383 {
                                    phase_orientation3 -= 16384;
                                }
                                let pwm1 = phase_pwm(phase_orientation, max_duty, torque);
                                let pwm2 = phase_pwm(phase_orientation2, max_duty, torque);
                                let pwm3 = phase_pwm(phase_orientation3, max_duty, torque);

                                c1.set_duty(pwm1);
                                c2.set_duty(pwm2);
                                c3.set_duty(pwm3);
                                c1.enable();
                                c2.enable();
                                c3.enable();
                            }
                        }
                    }
                    _ => {
                        c1.disable();
                        c2.disable();
                        c3.disable();
                    }
                },
                None => {
                    c1.disable();
                    c2.disable();
                    c3.disable();
                }
            }
        }

        if let Ok(frame) = can.receive0(&mut can_rx_buffer) {
            let frame = frame.unwrap();
            match frame.id {
                Standard(id) if id.as_raw() == 0 => {
                    GeneralCommandFrame::api_decode(&can_rx_buffer)
                        .iter()
                        .for_each(|gcf| match gcf {
                            GeneralCommandFrame::RequestChipId1 => {
                                let data1 = GeneralResponseFrame::ChipID1 { chip_id1 };
                                can.can_transmit(true, slave_address, &data1);
                            }
                            GeneralCommandFrame::RequestChipId2 { chip_id1: id } => {
                                if id == &chip_id1 {
                                    let data2 = GeneralResponseFrame::ChipID2 { chip_id2 };
                                    can.can_transmit(true, slave_address, &data2);
                                }
                            }
                            GeneralCommandFrame::ChipID1 { chip_id1: id } => {
                                if id == &chip_id1 {
                                    chip_id1_received = true;
                                    chip_id2_received = false;
                                } else {
                                    chip_id1_received = false;
                                    chip_id2_received = false;
                                }
                            }
                            GeneralCommandFrame::ChipID2 { chip_id2: id } => {
                                if chip_id1_received && (id == &chip_id2) {
                                    chip_id2_received = true;
                                } else {
                                    chip_id1_received = false;
                                    chip_id2_received = false;
                                }
                            }
                            GeneralCommandFrame::SetChannel { master, slave } => {
                                if chip_id1_received && chip_id2_received {
                                    master_address = *master;
                                    slave_address = *slave;
                                }
                            }
                        });
                }
                Standard(id) if id.as_raw() == master_address => {
                    ServoCommandFrame::api_decode(&can_rx_buffer)
                        .iter()
                        .for_each(|scf| match scf {
                            ServoCommandFrame::Disable
                            | ServoCommandFrame::Brake
                            | ServoCommandFrame::BrushedHoldPosition { .. }
                            | ServoCommandFrame::BrushedForceDutyCycle { .. }
                            | ServoCommandFrame::BrushlessForcePosition { .. }
                            | ServoCommandFrame::BrushlessHoldPosition { .. } => {
                                current_command = scf.clone();
                            }
                            ServoCommandFrame::SetPositionPGain { p } => position_p_gain = *p,
                            ServoCommandFrame::SetPositionIGain {
                                i,
                                cycles_to_max_out,
                            } => {
                                position_i_gain = *i;
                                max_position_integral = 8192_i64 * (*cycles_to_max_out as i64);
                            }
                            ServoCommandFrame::SetVelocityPGain { p } => velocity_p_gain = *p,
                            ServoCommandFrame::SetVelocityIGain {
                                i,
                                cycles_to_max_out,
                            } => {
                                velocity_i_gain = *i;
                                velocity_integral_cycles_to_max_out = *cycles_to_max_out;
                                max_velocity_integral =
                                    max_velocity * velocity_integral_cycles_to_max_out as f32;
                            }
                            ServoCommandFrame::SetMaxVelocity { v } => {
                                max_velocity = v.clamp(0.0, f32::MAX);
                                max_velocity_integral =
                                    max_velocity * velocity_integral_cycles_to_max_out as f32;
                            }
                            ServoCommandFrame::SetDataRate { data_rate: dr } => {
                                data_rate = (*dr).clamp(0, 1000) as u32
                            }
                            ServoCommandFrame::SetGeneralConfig1 {
                                duty_cycle_limit: d,
                                reverse_motor: r,
                            } => {
                                duty_cycle_limit = d.clamp(0.0, 1.0);
                                reverse_motor = *r;
                            }
                            ServoCommandFrame::SetGeneralConfig2 {
                                velocity_iir_filter_gain: g,
                                sensor_sample_rate_khz: _, //TODO: adjust sample rate
                            } => {
                                velocity_iir_filter_gain = g.clamp(0.0, 1.0);
                                inverse_velocity_iir_filter_gain = 1.0 - velocity_iir_filter_gain;
                            }
                            ServoCommandFrame::SetBrushlessConfig {
                                zero_phase: z,
                                pole_pairs: p,
                            } => {
                                encoder_zero = *z as u32;
                                pole_pairs = *p as u32;
                            }
                            ServoCommandFrame::SetPositionLowPassConfig { gain } => {
                                position_low_pass_gain = gain.clamp(0.0, 1.0);
                            }
                        });
                }
                _ => {}
            }
        }
    }
}
