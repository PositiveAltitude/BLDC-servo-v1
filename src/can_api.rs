use alloc::vec::Vec;
use bincode::error::{DecodeError, EncodeError};
use bincode::{config, Decode, Encode};

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum GeneralResponseFrame {
    ChipID1 { chip_id1: [u8; 6] },
    ChipID2 { chip_id2: [u8; 6] },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum GeneralCommandFrame {
    RequestChipId1,
    RequestChipId2 { chip_id1: [u8; 6] },
    ChipID1 { chip_id1: [u8; 6] },
    ChipID2 { chip_id2: [u8; 6] },
    SetChannel { master: u16, slave: u16 },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum ServoCommandFrame {
    Disable,
    Brake,
    BrushedHoldPosition {
        position: u16,
    },
    BrushedForceDutyCycle {
        duty_cycle: f32,
    },
    BrushlessHoldPosition {
        position: u16,
    },
    BrushlessForcePosition {
        position: u16,
        power: f32,
    },

    //Settings
    SetDataRate {
        data_rate: u16,
    },
    SetGeneralConfig1 {
        reverse_motor: bool,
        duty_cycle_limit: f32,
    },
    SetGeneralConfig2 {
        velocity_iir_filter_gain: f32,
        sensor_sample_rate_khz: u8,
        // main_loop_frequency_khz: u8,
    },
    SetBrushlessConfig {
        zero_phase: u16,
        pole_pairs: u8,
    },
    SetPositionPGain {
        p: f32,
    },
    SetPositionIGain {
        i: f32,
        cycles_to_max_out: u16,
    },
    SetVelocityPGain {
        p: f32,
    },
    SetVelocityIGain {
        i: f32,
        cycles_to_max_out: u16,
    },
    SetMaxVelocity {
        v: f32,
    },
    SetPositionLowPassConfig {
        gain: f32,
    },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum ServoResponseFrame {
    State {
        sensor_detected: bool,
        position: u16,
        velocity: i16,
        current: i16,
    },
}

pub trait ApiEncodeDecode: Encode + Decode<()> {
    fn api_encode(&self) -> Result<Vec<u8>, EncodeError> {
        let config = config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let vec = bincode::encode_to_vec(self, config);

        match vec {
            Ok(vec) => {
                let mut v = vec;
                v.remove(0);
                v.remove(0);
                v.remove(0);
                Ok(v)
            }
            Err(a) => Err(a),
        }
    }
    fn api_decode(data: &[u8]) -> Result<Self, DecodeError> {
        let config = config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();

        let mut vec = Vec::from(data);
        vec.insert(0, 0);
        vec.insert(0, 0);
        vec.insert(0, 0);

        bincode::decode_from_slice(vec.as_slice(), config).map(|d| d.0)
    }
}

impl ApiEncodeDecode for GeneralCommandFrame {}
impl ApiEncodeDecode for GeneralResponseFrame {}
impl ApiEncodeDecode for ServoCommandFrame {}
impl ApiEncodeDecode for ServoResponseFrame {}
