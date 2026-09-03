#![no_std]

use bincode::error::{DecodeError as BincodeDecodeError, EncodeError as BincodeEncodeError};
use bincode::{config, Decode, Encode};

pub const MAX_CAN_PAYLOAD_LEN: usize = 8;
const ENUM_PREFIX_LEN: usize = 3;
const ENCODE_BUFFER_LEN: usize = 64;

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
    // Keep new variants at the end so existing CAN discriminants stay stable.
    SetPositionDGain {
        d: f32,
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

#[derive(Debug)]
pub enum FrameEncodeError {
    Bincode(BincodeEncodeError),
    NonCompactDiscriminant,
    PayloadTooLong { length: usize },
}

impl From<BincodeEncodeError> for FrameEncodeError {
    fn from(error: BincodeEncodeError) -> Self {
        Self::Bincode(error)
    }
}

#[derive(Debug)]
pub enum FrameDecodeError {
    Bincode(BincodeDecodeError),
    PayloadTooLong { length: usize },
}

impl From<BincodeDecodeError> for FrameDecodeError {
    fn from(error: BincodeDecodeError) -> Self {
        Self::Bincode(error)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EncodedFrame {
    bytes: [u8; MAX_CAN_PAYLOAD_LEN],
    len: usize,
}

impl EncodedFrame {
    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

pub trait ApiEncodeDecode: Encode + Decode<()> {
    fn api_encode(&self) -> Result<EncodedFrame, FrameEncodeError> {
        let config = config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let mut encoded = [0_u8; ENCODE_BUFFER_LEN];
        let encoded_len = bincode::encode_into_slice(self, &mut encoded, config)?;

        if encoded_len < ENUM_PREFIX_LEN || encoded[..ENUM_PREFIX_LEN] != [0; ENUM_PREFIX_LEN] {
            return Err(FrameEncodeError::NonCompactDiscriminant);
        }

        let payload = &encoded[ENUM_PREFIX_LEN..encoded_len];
        if payload.len() > MAX_CAN_PAYLOAD_LEN {
            return Err(FrameEncodeError::PayloadTooLong {
                length: payload.len(),
            });
        }

        let mut bytes = [0_u8; MAX_CAN_PAYLOAD_LEN];
        bytes[..payload.len()].copy_from_slice(payload);
        Ok(EncodedFrame {
            bytes,
            len: payload.len(),
        })
    }

    fn api_decode(data: &[u8]) -> Result<Self, FrameDecodeError> {
        if data.len() > MAX_CAN_PAYLOAD_LEN {
            return Err(FrameDecodeError::PayloadTooLong { length: data.len() });
        }

        let config = config::standard()
            .with_big_endian()
            .with_fixed_int_encoding();
        let mut encoded = [0_u8; MAX_CAN_PAYLOAD_LEN + ENUM_PREFIX_LEN];
        encoded[ENUM_PREFIX_LEN..ENUM_PREFIX_LEN + data.len()].copy_from_slice(data);

        bincode::decode_from_slice(&encoded[..ENUM_PREFIX_LEN + data.len()], config)
            .map(|decoded| decoded.0)
            .map_err(FrameDecodeError::from)
    }
}

impl ApiEncodeDecode for GeneralCommandFrame {}
impl ApiEncodeDecode for GeneralResponseFrame {}
impl ApiEncodeDecode for ServoCommandFrame {}
impl ApiEncodeDecode for ServoResponseFrame {}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_round_trip<T>(value: T)
    where
        T: ApiEncodeDecode + core::fmt::Debug + PartialEq,
    {
        let encoded = value.api_encode().unwrap();
        assert!(encoded.len() <= MAX_CAN_PAYLOAD_LEN);
        assert_eq!(T::api_decode(encoded.as_slice()).unwrap(), value);
    }

    #[test]
    fn preserves_existing_discriminants_and_encoding() {
        assert_eq!(
            ServoCommandFrame::Disable.api_encode().unwrap().as_slice(),
            &[0]
        );
        assert_eq!(
            ServoCommandFrame::BrushedHoldPosition { position: 0x1234 }
                .api_encode()
                .unwrap()
                .as_slice(),
            &[2, 0x12, 0x34]
        );
        assert_eq!(
            ServoCommandFrame::SetPositionLowPassConfig { gain: 0.5 }
                .api_encode()
                .unwrap()
                .as_slice(),
            &[15, 0x3f, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            ServoCommandFrame::SetPositionDGain { d: 0.25 }
                .api_encode()
                .unwrap()
                .as_slice(),
            &[16, 0x3e, 0x80, 0x00, 0x00]
        );
    }

    #[test]
    fn round_trips_all_servo_commands() {
        assert_round_trip(ServoCommandFrame::Disable);
        assert_round_trip(ServoCommandFrame::Brake);
        assert_round_trip(ServoCommandFrame::BrushedHoldPosition { position: 1234 });
        assert_round_trip(ServoCommandFrame::BrushedForceDutyCycle { duty_cycle: -0.5 });
        assert_round_trip(ServoCommandFrame::BrushlessHoldPosition { position: 2345 });
        assert_round_trip(ServoCommandFrame::BrushlessForcePosition {
            position: 3456,
            power: 0.75,
        });
        assert_round_trip(ServoCommandFrame::SetDataRate { data_rate: 500 });
        assert_round_trip(ServoCommandFrame::SetGeneralConfig1 {
            reverse_motor: true,
            duty_cycle_limit: 0.8,
        });
        assert_round_trip(ServoCommandFrame::SetGeneralConfig2 {
            velocity_iir_filter_gain: 0.97,
            sensor_sample_rate_khz: 10,
        });
        assert_round_trip(ServoCommandFrame::SetBrushlessConfig {
            zero_phase: 1234,
            pole_pairs: 7,
        });
        assert_round_trip(ServoCommandFrame::SetPositionPGain { p: 0.02 });
        assert_round_trip(ServoCommandFrame::SetPositionIGain {
            i: 0.001,
            cycles_to_max_out: 100,
        });
        assert_round_trip(ServoCommandFrame::SetVelocityPGain { p: 0.03 });
        assert_round_trip(ServoCommandFrame::SetVelocityIGain {
            i: 0.002,
            cycles_to_max_out: 200,
        });
        assert_round_trip(ServoCommandFrame::SetMaxVelocity { v: 42.0 });
        assert_round_trip(ServoCommandFrame::SetPositionLowPassConfig { gain: 0.5 });
        assert_round_trip(ServoCommandFrame::SetPositionDGain { d: 0.25 });
    }

    #[test]
    fn round_trips_general_frames_and_telemetry() {
        assert_round_trip(GeneralCommandFrame::SetChannel {
            master: 0x123,
            slave: 0x456,
        });
        assert_round_trip(GeneralResponseFrame::ChipID1 {
            chip_id1: [1, 2, 3, 4, 5, 6],
        });
        assert_round_trip(ServoResponseFrame::State {
            sensor_detected: true,
            position: 1234,
            velocity: -123,
            current: 456,
        });
    }
}
