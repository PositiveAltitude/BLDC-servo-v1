use alloc::vec::Vec;
use bincode::{config, Encode, Decode};
use bincode::error::{DecodeError, EncodeError};

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum GeneralResponseFrame {
    ChipID1 { chip_id1: [u8; 6] },
    ChipID2 { chip_id2: [u8; 6] },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum GeneralCommandFrame {
    RequestInitialization,
    RequestChipId1,
    RequestChipId2 { chip_id1: [u8; 6] },
    ChipID1 { chip_id1: [u8; 6] },
    ChipID2 { chip_id2: [u8; 6] },
    SetChannel { master: u16, slave: u16 },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum AqueductCommandFrame {
    RequestChipId,
    SetUpdateRate { update_period: u16 },
    SetLedConstant { r: u8, g: u8, b: u8 },
    SetLedBlink { r: u8, g: u8, b: u8, duration_on_50ms: u8, duration_off_50ms: u8 },
    SetLedPulse { r1: u8, g1: u8, b1: u8, r2: u8, g2: u8, b2: u8, duration_50ms: u8 },
    ActivatePump { duration_50ms: u8 },
}

#[derive(Encode, Decode, PartialEq, Debug, Clone)]
pub enum AqueductResponseFrame {
    ChipID1 { chip_id1: [u8; 6] },
    ChipID2 { chip_id2: [u8; 6] },
    ButtonDown,
    ButtonUp,
    State { button: bool, pump: bool, weight: i32 },
    ServoState{ vcc: u16, current1: i32}
}

pub trait ApiEncodeDecode: Encode + Decode {
    fn api_encode(&self) -> Result<Vec<u8>, EncodeError> {
        let config = config::standard().with_big_endian().with_fixed_int_encoding();
        let vec = bincode::encode_to_vec(&self, config);

        match vec {
            Ok(vec) => {
                let mut v = vec;
                v.remove(0);
                v.remove(0);
                v.remove(0);
                Ok(v)
            }
            Err(a) => {
                Err(a)
            }
        }
    }
    fn api_decode(data: &[u8]) -> Result<Self, DecodeError> {
        let config = config::standard().with_big_endian().with_fixed_int_encoding();

        let mut vec = Vec::from(data);
        vec.insert(0, 0);
        vec.insert(0, 0);
        vec.insert(0, 0);

        bincode::decode_from_slice(vec.as_slice(), config).map(|d| d.0)
    }
}

impl ApiEncodeDecode for GeneralCommandFrame {}
impl ApiEncodeDecode for GeneralResponseFrame {}
impl ApiEncodeDecode for AqueductCommandFrame {}
impl ApiEncodeDecode for AqueductResponseFrame {}
