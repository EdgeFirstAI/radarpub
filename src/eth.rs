use ndarray::{Array4, ArrayView4, Axis};
use num::Complex;
use std::{cmp::min, fmt, num::Wrapping, vec};
use tracing::instrument;

/// Fixed size size of the SMS UDP packets.
pub const SMS_PACKET_SIZE: usize = 1458;

#[allow(unused)]
#[derive(Debug)]
pub enum SMSError {
    IoError(std::io::Error),
    StartPattern(u8),
    UnexpectedEndOfSlice(usize),
    InvalidHeaderLength(u8),
    InvalidPayloadLength(u16),
    InvalidPortId(u32),
    InvalidDebugFlags(u8),
    MessageCounterMissing,
    DebugHeaderMissing,
    PortHeaderMissing,
    CubeHeaderMissing,
    BinPropertiesMissing,
    MessageSequenceError,
    FrameCounterError,
    ShapeError(ndarray::ShapeError),
    MissingCubeData(usize, usize),
    DroppedMessages(u16),
}

impl std::error::Error for SMSError {}

impl From<std::io::Error> for SMSError {
    fn from(err: std::io::Error) -> SMSError {
        SMSError::IoError(err)
    }
}

impl From<ndarray::ShapeError> for SMSError {
    fn from(err: ndarray::ShapeError) -> SMSError {
        SMSError::ShapeError(err)
    }
}

impl fmt::Display for SMSError {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            SMSError::IoError(err) => write!(f, "io error: {}", err),
            SMSError::StartPattern(pattern) => {
                write!(f, "unexpected start pattern: 0x{:02X}", pattern)
            }
            SMSError::UnexpectedEndOfSlice(size) => {
                write!(f, "unexpected end of slice: {}", size)
            }
            SMSError::InvalidHeaderLength(len) => {
                write!(f, "invalid header length: {}", len)
            }
            SMSError::InvalidPayloadLength(len) => {
                write!(f, "invalid payload length: {}", len)
            }
            SMSError::InvalidPortId(id) => {
                write!(f, "invalid port id: {}", id)
            }
            SMSError::InvalidDebugFlags(flags) => {
                write!(f, "invalid debug flags: 0x{:02X}", flags)
            }
            SMSError::MessageCounterMissing => {
                write!(f, "message counter missing")
            }
            SMSError::DebugHeaderMissing => {
                write!(f, "debug header missing")
            }
            SMSError::PortHeaderMissing => {
                write!(f, "port header missing")
            }
            SMSError::CubeHeaderMissing => {
                write!(f, "cube header missing")
            }
            SMSError::BinPropertiesMissing => {
                write!(f, "bin properties missing")
            }
            SMSError::MessageSequenceError => {
                write!(f, "message sequence error")
            }
            SMSError::FrameCounterError => {
                write!(f, "frame counter error")
            }
            SMSError::ShapeError(err) => {
                write!(f, "shape error: {}", err)
            }
            SMSError::MissingCubeData(len, expect) => {
                write!(f, "missing cube data [{}/{}]", len, expect)
            }
            SMSError::DroppedMessages(dropped) => {
                write!(f, "dropped messages: {}", dropped)
            }
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TransportHeader {
    pub start_pattern: u8,
    pub protocol_version: u8,
    pub header_length: u8,
    pub payload_length: u16,
    pub application_protocol: u8,
    pub flags: u32,
    pub message_counter: Option<Wrapping<u16>>,
    pub client_id: Option<u32>,
    pub data_id: Option<u16>,
    pub segmentation: Option<u16>,
    pub crc: u16,
}

impl TransportHeader {
    /// Length of the crc field in bytes/octets.
    pub const CRC_LEN: usize = 2;
    /// Maximum length of an SMS transport header in bytes/octets.
    pub const MAX_LEN: usize = 22;
    /// Minimum length of an SMS transport header in bytes/octets.
    pub const MIN_LEN: usize = 12;
}

/// A slice containing an SMS transport header.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TransportHeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> TransportHeaderSlice<'a> {
    pub fn from_slice(slice: &'a [u8]) -> Result<TransportHeaderSlice<'a>, SMSError> {
        if slice.len() < TransportHeader::MIN_LEN {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        if slice[0] != 0x7E {
            return Err(SMSError::StartPattern(slice[0]));
        }

        // Confirm that the slice is large enough to hold the CRC
        // starting from the offset to account for optional fields.
        if Self::crc_offset(slice) + TransportHeader::CRC_LEN > slice.len() {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        // Confirm calculated header size matches the reported header size.
        if Self::crc_offset(slice) + TransportHeader::CRC_LEN != slice[2] as usize {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        // Confirm that the slice can hold the entire header and payload.
        if slice.len() < slice[2] as usize + u16::from_be_bytes([slice[3], slice[4]]) as usize {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        Ok(TransportHeaderSlice { slice })
    }

    pub fn to_header(&self) -> TransportHeader {
        let crc_offset = Self::crc_offset(self.slice);

        TransportHeader {
            start_pattern: self.slice[0],
            protocol_version: self.slice[1],
            header_length: self.slice[2],
            payload_length: u16::from_be_bytes([self.slice[3], self.slice[4]]),
            application_protocol: self.slice[5],
            flags: u32::from_be_bytes([self.slice[6], self.slice[7], self.slice[8], self.slice[9]]),
            message_counter: self.message_counter(),
            client_id: self.client_id(),
            data_id: self.data_id(),
            segmentation: self.segmentation(),
            crc: u16::from_be_bytes([self.slice[crc_offset], self.slice[crc_offset + 1]]),
        }
    }

    /// Returns the message_counter or None if not present.
    #[inline]
    pub fn message_counter(&self) -> Option<Wrapping<u16>> {
        if Self::message_counter_size(self.slice) > 0 {
            let offset = TransportHeader::MIN_LEN - TransportHeader::CRC_LEN;
            Some(Wrapping(u16::from_be_bytes([
                self.slice[offset],
                self.slice[offset + 1],
            ])))
        } else {
            None
        }
    }

    /// Returns the application protocol number.
    #[inline]
    pub fn application_protocol(&self) -> u8 {
        self.slice[5]
    }

    /// Returns the client_id or None if not present.
    #[inline]
    pub fn client_id(&self) -> Option<u32> {
        if Self::client_id_size(self.slice) > 0 {
            let offset = TransportHeader::MIN_LEN - TransportHeader::CRC_LEN
                + Self::message_counter_size(self.slice);
            Some(u32::from_be_bytes([
                self.slice[offset],
                self.slice[offset + 1],
                self.slice[offset + 2],
                self.slice[offset + 3],
            ]))
        } else {
            None
        }
    }

    /// Returns the data_id or None if not present.
    #[inline]
    pub fn data_id(&self) -> Option<u16> {
        if Self::data_id_size(self.slice) > 0 {
            let offset = TransportHeader::MIN_LEN - TransportHeader::CRC_LEN
                + Self::message_counter_size(self.slice)
                + Self::client_id_size(self.slice);
            Some(u16::from_be_bytes([
                self.slice[offset],
                self.slice[offset + 1],
            ]))
        } else {
            None
        }
    }

    /// Returns the segmentation or None if not present.
    #[inline]
    pub fn segmentation(&self) -> Option<u16> {
        if Self::segmentation_size(self.slice) > 0 {
            let offset = TransportHeader::MIN_LEN - TransportHeader::CRC_LEN
                + Self::message_counter_size(self.slice)
                + Self::client_id_size(self.slice)
                + Self::data_id_size(self.slice);
            Some(u16::from_be_bytes([
                self.slice[offset],
                self.slice[offset + 1],
            ]))
        } else {
            None
        }
    }

    /// Returns the size of the message_counter field in bytes.
    #[inline]
    fn message_counter_size(slice: &'a [u8]) -> usize {
        if slice[9] & 0x01 != 0 {
            2
        } else {
            0
        }
    }

    /// Returns the size of the client_id field in bytes.
    #[inline]
    fn client_id_size(slice: &'a [u8]) -> usize {
        if slice[9] & 0x08 != 0 {
            4
        } else {
            0
        }
    }

    /// Returns the size of the data_id field in bytes.
    #[inline]
    fn data_id_size(slice: &'a [u8]) -> usize {
        if slice[9] & 0x20 != 0 {
            2
        } else {
            0
        }
    }

    /// Returns the size of the segmentation field in bytes.
    #[inline]
    fn segmentation_size(slice: &'a [u8]) -> usize {
        if slice[9] & 0x40 != 0 {
            2
        } else {
            0
        }
    }

    /// Returns the crc offset in the header slice.
    #[inline]
    fn crc_offset(slice: &'a [u8]) -> usize {
        TransportHeader::MIN_LEN - TransportHeader::CRC_LEN
            + Self::message_counter_size(slice)
            + Self::client_id_size(slice)
            + Self::data_id_size(slice)
            + Self::segmentation_size(slice)
    }

    /// Returns the header length in bytes.
    #[inline]
    pub fn len(&self) -> usize {
        TransportHeader::MIN_LEN
            + Self::message_counter_size(self.slice)
            + Self::client_id_size(self.slice)
            + Self::data_id_size(self.slice)
            + Self::segmentation_size(self.slice)
    }

    /// Returns true if the underlyinc slice is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.slice.is_empty()
    }

    /// Returns the debug header slice or an error if not present.
    #[inline]
    pub fn debug_header(&self) -> Result<DebugHeaderSlice<'a>, SMSError> {
        if self.application_protocol() != 5 {
            return Err(SMSError::DebugHeaderMissing);
        }

        DebugHeaderSlice::from_slice(self.payload())
    }

    /// Returns the port header slice or an error if not present.
    #[inline]
    pub fn port_header(&self) -> Result<PortHeaderSlice<'a>, SMSError> {
        match self.application_protocol() {
            5 => match self.debug_header()?.flags() {
                // The port header is present when flags are 1 or 3.
                1 => Ok(PortHeaderSlice::from_slice(
                    &self.payload()[DebugHeader::LEN..],
                )?),
                3 => Ok(PortHeaderSlice::from_slice(
                    &self.payload()[DebugHeader::LEN..],
                )?),
                _ => Err(SMSError::PortHeaderMissing),
            },
            8 => Ok(PortHeaderSlice::from_slice(self.payload())?),
            _ => Err(SMSError::PortHeaderMissing),
        }
    }

    /// Returns the cube header slice or an error if not present.
    #[inline]
    pub fn cube_header(&self) -> Result<CubeHeaderSlice<'a>, SMSError> {
        self.port_header()?.cube_header()
    }

    /// Returns the bin properties slice or an error if not present.
    #[inline]
    pub fn bin_properties(&self) -> Result<BinPropertiesSlice<'a>, SMSError> {
        self.port_header()?.bin_properties()
    }

    /// Returns the frame counter or None if not present.
    #[inline]
    pub fn frame_counter(&self) -> Option<u32> {
        match self.debug_header() {
            Ok(header) => Some(header.frame_counter()),
            Err(_) => None,
        }
    }

    /// Returns the slice containing the payload.
    #[inline]
    pub fn payload(&self) -> &'a [u8] {
        unsafe {
            // SAFETY: Safe as the slice length was verified
            // to be at least UdpHeader::LEN by "from_slice".
            core::slice::from_raw_parts(
                self.slice.as_ptr().add(self.len()),
                self.slice.len() - self.len(),
            )
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DebugHeader {
    pub frame_counter: u32,
    pub flags: u8,
    pub frame_delay: u8,
}

impl DebugHeader {
    /// End of data flag, designates the end of the radar data cube.
    pub const END_OF_DATA: u8 = 2;
    /// Frame data flag.
    pub const FRAME_DATA: u8 = 0;
    /// Frame footer flag, designates the bin properties message.
    pub const FRAME_FOOTER: u8 = 3;
    /// Length of the debug header in bytes/octets.
    pub const LEN: usize = 8;
    /// Start of frame flag.
    pub const START_OF_FRAME: u8 = 1;
}

/// A slice containing an SMS debug port header.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct DebugHeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> DebugHeaderSlice<'a> {
    pub fn from_slice(slice: &'a [u8]) -> Result<DebugHeaderSlice<'a>, SMSError> {
        if slice.len() < DebugHeader::LEN {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        Ok(DebugHeaderSlice { slice })
    }

    pub fn to_header(&self) -> DebugHeader {
        DebugHeader {
            frame_counter: u32::from_be_bytes([
                self.slice[0],
                self.slice[1],
                self.slice[2],
                self.slice[3],
            ]),
            flags: self.slice[4],
            frame_delay: self.slice[5],
        }
    }

    /// Returns the frame counter.
    #[inline]
    pub fn frame_counter(&self) -> u32 {
        u32::from_be_bytes([self.slice[0], self.slice[1], self.slice[2], self.slice[3]])
    }

    /// Returns the flags.
    #[inline]
    pub fn flags(&self) -> u8 {
        self.slice[4]
    }

    /// Returns the frame delay.
    #[inline]
    pub fn frame_delay(&self) -> u8 {
        self.slice[5]
    }

    /// Returns the slice containing the payload.
    #[inline]
    pub fn payload(&self) -> &'a [u8] {
        unsafe {
            // SAFETY: Safe as the slice length was verified
            // to be at least UdpHeader::LEN by "from_slice".
            core::slice::from_raw_parts(
                self.slice.as_ptr().add(DebugHeader::LEN),
                self.slice.len() - DebugHeader::LEN,
            )
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PortHeader {
    pub id: u32,
    pub interface_version_major: i16,
    pub interface_version_minor: i16,
    pub timestamp: u64,
    pub size: u32,
    pub endianess: u8,
    pub index: u8,
    pub header_version_major: u8,
    pub header_version_minor: u8,
}

impl PortHeader {
    /// Length of the port header in bytes/octets.
    pub const LEN: usize = 24;
}

/// A slice containing an SMS generic port header.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PortHeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> PortHeaderSlice<'a> {
    pub fn from_slice(slice: &'a [u8]) -> Result<PortHeaderSlice<'a>, SMSError> {
        if slice.len() < PortHeader::LEN {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        Ok(PortHeaderSlice { slice })
    }

    pub fn to_header(&self) -> PortHeader {
        PortHeader {
            id: u32::from_be_bytes([self.slice[0], self.slice[1], self.slice[2], self.slice[3]]),
            interface_version_major: i16::from_be_bytes([self.slice[4], self.slice[5]]),
            interface_version_minor: i16::from_be_bytes([self.slice[6], self.slice[7]]),
            timestamp: u64::from_be_bytes([
                self.slice[8],
                self.slice[9],
                self.slice[10],
                self.slice[11],
                self.slice[12],
                self.slice[13],
                self.slice[14],
                self.slice[15],
            ]),
            size: u32::from_be_bytes([
                self.slice[16],
                self.slice[17],
                self.slice[18],
                self.slice[19],
            ]),
            endianess: self.slice[20],
            index: self.slice[21],
            header_version_major: self.slice[22],
            header_version_minor: self.slice[23],
        }
    }

    /// Returns the port id.
    #[inline]
    pub fn id(&self) -> u32 {
        u32::from_be_bytes([self.slice[0], self.slice[1], self.slice[2], self.slice[3]])
    }

    /// Returns the timestamp.
    #[inline]
    pub fn timestamp(&self) -> u64 {
        u64::from_be_bytes([
            self.slice[8],
            self.slice[9],
            self.slice[10],
            self.slice[11],
            self.slice[12],
            self.slice[13],
            self.slice[14],
            self.slice[15],
        ])
    }

    /// Returns the radar cube header slice or an error if not present.
    #[inline]
    pub fn cube_header(&self) -> Result<CubeHeaderSlice<'a>, SMSError> {
        match self.id() {
            5 => CubeHeaderSlice::from_slize(self.payload()),
            _ => Err(SMSError::CubeHeaderMissing),
        }
    }

    /// Returns the bin properties slice or an error if not present.
    #[inline]
    pub fn bin_properties(&self) -> Result<BinPropertiesSlice<'a>, SMSError> {
        match self.id() {
            63 => BinPropertiesSlice::from_slize(self.payload()),
            _ => Err(SMSError::BinPropertiesMissing),
        }
    }

    /// Returns the slice containing the payload.
    #[inline]
    pub fn payload(&self) -> &'a [u8] {
        unsafe {
            // SAFETY: Safe as the slice length was verified
            // to be at least UdpHeader::LEN by "from_slice".
            core::slice::from_raw_parts(
                self.slice.as_ptr().add(PortHeader::LEN),
                self.slice.len() - PortHeader::LEN,
            )
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CubeHeader {
    /// Memory offset from one radar cube element to its imaginary part.
    pub imag_offset: i32,
    /// Memory offset from one radar cube element to its real part.
    pub real_offset: i32,
    /// Memory offset between two range gates (doppler bin, channel and chirp
    /// type remain constant)
    pub range_gate_offset: i32,
    /// Memory offset between two one doppler bins (range gate, channel and
    /// chirp type remain constant)
    pub doppler_bin_offset: i32,
    /// Memory offset between two channels (range gate, doppler bin and chirp
    /// type remain constant)
    pub rx_channel_offset: i32,
    /// Memory offset between two chirp types (range gate doppler bin and
    /// channel remain constant)
    pub chirp_type_offset: i32,
    /// Number of range gates of the range doppler matrix.
    pub range_gates: i16,
    /// The index of the first range gate that is stored in the range doppler
    /// matrix, counting starts from 0.
    pub first_range_gate: i16,
    /// Number of Doppler bins of the range doppler matrix.
    pub doppler_bins: i16,
    /// Number of channels (one range doppler matrix is stored for each RX
    /// channel)
    pub rx_channels: i8,
    /// Number of chirp types in the radar cube.
    pub chirp_types: i8,
    /// Size of one radar cube element in bytes.
    pub element_size: i8,
    /// Type of radar cube data in which allowed values listed in
    /// RC_ELEMENT_TYPES.
    pub element_type: i8,
    /// Number of padding bytes for radar cube data
    pub padding_bytes: i8,
}

impl CubeHeader {
    /// Length of the cube header in bytes/octets.
    pub const LEN: usize = 40;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct CubeHeaderSlice<'a> {
    slice: &'a [u8],
}

impl<'a> CubeHeaderSlice<'a> {
    pub fn from_slize(slice: &'a [u8]) -> Result<CubeHeaderSlice<'a>, SMSError> {
        if slice.len() < CubeHeader::LEN {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        Ok(CubeHeaderSlice { slice })
    }

    pub fn to_header(&self) -> CubeHeader {
        CubeHeader {
            imag_offset: i32::from_be_bytes([
                self.slice[0],
                self.slice[1],
                self.slice[2],
                self.slice[3],
            ]),
            real_offset: i32::from_be_bytes([
                self.slice[4],
                self.slice[5],
                self.slice[6],
                self.slice[7],
            ]),
            range_gate_offset: i32::from_be_bytes([
                self.slice[8],
                self.slice[9],
                self.slice[10],
                self.slice[11],
            ]),
            doppler_bin_offset: i32::from_be_bytes([
                self.slice[12],
                self.slice[13],
                self.slice[14],
                self.slice[15],
            ]),
            rx_channel_offset: i32::from_be_bytes([
                self.slice[16],
                self.slice[17],
                self.slice[18],
                self.slice[19],
            ]),
            chirp_type_offset: i32::from_be_bytes([
                self.slice[20],
                self.slice[21],
                self.slice[22],
                self.slice[23],
            ]),
            range_gates: i16::from_be_bytes([self.slice[24], self.slice[25]]),
            first_range_gate: i16::from_be_bytes([self.slice[26], self.slice[27]]),
            doppler_bins: i16::from_be_bytes([self.slice[28], self.slice[29]]),
            rx_channels: i8::from_be_bytes([self.slice[30]]),
            chirp_types: i8::from_be_bytes([self.slice[31]]),
            element_size: i8::from_be_bytes([self.slice[32]]),
            element_type: i8::from_be_bytes([self.slice[33]]),
            // 5 reserved bytes before padding_bytes
            padding_bytes: i8::from_be_bytes([self.slice[39]]),
        }
    }

    /// Returns the number of range gates of the range doppler matrix.
    #[inline]
    pub fn range_gates(&self) -> i16 {
        i16::from_be_bytes([self.slice[24], self.slice[25]])
    }

    /// Returns the number of doppler bins of the range doppler matrix.
    #[inline]
    pub fn doppler_bins(&self) -> i16 {
        i16::from_be_bytes([self.slice[28], self.slice[29]])
    }

    /// Returns the number of channels (one range doppler matrix is stored for
    /// each RX channel).
    #[inline]
    pub fn rx_channels(&self) -> i8 {
        self.slice[30] as i8
    }

    /// Returns the number of chirp types in the radar cube.
    #[inline]
    pub fn chirp_types(&self) -> i8 {
        self.slice[31] as i8
    }

    /// Returns the number of padding bytes before the radar cube data.
    #[inline]
    pub fn padding_bytes(&self) -> usize {
        self.slice[39] as usize
    }

    /// Returns the slice containing the payload.
    #[inline]
    pub fn payload(&self) -> &'a [u8] {
        unsafe {
            // SAFETY: Safe as the slice length was verified
            // to be at least UdpHeader::LEN by "from_slice".
            core::slice::from_raw_parts(
                self.slice
                    .as_ptr()
                    .add(CubeHeader::LEN + self.padding_bytes()),
                self.slice.len() - (CubeHeader::LEN + self.padding_bytes()),
            )
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BinProperties {
    pub speed_per_bin: f32,
    pub range_per_bin: f32,
    pub bin_per_speed: f32,
}

impl BinProperties {
    /// Length of the bin properties in bytes/octets.
    pub const LEN: usize = 12;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BinPropertiesSlice<'a> {
    slice: &'a [u8],
}

impl<'a> BinPropertiesSlice<'a> {
    pub fn from_slize(slice: &'a [u8]) -> Result<BinPropertiesSlice<'a>, SMSError> {
        if slice.len() < BinProperties::LEN {
            return Err(SMSError::UnexpectedEndOfSlice(slice.len()));
        }

        Ok(BinPropertiesSlice { slice })
    }

    pub fn to_header(&self) -> BinProperties {
        BinProperties {
            speed_per_bin: f32::from_be_bytes([
                self.slice[0],
                self.slice[1],
                self.slice[2],
                self.slice[3],
            ]),
            range_per_bin: f32::from_be_bytes([
                self.slice[4],
                self.slice[5],
                self.slice[6],
                self.slice[7],
            ]),
            bin_per_speed: f32::from_be_bytes([
                self.slice[8],
                self.slice[9],
                self.slice[10],
                self.slice[11],
            ]),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RadarCube {
    pub timestamp: u64,
    pub frame_counter: u32,
    pub packets_captured: u16,
    pub packets_skipped: u16,
    pub missing_data: usize,
    pub bin_properties: BinProperties,
    pub data: ndarray::Array4<Complex<i16>>,
}

impl fmt::Display for RadarCube {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "RadarCube {{ frame_counter: {}, shape: {:?}, bin_properties: {:?} }}",
            self.frame_counter,
            self.data.shape(),
            self.bin_properties
        )
    }
}

pub struct RadarCubeReader {
    timestamp: u64,
    frame_counter: u32,
    first_message: Wrapping<u16>,
    message_counter: Wrapping<u16>,
    received_messages: Wrapping<u16>,
    packets_captured: Wrapping<u16>,
    packets_skipped: Wrapping<u16>,
    error: Option<SMSError>,
    cube_header: Option<CubeHeader>,
    cube_index: usize,
    cube_captured: usize,
    cube: Vec<Complex<i16>>,
}

impl Default for RadarCubeReader {
    fn default() -> Self {
        Self::new()
    }
}

impl RadarCubeReader {
    pub fn new() -> RadarCubeReader {
        RadarCubeReader {
            timestamp: 0,
            frame_counter: 0,
            first_message: Wrapping(0),
            message_counter: Wrapping(0),
            received_messages: Wrapping(0),
            packets_captured: Wrapping(0),
            packets_skipped: Wrapping(0),
            error: None,
            cube_header: None,
            cube_index: 0,
            cube_captured: 0,
            cube: vec![],
        }
    }

    #[instrument(skip_all)]
    fn start_of_frame(
        &mut self,
        transport: &TransportHeaderSlice,
        debug_header: &DebugHeaderSlice,
    ) -> Result<Option<RadarCube>, SMSError> {
        *self = Self::default();
        self.timestamp = transport.port_header()?.timestamp();
        self.frame_counter = debug_header.frame_counter();
        self.first_message = transport.message_counter().unwrap();
        self.message_counter = self.first_message;
        self.received_messages = Wrapping(1);
        self.cube_header = Some(transport.cube_header()?.to_header());
        self.cube = vec![Complex::<i16>::new(32767, 32767); self.volume()?];
        // .resize(self.volume()?, Complex::<i16>::new(32767, 32767));
        let cube: Vec<u32> = transport
            .cube_header()?
            .payload()
            .chunks_exact(4)
            .map(|chunk| u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect();
        let cube =
            unsafe { std::slice::from_raw_parts(cube.as_ptr() as *const Complex<i16>, cube.len()) };
        self.cube[..cube.len()].copy_from_slice(cube);
        self.cube_index = cube.len();
        self.cube_captured = cube.len();
        self.packets_captured = Wrapping(1);

        Ok(None)
    }

    #[instrument(skip_all)]
    fn frame_footer(
        &mut self,
        transport: &TransportHeaderSlice,
        debug_header: &DebugHeaderSlice,
    ) -> Result<Option<RadarCube>, SMSError> {
        if self.cube_header.is_none() {
            *self = Self::default();
            return Err(SMSError::CubeHeaderMissing);
        }

        if self.frame_counter != debug_header.frame_counter() {
            *self = Self::default();
            return Err(SMSError::FrameCounterError);
        }

        if self.error.is_some() {
            let mut error = None;
            std::mem::swap(&mut self.error, &mut error);
            *self = Self::default();
            return Err(error.take().unwrap());
        }

        if self.cube_index < self.cube.len() {
            return Err(SMSError::MissingCubeData(self.cube_index, self.cube.len()));
        }

        let src = ArrayView4::from_shape(self.shape().unwrap(), &self.cube[..]).unwrap();
        let mut dst = Array4::<Complex<i16>>::zeros(self.shape().unwrap());
        let middle = src.shape()[3] / 2;
        let (src_right, src_left) = src.view().split_at(Axis(3), middle);
        let (mut dst_right, mut dst_left) = dst.view_mut().split_at(Axis(3), middle);
        dst_left.assign(&src_right);
        dst_right.assign(&src_left);
        dst.invert_axis(ndarray::Axis(1));

        let cube = RadarCube {
            timestamp: self.timestamp,
            packets_captured: self.packets_captured.0,
            packets_skipped: self.packets_skipped.0,
            frame_counter: self.frame_counter,
            bin_properties: transport.bin_properties().unwrap().to_header(),
            missing_data: self.volume()? - self.cube_captured,
            data: dst,
        };

        *self = Self::default();

        Ok(Some(cube))
    }

    /// This function fires on each UDP packet we receive so we only instrument
    /// at the trace level to avoid too much noise.  The critical portions for
    /// the radar data cube are the start_of_frame and frame_footer functions
    /// which are instrumented at the info level.
    #[instrument(skip_all, level = "trace")]
    fn frame_data(
        &mut self,
        transport: &TransportHeaderSlice,
        debug_header: &DebugHeaderSlice,
    ) -> Result<Option<RadarCube>, SMSError> {
        // Ignore data messages if the cube header is not present.  An
        // error will be returned when the frame footer is encountered.
        if self.cube_header.is_none() {
            return Ok(None);
        }

        // Ignore data messages if the frame counter does not match the
        // current frame counter.  We also move the index to the end of
        // the buffer to signal that we no longer want to read into the
        // now corrupt cube.  An error will be returned once we reach
        // the frame footer.
        if self.frame_counter != debug_header.frame_counter() {
            self.error = Some(SMSError::FrameCounterError);
            self.cube_index = self.cube.len();

            return Ok(None);
        }

        let message_counter = match transport.message_counter() {
            Some(message_counter) => message_counter,
            None => return Err(SMSError::MessageCounterMissing),
        };

        let expected_counter = self.message_counter + Wrapping(1);
        self.message_counter = message_counter;
        self.received_messages += Wrapping(1);

        // Identify missing messages and adjust the cube index
        // accordingly.  These messages should generally be
        // dropped by the client as they contain corrupt cubes.
        // The client is free to decide how to handle these by
        // counting the number of missing elements, those with
        // a value of 32767 (for both real and imaginary).
        if expected_counter != message_counter {
            // Calculate offset from the missing messages.
            // This code assumes that all the payloads are of
            // equal size when calculating the offset.
            let offset = (message_counter - expected_counter).0 as usize;
            let offset = offset * transport.debug_header()?.payload().len() / 4;
            self.cube_index += offset;

            // Avoid logging dropped messages once the cube has
            // been filled.  We don't care about dropped packets
            // in the dropped half of the radar cube frame.
            if self.cube_index < self.cube.len() {
                self.packets_skipped += message_counter - expected_counter;
            }
        }

        // This is a quick check to see if the cube is full. As
        // the DRVEGRD protocol will always transmit the maximum
        // possible cube size we want to ignore the random data
        // transmitted after the cube.
        if self.cube_index < self.cube.len() {
            self.packets_captured += 1;
            let cube: Vec<u32> = transport
                .debug_header()?
                .payload()
                .chunks_exact(4)
                .map(|chunk| u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
                .collect();
            let cube = unsafe {
                std::slice::from_raw_parts(cube.as_ptr() as *const Complex<i16>, cube.len())
            };
            let len = min(cube.len(), self.cube.len() - self.cube_index);
            self.cube[self.cube_index..(self.cube_index + len)].copy_from_slice(&cube[..len]);
            self.cube_index += cube.len();
            self.cube_captured += len as usize;
        }

        Ok(None)
    }

    pub fn read(&mut self, slice: &[u8]) -> Result<Option<RadarCube>, SMSError> {
        let transport = TransportHeaderSlice::from_slice(slice)?;
        let debug_header = transport.debug_header()?;

        match debug_header.flags() {
            DebugHeader::START_OF_FRAME => self.start_of_frame(&transport, &debug_header),
            DebugHeader::FRAME_FOOTER => self.frame_footer(&transport, &debug_header),
            DebugHeader::FRAME_DATA | DebugHeader::END_OF_DATA => {
                self.frame_data(&transport, &debug_header)
            }
            flags => Err(SMSError::InvalidDebugFlags(flags)),
        }
    }

    /// Returns the shape of the radar cube or the error CubeHeaderMissing if
    /// the cube header is not present.  The shape is represented as
    /// [chirp_types, rx_channels, range_gates, doppler_bins] with each value
    /// being a complex 16-bit integer.
    pub fn shape(&self) -> Result<[usize; 4], SMSError> {
        match &self.cube_header {
            Some(header) => Ok([
                header.chirp_types as usize,
                header.range_gates as usize,
                header.rx_channels as usize,
                header.doppler_bins as usize,
            ]),
            None => Err(SMSError::CubeHeaderMissing),
        }
    }

    /// Returns the radar cube volume or the error CubeHeaderMissing if the cube
    /// header is not present.  The volume is in the form of elements, each of
    /// which is the complex power of the radar signal as a Complex<i16>.
    pub fn volume(&self) -> Result<usize, SMSError> {
        self.shape().map(|shape| shape.iter().product())
    }
}

#[cfg(test)]
mod tests {
    use etherparse::{SlicedPacket, TransportSlice};
    use log::error;
    use pcarp::Capture;
    use std::fs::File;

    use super::*;

    #[test]
    fn test_pcap() -> Result<(), SMSError> {
        let mut first_frame = None;
        let mut last_frame = None;

        let office_3_first_frame = 27;
        let office_3_last_frame = 71;
        let path = "testdata/office_3.pcapng";
        let file = File::open(path).unwrap();

        let mut reader = RadarCubeReader::default();

        for cap in Capture::new(file) {
            match SlicedPacket::from_ethernet(&cap.unwrap().data) {
                Err(err) => error!("Err {:?}", err),
                Ok(pkt) => {
                    if let Some(TransportSlice::Udp(udp)) = pkt.transport {
                        if let Ok(sms) = TransportHeaderSlice::from_slice(udp.payload()) {
                            if first_frame.is_none() {
                                first_frame = sms.frame_counter();
                            }

                            last_frame = sms.frame_counter();

                            // println!("{:?}", sms.to_header());

                            // if let Ok(debug) = sms.debug_header() {
                            //     println!("{:?}", debug.to_header());
                            // }

                            // if let Ok(port) = sms.port_header() {
                            //     println!("{:?}", port.to_header());
                            // }

                            // if let Ok(cube) = sms.cube_header() {
                            //     println!("{:?}", cube.to_header());
                            // }

                            // if let Ok(bin) = sms.bin_properties() {
                            //     println!("{:?}", bin.to_header());
                            // }
                        }

                        match reader.read(udp.payload()) {
                            Ok(Some(cube)) => {
                                println!(
                                    "cube shape: {:?} size: {}",
                                    cube.data.shape(),
                                    cube.data.len()
                                )
                            }
                            Ok(None) => (),
                            // Ignore StartPattern errors when reading from pcap which includes
                            // non-SMS data.
                            Err(SMSError::StartPattern(_)) => (),
                            Err(err) => println!("Cube Error: {:?}", err),
                        }
                    }
                }
            }
        }

        assert_eq!(first_frame, Some(office_3_first_frame));
        assert_eq!(last_frame, Some(office_3_last_frame));

        Ok(())
    }
}
