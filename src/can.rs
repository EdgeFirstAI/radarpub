// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.

use crc16::{State, CCITT_FALSE};
use log::{debug, trace};
use socketcan::{tokio::CanSocket, CanFrame, EmbeddedFrame, Id as CanId, StandardId};
use std::{fmt, io};

#[allow(unused)]
/// DRVEGRD protocol error types.
///
/// Follows UATv4 protocol specification naming conventions.
#[allow(clippy::enum_variant_names)]
#[derive(Debug)]
pub enum Error {
    /// I/O error from underlying socket operations
    Io(io::Error),
    /// Invalid header format or content
    InvalidHeader(String),
    /// Message sequence number mismatch
    OutOfSequence(String),
    /// No CAN socket available
    NoSocket,
    /// Response ID does not match request
    InvalidResponseId(u16),
    /// Unsupported UAT protocol version
    UATProtocolUnsupported(u16),
    /// CRC check failed
    UATCRCError,
    /// UAT protocol error code
    UATError(u16),
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        match self {
            Error::Io(err) => write!(f, "io error: {}", err),
            Error::InvalidHeader(err) => write!(f, "invalid header: {}", err),
            Error::OutOfSequence(err) => write!(f, "out of sequence: {}", err),
            Error::NoSocket => write!(f, "no socket"),
            Error::InvalidResponseId(id) => write!(f, "invalid response id: {}", id),
            Error::UATProtocolUnsupported(ver) => {
                write!(f, "UAT protocol version {} unsupported", ver)
            }
            Error::UATCRCError => write!(f, "UAT CRC error"),
            Error::UATError(err) => write!(f, "UAT error: {}", err),
        }
    }
}

/// Raw CAN message packet from DRVEGRD radar.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Packet {
    /// CAN message ID (extended 29-bit)
    pub id: u32,
    /// 8-byte data payload as u64
    pub data: u64,
}

/// Complete radar frame containing header and target list.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// Frame header with timing and configuration
    pub header: Header,
    /// Array of detected targets (up to 256)
    pub targets: [Target; 256],
}

impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Frame => {:?} {:?}",
            self.header,
            self.targets.to_vec().split_at(self.header.n_targets).0
        )
    }
}

/// Radar frame header with timing and configuration data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Header {
    /// Timestamp seconds (UNIX epoch)
    pub seconds: u32,
    /// Timestamp nanoseconds
    pub nanoseconds: u32,
    /// Frame cycle duration in seconds
    pub cycle_duration: f64,
    /// Sequential frame counter
    pub cycle_counter: u32,
    /// Number of valid targets in frame
    pub n_targets: usize,
    /// Active transmit antenna ID
    pub tx_antenna: u8,
    /// Frequency sweep configuration
    pub frequency_sweep: u8,
    /// Center frequency setting (76-77 GHz)
    pub center_frequency: u8,
}

/// Detected radar target with position and characteristics.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Target {
    /// Range distance in meters
    pub range: f64,
    /// Azimuth angle in radians
    pub azimuth: f64,
    /// Elevation angle in radians
    pub elevation: f64,
    /// Radial velocity in m/s
    pub speed: f64,
    /// Radar cross-section in dBsm
    pub rcs: f64,
    /// Received power in dBm
    pub power: f64,
    /// Noise level in dBm
    pub noise: f64,
}

#[allow(unused)]
#[derive(Copy, Clone)]
enum MessageType {
    Command = 0,
    StatusRequest = 1,
    ParameterWrite = 2,
    ParameterRead = 3,
    ParameterWriteRead = 4,
}

/// Configurable radar parameters.
///
/// These parameters can be read and written via CAN to configure
/// the radar sensor operation.
#[allow(unused)]
#[derive(Copy, Clone, Debug)]
pub enum Parameter {
    /// Transmit antenna selection (0-3)
    TxAntenna = 0,
    /// Center frequency setting (76-77 GHz)
    CenterFrequency = 1,
    /// Frequency sweep bandwidth
    FrequencySweep = 2,
    /// Range mode toggle
    RangeToggle = 5,
    /// Detection sensitivity threshold
    DetectionSensitivity = 13,
    /// Enable/disable target list output
    EnableTargetList = 200,
}

impl clap::ValueEnum for Parameter {
    fn value_variants<'a>() -> &'a [Self] {
        &[
            Parameter::CenterFrequency,
            Parameter::FrequencySweep,
            Parameter::RangeToggle,
            Parameter::DetectionSensitivity,
            Parameter::EnableTargetList,
        ]
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            Self::CenterFrequency => Some(clap::builder::PossibleValue::new("center_frequency")),
            Self::FrequencySweep => Some(clap::builder::PossibleValue::new("frequency_sweep")),
            Self::RangeToggle => Some(clap::builder::PossibleValue::new("range_toggle")),
            Self::DetectionSensitivity => {
                Some(clap::builder::PossibleValue::new("detection_sensitivity"))
            }
            Self::EnableTargetList => Some(clap::builder::PossibleValue::new("enable_target_list")),
            Self::TxAntenna => None,
        }
    }
}

// Smart Micro DRVEGRD Protocol: Status Query Values
/// Radar sensor status and version information fields.
///
/// Used by drvegrdctl for sensor diagnostics and version information.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.2
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum Status {
    /// Software generation number
    SoftwareGeneration = 2,
    /// Major version number
    MajorVersion = 3,
    /// Minor version number
    MinorVersion = 4,
    /// Patch version number
    PatchVersion = 5,
    /// Sensor serial number
    SerialNumber = 9,
}

impl clap::ValueEnum for Status {
    fn value_variants<'a>() -> &'a [Self] {
        &[
            Status::SoftwareGeneration,
            Status::MajorVersion,
            Status::MinorVersion,
            Status::PatchVersion,
            Status::SerialNumber,
        ]
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            Self::SoftwareGeneration => {
                Some(clap::builder::PossibleValue::new("software_generation"))
            }
            Self::MajorVersion => Some(clap::builder::PossibleValue::new("major_version")),
            Self::MinorVersion => Some(clap::builder::PossibleValue::new("minor_version")),
            Self::PatchVersion => Some(clap::builder::PossibleValue::new("patch_version")),
            Self::SerialNumber => Some(clap::builder::PossibleValue::new("serial_number")),
        }
    }
}

/// Sensor control commands.
///
/// Smart Micro DRVEGRD Protocol: Sensor Control Commands
/// Used by drvegrdctl for sensor configuration and management.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.1
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum Command {
    /// Reset sensor to factory defaults
    FactoryReset = 340,
    /// Soft reset of sensor
    SensorReset = 342,
    /// Save current parameters to non-volatile memory
    SaveParameters = 344,
    /// Reset parameters to last saved values
    ResetParameters = 345,
    /// Restore default parameter values
    DefaultParameters = 346,
    /// Set timestamp seconds
    SetSeconds = 350,
    /// Set timestamp fractional seconds
    SetFractionalSeconds = 351,
}

impl clap::ValueEnum for Command {
    fn value_variants<'a>() -> &'a [Self] {
        &[
            Command::FactoryReset,
            Command::SensorReset,
            Command::SaveParameters,
            Command::ResetParameters,
            Command::DefaultParameters,
            Command::SetSeconds,
            Command::SetFractionalSeconds,
        ]
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            Self::FactoryReset => Some(clap::builder::PossibleValue::new("factory_reset")),
            Self::SensorReset => Some(clap::builder::PossibleValue::new("sensor_reset")),
            Self::SaveParameters => Some(clap::builder::PossibleValue::new("save_parameters")),
            Self::ResetParameters => Some(clap::builder::PossibleValue::new("reset_parameters")),
            Self::DefaultParameters => {
                Some(clap::builder::PossibleValue::new("default_parameters"))
            }
            Self::SetSeconds => Some(clap::builder::PossibleValue::new("set_seconds")),
            Self::SetFractionalSeconds => {
                Some(clap::builder::PossibleValue::new("set_fractional_seconds"))
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct InstructionHeader {
    pub uat_id: u16,
    pub message_index: u8,
    pub protocol_version: u8,
    pub device_id: u8,
    pub instructions: u8,
    pub crc: u16,
}

impl From<&InstructionHeader> for [u8; 8] {
    fn from(header: &InstructionHeader) -> Self {
        let uat_id = header.uat_id.to_le_bytes();
        let crc = header.crc.to_le_bytes();

        let msg = [
            uat_id[0],
            uat_id[1],
            header.message_index,
            header.protocol_version,
            header.device_id,
            header.instructions,
            crc[0],
            crc[1],
        ];

        trace!("InstructionHeader:   {:02X?}", msg);
        msg
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct InstructionMessage1 {
    pub uat_id: u16,
    pub message_index: u8,
    pub message_type: u8,
    pub parnum: u16,
    pub dim0: u8,
    pub dim1: u8,
}

impl From<&InstructionMessage1> for [u8; 8] {
    fn from(message: &InstructionMessage1) -> Self {
        let uat_id = message.uat_id.to_le_bytes();
        let parnum = message.parnum.to_le_bytes();

        let msg = [
            uat_id[0],
            uat_id[1],
            message.message_index,
            message.message_type,
            parnum[0],
            parnum[1],
            message.dim0,
            message.dim1,
        ];

        trace!("InstructionMessage1: {:02X?}", msg);
        msg
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct InstructionMessage2 {
    pub uat_id: u16,
    pub message_index: u8,
    pub format: u8,
    pub value: u32,
}

impl From<&InstructionMessage2> for [u8; 8] {
    fn from(message: &InstructionMessage2) -> Self {
        let uat_id = message.uat_id.to_le_bytes();
        let value = message.value.to_le_bytes();

        let msg = [
            uat_id[0],
            uat_id[1],
            message.message_index,
            message.format,
            value[0],
            value[1],
            value[2],
            value[3],
        ];

        trace!("InstructionMessage2: {:02X?}", msg);
        msg
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
// Smart Micro DRVEGRD Protocol: Response Message Header
// First frame of multi-frame response from sensor.
// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.3
#[allow(dead_code)]
struct ResponseHeader {
    pub udt_index: u16,
    pub protocol_version: u16,
    pub device_id: u8,
    pub instructions: u8,
    pub crc: u16,
}

impl From<&[u8; 8]> for ResponseHeader {
    fn from(data: &[u8; 8]) -> Self {
        trace!("ResponseHeader:   {:02X?}", data);

        let udt_index = u16::from_le_bytes([data[0], data[1]]);
        let protocol_version = u16::from_le_bytes([data[2], data[3]]);
        let device_id = data[4];
        let instructions = data[5];
        let crc = u16::from_le_bytes([data[6], data[7]]);

        ResponseHeader {
            udt_index,
            protocol_version,
            device_id,
            instructions,
            crc,
        }
    }
}

impl From<u64> for ResponseHeader {
    fn from(data: u64) -> Self {
        let data = data.to_le_bytes();
        ResponseHeader::from(&data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
// Smart Micro DRVEGRD Protocol: Response Message Frame 1
// Second frame of multi-frame response (value bytes 0-7).
#[allow(dead_code)]
struct ResponseMessage1 {
    pub udt_index: u16,
    pub message_index: u8,
    pub message_type: u8,
    pub uat_id: u16,
    pub parnum: u16,
}

impl From<&[u8; 8]> for ResponseMessage1 {
    fn from(data: &[u8; 8]) -> Self {
        trace!("ResponseMessage1: {:02X?}", data);

        let udt_index = u16::from_le_bytes([data[0], data[1]]);
        let message_index = data[2];
        let message_type = data[3];
        let uat_id = u16::from_le_bytes([data[4], data[5]]);
        let parnum = u16::from_le_bytes([data[6], data[7]]);

        ResponseMessage1 {
            udt_index,
            message_index,
            message_type,
            uat_id,
            parnum,
        }
    }
}

impl From<u64> for ResponseMessage1 {
    fn from(data: u64) -> Self {
        let data = data.to_le_bytes();
        ResponseMessage1::from(&data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
// Smart Micro DRVEGRD Protocol: Response Message Frame 2
// Third frame of multi-frame response (value bytes 8-15).
#[allow(dead_code)]
struct ResponseMessage2 {
    pub udt_index: u16,
    pub message_index: u8,
    pub result: u8,
    pub value: u32,
}

impl From<&[u8; 8]> for ResponseMessage2 {
    fn from(data: &[u8; 8]) -> Self {
        trace!("ResponseMessage2: {:02X?}", data);

        let udt_index = u16::from_le_bytes([data[0], data[1]]);
        let message_index = data[2];
        let result = data[3];
        let value = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);

        ResponseMessage2 {
            udt_index,
            message_index,
            result,
            value,
        }
    }
}

impl From<u64> for ResponseMessage2 {
    fn from(data: u64) -> Self {
        let data = data.to_le_bytes();
        ResponseMessage2::from(&data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
// Smart Micro DRVEGRD Protocol: Response Message Frame 3
// Fourth frame of multi-frame response (value bytes 16-23).
#[allow(dead_code)]
struct ResponseMessage3 {
    pub udt_index: u16,
    pub message_index: u8,
    pub format: u8,
    pub dim0: u8,
    pub dim1: u8,
    _res: u16,
}

impl From<&[u8; 8]> for ResponseMessage3 {
    fn from(data: &[u8; 8]) -> Self {
        trace!("ResponseMessage3: {:02X?}", data);

        let udt_index = u16::from_le_bytes([data[0], data[1]]);
        let message_index = data[2];
        let format = data[3];
        let dim0 = data[4];
        let dim1 = data[5];

        ResponseMessage3 {
            udt_index,
            message_index,
            format,
            dim0,
            dim1,
            _res: 0,
        }
    }
}

impl From<u64> for ResponseMessage3 {
    fn from(data: u64) -> Self {
        let data = data.to_le_bytes();
        ResponseMessage3::from(&data)
    }
}

/// Calculates the CRC16-CCITT checksum for the given message.
// Calculate CRC-16 CCITT for command/response messages.
// Used by sensor control functions (drvegrdctl).
#[allow(dead_code)]
fn message_crc(
    header: &InstructionHeader,
    message1: &InstructionMessage1,
    message2: &InstructionMessage2,
) -> u16 {
    let mut crc = State::<CCITT_FALSE>::new();
    let header = <[u8; 8]>::from(header);
    let message1 = <[u8; 8]>::from(message1);
    let message2 = <[u8; 8]>::from(message2);

    crc.update(&header[..6]);
    crc.update(&message1);
    crc.update(&message2);

    crc.get()
}

/// Sends a prepared instruction request to the SmartMicro using the UATv4
/// protocol.
// Send instruction message to sensor (write command/parameter).
// Used by drvegrdctl for sensor configuration.
// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.1
#[allow(dead_code)]
async fn send_instruction(
    sock: &CanSocket,
    header: InstructionHeader,
    message1: InstructionMessage1,
    message2: InstructionMessage2,
) -> Result<(), Error> {
    let mut header = header; // mutable copy of the header for crc updates
    header.crc = message_crc(&header, &message1, &message2);

    let id = StandardId::new(0x3FB).unwrap();
    let header_frame = CanFrame::new(id, &<[u8; 8]>::from(&header)).unwrap();
    let message1_frame = CanFrame::new(id, &<[u8; 8]>::from(&message1)).unwrap();
    let message2_frame = CanFrame::new(id, &<[u8; 8]>::from(&message2)).unwrap();

    sock.write_frame(header_frame).await?;
    sock.write_frame(message1_frame).await?;
    sock.write_frame(message2_frame).await?;

    Ok(())
}

/// Receives an instruction response from the SmartMicro using the UATv4
/// protocol.
// Receive and parse response message from sensor.
// Used by drvegrdctl for reading sensor state and diagnostics.
#[allow(dead_code)]
async fn recv_response(sock: &CanSocket) -> Result<u32, Error> {
    let mut header = Packet { id: 0, data: 0 };

    // Retry loop in case we receive a buffered target frame before the response.
    for _ in 0..100 {
        header = read_frame(sock).await?;
        if header.id == 0x700 {
            break;
        }
    }

    if header.id != 0x700 {
        return Err(Error::InvalidResponseId(header.id as u16));
    }

    let header = ResponseHeader::from(header.data);
    trace!("{:?}", header);

    if header.protocol_version == 2 {
        return Err(Error::UATCRCError);
    } else if header.protocol_version != 5 {
        return Err(Error::UATProtocolUnsupported(header.protocol_version));
    }

    let message1 = read_frame(sock).await?;
    if message1.id != 0x700 {
        return Err(Error::InvalidResponseId(message1.id as u16));
    }
    let message1 = ResponseMessage1::from(message1.data);
    trace!("{:?}", message1);

    let message2 = read_frame(sock).await?;
    if message2.id != 0x700 {
        return Err(Error::InvalidResponseId(message2.id as u16));
    }
    let message2 = ResponseMessage2::from(message2.data);
    trace!("{:?}", message2);

    let message3 = read_frame(sock).await?;
    if message3.id != 0x700 {
        return Err(Error::InvalidResponseId(message3.id as u16));
    }
    let message3 = ResponseMessage3::from(message3.data);
    trace!("{:?}", message3);

    if message2.result != 0 {
        return Err(Error::UATError(message2.result as u16));
    }

    debug!("response 1: {:?} 2: {:?}", message1, message2);

    Ok(message2.value)
}

/// Send command to sensor and await response.
///
/// # Arguments
/// * `sock` - Active CAN socket connection
/// * `command` - Command to execute
/// * `value` - Command parameter value
///
/// # Returns
/// Response value from sensor
///
/// # Errors
/// Returns Error if CAN communication fails or sensor reports error
///
/// Public API for drvegrdctl binary.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.1
#[allow(dead_code)]
pub async fn send_command(sock: &CanSocket, command: Command, value: u32) -> Result<u32, Error> {
    debug!("send_command {:?} {}", command, value);

    let header = InstructionHeader {
        crc: 0,
        instructions: 1,
        device_id: 0,
        protocol_version: 4,
        message_index: 0,
        uat_id: 1000,
    };

    let message1 = InstructionMessage1 {
        dim0: 0,
        dim1: 0,
        parnum: command as u16,
        message_type: MessageType::Command as u8,
        message_index: 1,
        uat_id: 1000,
    };

    let message2 = InstructionMessage2 {
        value,
        format: 0,
        message_index: 2,
        uat_id: 1000,
    };

    send_instruction(sock, header, message1, message2).await?;
    recv_response(sock).await
}

/// Write parameter value to sensor.
///
/// # Arguments
/// * `sock` - Active CAN socket connection
/// * `param` - Parameter to write
/// * `value` - New parameter value
///
/// # Returns
/// Confirmation value from sensor
///
/// # Errors
/// Returns Error if CAN communication fails or sensor reports error
///
/// Public API for drvegrdctl binary.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 4.1
#[allow(dead_code)]
pub async fn write_parameter(sock: &CanSocket, param: Parameter, value: u32) -> Result<u32, Error> {
    debug!("write_parameter {:?} {}", param, value);

    let header = InstructionHeader {
        crc: 0,
        instructions: 1,
        device_id: 0,
        protocol_version: 4,
        message_index: 0,
        uat_id: 2010,
    };

    let message1 = InstructionMessage1 {
        dim0: 0,
        dim1: 0,
        parnum: param as u16,
        message_type: MessageType::ParameterWrite as u8,
        message_index: 1,
        uat_id: 2010,
    };

    let message2 = InstructionMessage2 {
        value,
        format: 0,
        message_index: 2,
        uat_id: 2010,
    };

    send_instruction(sock, header, message1, message2).await?;
    recv_response(sock).await
}

/// Read parameter value from sensor.
///
/// # Arguments
/// * `sock` - Active CAN socket connection
/// * `param` - Parameter to read
///
/// # Returns
/// Current parameter value
///
/// # Errors
/// Returns Error if CAN communication fails or sensor reports error
///
/// Public API for drvegrdctl binary.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 4.1
#[allow(dead_code)]
pub async fn read_parameter(sock: &CanSocket, param: Parameter) -> Result<u32, Error> {
    debug!("read_parameter {:?}", param);

    let header = InstructionHeader {
        crc: 0,
        instructions: 1,
        device_id: 0,
        protocol_version: 4,
        message_index: 0,
        uat_id: 2010,
    };

    let message1 = InstructionMessage1 {
        dim0: 0,
        dim1: 0,
        parnum: param as u16,
        message_type: MessageType::ParameterRead as u8,
        message_index: 1,
        uat_id: 2010,
    };

    let message2 = InstructionMessage2 {
        value: 0,
        format: 0,
        message_index: 2,
        uat_id: 2010,
    };

    send_instruction(sock, header, message1, message2).await?;
    recv_response(sock).await
}

/// Read status field from sensor.
///
/// # Arguments
/// * `sock` - Active CAN socket connection
/// * `status` - Status field to read
///
/// # Returns
/// Current status value
///
/// # Errors
/// Returns Error if CAN communication fails or sensor reports error
///
/// Public API for drvegrdctl binary.
/// See: DRVEGRD Communication Protocol Specification v4.2, Section 5.2
#[allow(dead_code)]
pub async fn read_status(sock: &CanSocket, status: Status) -> Result<u32, Error> {
    debug!("read_status");

    let header = InstructionHeader {
        crc: 0,
        instructions: 1,
        device_id: 0,
        protocol_version: 4,
        message_index: 0,
        uat_id: 2012,
    };

    let message1 = InstructionMessage1 {
        dim0: 0,
        dim1: 0,
        parnum: status as u16,
        message_type: MessageType::StatusRequest as u8,
        message_index: 1,
        uat_id: 2012,
    };

    let message2 = InstructionMessage2 {
        value: 0,
        format: 0,
        message_index: 2,
        uat_id: 2012,
    };

    send_instruction(sock, header, message1, message2).await?;
    recv_response(sock).await
}

/// The read_message function is a state machine that reads a frame from the
/// provided CAN reader. It returns a Frame struct when a complete frame has
/// been read.  The function will throw away any incomplete frames, returning
/// the first full frame it encounters.
///
/// The CAN reader is a function that returns a future that resolves to a Packet
/// struct which represents a CAN ID and data. This allows the function to be
/// used with a variety of CAN readers.
///
/// The reader function is called with a user argument which should be used
/// to pass a state argument to the reader, such as a CAN socket.
pub async fn read_message(sock: &CanSocket) -> Result<Frame, Error> {
    // Read packets until we find the starting header packet
    let pkt = loop {
        let pkt = read_frame(sock).await?;
        if (pkt.id == 0x400) && ((pkt.data >> 62) & 3) == 0 {
            break pkt;
        }
    };

    let header = read_header_0(pkt.data, None)?;
    let header = read_header_1(read_frame(sock).await?.data, Some(header))?;
    let header = read_header_2(read_frame(sock).await?.data, Some(header))?;

    let mut targets = [Target::default(); 256];

    for i in 0..header.n_targets as u32 {
        let pkt = read_frame(sock).await?;
        if 0x401 + i != pkt.id {
            Err(Error::OutOfSequence(format!(
                "expected target {} but got {}",
                0x401 + i,
                pkt.id
            )))?;
        }
        let target = read_data_0(pkt.data, None);

        let pkt = read_frame(sock).await?;
        if 0x401 + i != pkt.id {
            Err(Error::OutOfSequence(format!(
                "expected target {} but got {}",
                0x401 + i,
                pkt.id
            )))?;
        }
        let target = read_data_1(pkt.data, Some(target));

        targets[i as usize] = target;
    }

    Ok(Frame { header, targets })
}

/// Parse radar frame header from CAN data payload.
///
/// # Arguments
/// * `data` - 8-byte CAN frame data as u64
/// * `hdr` - Optional previous header for multi-packet accumulation
///
/// # Returns
/// Complete or partial Header structure
///
/// # Errors
/// Returns Error if header format is invalid
///
/// Alternative header parsing function for testing/debugging.
/// Kept for protocol documentation and future use.
#[allow(dead_code)]
pub fn read_header(data: u64, hdr: Option<Header>) -> Result<Header, Error> {
    match (data >> 62) & 3 {
        0 => read_header_0(data, hdr),
        1 => read_header_1(data, hdr),
        2 => read_header_2(data, hdr),
        val => Err(Error::InvalidHeader(format!("invalid header type {val}"))),
    }
}

fn read_header_0(data: u64, hdr: Option<Header>) -> Result<Header, Error> {
    if (data >> 62) & 3 != 0 {
        return Err(Error::OutOfSequence(format!(
            "expected header 0 but got {}",
            (data >> 62) & 3
        )));
    }

    let cycle_duration = (data & 0xFFF) as i32;
    let cycle_counter = ((data >> 15) & 0xFFFFFFFF) as u32;
    let n_targets = ((data >> 47) & 0xFF) as usize;
    let tx_antenna = ((data >> 56) & 0x3) as u8;
    let frequency_sweep = ((data >> 58) & 0x3) as u8;
    let center_frequency = ((data >> 60) & 0x3) as u8;

    match hdr {
        Some(hdr) => Ok(Header {
            seconds: hdr.seconds,
            nanoseconds: hdr.nanoseconds,
            cycle_duration: cycle_duration as f64 * 0.064,
            cycle_counter,
            n_targets,
            tx_antenna,
            frequency_sweep,
            center_frequency,
        }),
        None => Ok(Header {
            seconds: 0,
            nanoseconds: 0,
            cycle_duration: cycle_duration as f64 * 0.064,
            cycle_counter,
            n_targets,
            tx_antenna,
            frequency_sweep,
            center_frequency,
        }),
    }
}

fn read_header_1(data: u64, hdr: Option<Header>) -> Result<Header, Error> {
    if (data >> 62) & 3 != 1 {
        return Err(Error::OutOfSequence(format!(
            "expected header 1 but got {}",
            (data >> 62) & 3
        )));
    }

    match hdr {
        Some(hdr) => Ok(Header {
            seconds: hdr.seconds,
            nanoseconds: hdr.nanoseconds,
            cycle_duration: hdr.cycle_duration,
            cycle_counter: hdr.cycle_counter,
            n_targets: hdr.n_targets,
            tx_antenna: hdr.tx_antenna,
            frequency_sweep: hdr.frequency_sweep,
            center_frequency: hdr.center_frequency,
        }),
        None => Ok(Header {
            seconds: 0,
            nanoseconds: 0,
            cycle_duration: 0.0,
            cycle_counter: 0,
            n_targets: 0,
            tx_antenna: 0,
            frequency_sweep: 0,
            center_frequency: 0,
        }),
    }
}

fn read_header_2(data: u64, hdr: Option<Header>) -> Result<Header, Error> {
    if (data >> 62) & 3 != 2 {
        return Err(Error::OutOfSequence(format!(
            "expected header 2 but got {}",
            (data >> 62) & 3
        )));
    }

    match hdr {
        Some(hdr) => Ok(Header {
            seconds: hdr.seconds,
            nanoseconds: hdr.nanoseconds,
            cycle_duration: hdr.cycle_duration,
            cycle_counter: hdr.cycle_counter,
            n_targets: hdr.n_targets,
            tx_antenna: hdr.tx_antenna,
            frequency_sweep: hdr.frequency_sweep,
            center_frequency: hdr.center_frequency,
        }),
        None => Ok(Header {
            seconds: 0,
            nanoseconds: 0,
            cycle_duration: 0.0,
            cycle_counter: 0,
            n_targets: 0,
            tx_antenna: 0,
            frequency_sweep: 0,
            center_frequency: 0,
        }),
    }
}

/// Parse radar target data from CAN payload.
///
/// # Arguments
/// * `data` - 8-byte CAN frame data as u64
/// * `tgt` - Optional previous target for multi-packet accumulation
///
/// # Returns
/// Complete or partial Target structure
///
/// Alternative target data parsing function for testing/debugging.
/// Kept for protocol documentation and future use.
#[allow(dead_code)]
pub fn read_data(data: u64, tgt: Option<Target>) -> Target {
    match data & 1 != 0 {
        true => read_data_1(data, tgt),
        false => read_data_0(data, tgt),
    }
}

fn read_data_0(data: u64, tgt: Option<Target>) -> Target {
    let range = ((data >> 1) & 0x1FFF) as i32;
    let azimuth = ((data >> 22) & 0x3FF) as i32 - 511;
    let speed = ((data >> 39) & 0xFFF) as i32 - 2992;

    match tgt {
        Some(tgt) => Target {
            range: range as f64 * 0.04,
            azimuth: azimuth as f64 * 0.16,
            elevation: tgt.elevation,
            speed: speed as f64 * 0.04,
            rcs: tgt.rcs,
            power: tgt.power,
            noise: tgt.noise,
        },
        None => Target {
            range: range as f64 * 0.04,
            azimuth: azimuth as f64 * 0.16,
            elevation: 0.0,
            speed: speed as f64 * 0.04,
            rcs: 0.0,
            power: 0.0,
            noise: 0.0,
        },
    }
}

fn read_data_1(data: u64, tgt: Option<Target>) -> Target {
    let rcs = ((data >> 1) & 0xFF) as i32 - 75;
    let power = ((data >> 9) & 0xFF) as i32;
    let noise = ((data >> 17) & 0xFF) as i32;
    let elevation = ((data >> 37) & 0x3FF) as i32 - 511;

    match tgt {
        Some(tgt) => Target {
            range: tgt.range,
            azimuth: tgt.azimuth,
            elevation: elevation as f64 * 0.04,
            speed: tgt.speed,
            rcs: rcs as f64 * 0.2,
            power: power as f64,
            noise: noise as f64 * 0.5,
        },
        None => Target {
            range: 0.0,
            azimuth: 0.0,
            elevation: elevation as f64 * 0.04,
            speed: 0.0,
            rcs: rcs as f64 * 0.2,
            power: power as f64,
            noise: noise as f64 * 0.5,
        },
    }
}

fn load_data(data: &[u8]) -> u64 {
    u64::from_le_bytes(data[0..8].try_into().unwrap())
}

/// Read next CAN frame from socket.
///
/// # Arguments
/// * `can` - Active CAN socket
///
/// # Returns
/// Next Packet from CAN bus
///
/// # Errors
/// Returns Error if socket read fails
pub async fn read_frame(can: &CanSocket) -> Result<Packet, Error> {
    match can.read_frame().await {
        Ok(CanFrame::Data(frame)) => {
            let id = match frame.id() {
                CanId::Standard(id) => id.as_raw() as u32,
                CanId::Extended(id) => id.as_raw(),
            };
            Ok(Packet {
                id,
                data: load_data(frame.data()),
            })
        }
        Ok(CanFrame::Remote(frame)) => panic!("Unexpected remote frame: {:?}", frame),
        Ok(CanFrame::Error(frame)) => panic!("Unexpected error frame: {:?}", frame),
        Err(err) => Err(Error::Io(err)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_endian() {
        let msg = [0x62, 0xC1, 0x40, 0x55, 0x03, 0xD8, 0x0D, 0x00];
        let msg_swapped = [0x00, 0x0D, 0xD8, 0x03, 0x55, 0x40, 0xC1, 0x62];

        let mut msg2 = msg;
        msg2.reverse();

        assert_ne!(msg, msg_swapped);
        assert_eq!(msg2, msg_swapped);

        let data = load_data(&msg);
        assert_eq!(data, 0x000DD8035540C162);
    }

    #[test]
    fn test_parse_headers() {
        let msg0 = [0x5b, 0x83, 0x82, 0x32, 0x3b, 0x80, 0x88, 0x0c];
        let dat0 = load_data(&msg0);
        let hdr0 = read_header(dat0, None).unwrap();

        assert_eq!(
            hdr0,
            Header {
                seconds: 0,
                nanoseconds: 0,
                cycle_duration: 54.976,
                cycle_counter: 7759109,
                n_targets: 17,
                tx_antenna: 0,
                frequency_sweep: 3,
                center_frequency: 0,
            }
        );

        let msg1 = [0x89, 0x83, 0x06, 0x00, 0x00, 0x00, 0x00, 0x40];
        let dat1 = load_data(&msg1);
        let hdr1 = read_header(dat1, None).unwrap();

        assert_eq!(
            hdr1,
            Header {
                seconds: 0,
                nanoseconds: 0,
                cycle_duration: 0.0,
                cycle_counter: 0,
                n_targets: 0,
                tx_antenna: 0,
                frequency_sweep: 0,
                center_frequency: 0,
            }
        );

        let msg2 = [0x6a, 0x7c, 0x26, 0xa3, 0x00, 0x00, 0x00, 0x80];
        let dat2 = load_data(&msg2);
        let hdr2 = read_header(dat2, None).unwrap();

        assert_eq!(
            hdr2,
            Header {
                seconds: 0,
                nanoseconds: 0,
                cycle_duration: 0.0,
                cycle_counter: 0,
                n_targets: 0,
                tx_antenna: 0,
                frequency_sweep: 0,
                center_frequency: 0,
            }
        );
    }

    #[test]
    fn test_parse_targets() {
        let msg0 = [0x62, 0xC1, 0x40, 0x55, 0x03, 0xD8, 0x0D, 0x00];
        let data0 = load_data(&msg0);
        let target0 = read_data(data0, None);

        assert_eq!(
            target0,
            Target {
                range: 7.08,
                azimuth: -27.2,
                elevation: 0.0,
                speed: 0.0,
                rcs: 0.0,
                power: 0.0,
                noise: 0.0
            }
        );

        let msg1 = [0x6D, 0x0A, 0x7D, 0x01, 0x60, 0xCB, 0x01, 0x00];
        let data1 = load_data(&msg1);
        let target1 = read_data(data1, None);

        assert_eq!(
            target1,
            Target {
                range: 0.0,
                azimuth: 0.0,
                elevation: 3.68,
                speed: 0.0,
                rcs: -4.2,
                power: 133.0,
                noise: 95.0,
            }
        );
    }

    #[test]
    fn test_crc() {
        // From Smart Micro Systems User Application Note UATv4 Section 7.1
        let msg = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39];
        assert_eq!(State::<CCITT_FALSE>::calculate(&msg), 0x29B1);
    }

    #[test]
    fn test_request_crc() {
        let header = InstructionHeader {
            crc: 0,
            instructions: 1,
            device_id: 0,
            protocol_version: 4,
            message_index: 0,
            uat_id: 2010,
        };

        let message1 = InstructionMessage1 {
            dim0: 0,
            dim1: 0,
            parnum: 2,
            message_type: MessageType::ParameterWrite as u8,
            message_index: 1,
            uat_id: 2010,
        };

        let message2 = InstructionMessage2 {
            value: 0,
            format: 0,
            message_index: 2,
            uat_id: 2010,
        };

        let crc = message_crc(&header, &message1, &message2);
        assert_eq!(crc, 0xD5AB);
    }
}
