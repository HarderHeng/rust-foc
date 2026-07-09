//! UDS protocol-level types: enums that appear in the wire format
//! or in the `UdsConfig` schema.
//!
//! Per ISO 14229-1. The state-machine enum (`SrvState`) and the
//! response codes (`Nrc`) live here too — they're protocol
//! values, not engine state.

// ---- Session (0x10) ----

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Session {
    Default = 0x01,
    Programming = 0x02,
    Extended = 0x03,
}

impl Session {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::Default),
            0x02 => Some(Self::Programming),
            0x03 => Some(Self::Extended),
            _ => None,
        }
    }
    pub const fn as_u8(self) -> u8 { self as u8 }
}

// ---- SecurityLevel (0x27) ----

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecurityLevel {
    Locked = 0,
    Sal1 = 1,
    Sal2 = 2,
    Sal3 = 3,
}

impl SecurityLevel {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Locked,
            1 => Self::Sal1,
            2 => Self::Sal2,
            _ => Self::Sal3,
        }
    }
    #[allow(dead_code)]
    pub const fn as_u8(self) -> u8 { self as u8 }
}

// ---- SrvState (engine state machine) ----

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SrvState {
    /// Accept new requests.
    Idle,
    /// A long-running job is in the pending queue.
    Pending,
}

// ---- Nrc (ISO 14229-1 Annex B.2) ----

#[allow(dead_code)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Nrc {
    GeneralReject                                  = 0x10,
    ServiceNotSupported                            = 0x11,
    SubFunctionNotSupported                        = 0x12,
    IncorrectMessageLengthOrInvalidFormat          = 0x13,
    ResponseTooLong                                = 0x14,
    BusyRepeatRequest                              = 0x21,
    ConditionsNotCorrect                           = 0x22,
    RequestSequenceError                           = 0x24,
    NoResponseFromSubnetComponent                  = 0x25,
    FailurePreventsExecutionOfRequestedAction      = 0x26,
    RequestOutOfRange                              = 0x31,
    SecurityAccessDenied                           = 0x33,
    AuthenticationRequired                         = 0x34,
    InvalidKey                                     = 0x35,
    ExceededNumberOfAttempts                       = 0x36,
    RequiredTimeDelayNotExpired                    = 0x37,
    UploadDownloadNotAccepted                      = 0x70,
    TransferDataSuspended                          = 0x71,
    GeneralProgrammingFailure                      = 0x72,
    WrongBlockSequenceNumber                       = 0x73,
    IllegalByteCountInBlockTransfer                = 0x75,
    RequestCorrectlyReceivedResponsePending        = 0x78,
    SubFunctionNotSupportedInActiveSession         = 0x7E,
    ServiceNotSupportedInActiveSession             = 0x7F,
}

impl Nrc {
    pub const fn code(self) -> u8 { self as u8 }

    /// Standard negative response: `[0x7F, sid, nrc]`.
    pub const fn negative_response(self, sid: u8) -> [u8; 3] {
        [0x7F, sid, self.code()]
    }
}
