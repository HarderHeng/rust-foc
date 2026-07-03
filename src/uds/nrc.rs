//! Negative Response Codes (NRC) for UDS (ISO 14229).
//!
//! Each variant maps to a single byte on the wire. The full set
//! is per ISO 14229-1 §Annex B.2; we declare all the codes we'll
//! ever send rather than a minimal subset, so adding new SIDs
//! in future phases doesn't require touching this file.

use core::fmt;

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

impl fmt::Display for Nrc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::GeneralReject => "generalReject",
            Self::ServiceNotSupported => "serviceNotSupported",
            Self::SubFunctionNotSupported => "subFunctionNotSupported",
            Self::IncorrectMessageLengthOrInvalidFormat => "incorrectMessageLengthOrInvalidFormat",
            Self::ResponseTooLong => "responseTooLong",
            Self::BusyRepeatRequest => "busyRepeatRequest",
            Self::ConditionsNotCorrect => "conditionsNotCorrect",
            Self::RequestSequenceError => "requestSequenceError",
            Self::NoResponseFromSubnetComponent => "noResponseFromSubnetComponent",
            Self::FailurePreventsExecutionOfRequestedAction =>
                "failurePreventsExecutionOfRequestedAction",
            Self::RequestOutOfRange => "requestOutOfRange",
            Self::SecurityAccessDenied => "securityAccessDenied",
            Self::AuthenticationRequired => "authenticationRequired",
            Self::InvalidKey => "invalidKey",
            Self::ExceededNumberOfAttempts => "exceededNumberOfAttempts",
            Self::RequiredTimeDelayNotExpired => "requiredTimeDelayNotExpired",
            Self::UploadDownloadNotAccepted => "uploadDownloadNotAccepted",
            Self::TransferDataSuspended => "transferDataSuspended",
            Self::GeneralProgrammingFailure => "generalProgrammingFailure",
            Self::WrongBlockSequenceNumber => "wrongBlockSequenceNumber",
            Self::IllegalByteCountInBlockTransfer => "illegalByteCountInBlockTransfer",
            Self::RequestCorrectlyReceivedResponsePending =>
                "requestCorrectlyReceivedResponsePending",
            Self::SubFunctionNotSupportedInActiveSession =>
                "subFunctionNotSupportedInActiveSession",
            Self::ServiceNotSupportedInActiveSession =>
                "serviceNotSupportedInActiveSession",
        };
        f.write_str(s)
    }
}
