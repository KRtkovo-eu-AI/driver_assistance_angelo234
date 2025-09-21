-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local rshift = bit.rshift
local lshift = bit.lshift
local band = bit.band
local bor = bit.bor

local max = math.max
local fmod = math.fmod
local floor = math.floor
local frexp = math.frexp

local ffi = require("ffi")
M.canBusAPI = ffi.load("PCANBasic.dll")
ffi.cdef [[
//  PCANBasic.h
//
//  ~~~~~~~~~~~~
//
//  PCAN-Basic API
//
//  ~~~~~~~~~~~~
//
//  ------------------------------------------------------------------
//  Author : Keneth Wagner
//	Last change: 15.10.2020 Wagner
//
//  Language: ANSI-C
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2020  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com
//
////////////////////////////////////////////////////////////
// windows.h typedefs
////////////////////////////////////////////////////////////
typedef char CHAR;
typedef CHAR *LPSTR;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;
typedef unsigned char       BYTE;
typedef unsigned __int64    UINT64, *PUINT64;

////////////////////////////////////////////////////////////
// Value definitions
////////////////////////////////////////////////////////////

// Currently defined and supported PCAN channels
//
enum PCANChannel
{
    PCAN_NONEBUS = 0x00U, // Undefined/default value for a PCAN bus

    PCAN_ISABUS1 = 0x21U, // PCAN-ISA interface, channel 1
    PCAN_ISABUS2 = 0x22U, // PCAN-ISA interface, channel 2
    PCAN_ISABUS3 = 0x23U, // PCAN-ISA interface, channel 3
    PCAN_ISABUS4 = 0x24U, // PCAN-ISA interface, channel 4
    PCAN_ISABUS5 = 0x25U, // PCAN-ISA interface, channel 5
    PCAN_ISABUS6 = 0x26U, // PCAN-ISA interface, channel 6
    PCAN_ISABUS7 = 0x27U, // PCAN-ISA interface, channel 7
    PCAN_ISABUS8 = 0x28U, // PCAN-ISA interface, channel 8

    PCAN_DNGBUS1 = 0x31U,   // PCAN-Dongle/LPT interface, channel 1
    PCAN_PCIBUS1 = 0x41U,   // PCAN-PCI interface, channel 1
    PCAN_PCIBUS2 = 0x42U,   // PCAN-PCI interface, channel 2
    PCAN_PCIBUS3 = 0x43U,   // PCAN-PCI interface, channel 3
    PCAN_PCIBUS4 = 0x44U,   // PCAN-PCI interface, channel 4
    PCAN_PCIBUS5 = 0x45U,   // PCAN-PCI interface, channel 5
    PCAN_PCIBUS6 = 0x46U,   // PCAN-PCI interface, channel 6
    PCAN_PCIBUS7 = 0x47U,   // PCAN-PCI interface, channel 7
    PCAN_PCIBUS8 = 0x48U,   // PCAN-PCI interface, channel 8
    PCAN_PCIBUS9 = 0x409U,  // PCAN-PCI interface, channel 9
    PCAN_PCIBUS10 = 0x40AU, // PCAN-PCI interface, channel 10
    PCAN_PCIBUS11 = 0x40BU, // PCAN-PCI interface, channel 11
    PCAN_PCIBUS12 = 0x40CU, // PCAN-PCI interface, channel 12
    PCAN_PCIBUS13 = 0x40DU, // PCAN-PCI interface, channel 13
    PCAN_PCIBUS14 = 0x40EU, // PCAN-PCI interface, channel 14
    PCAN_PCIBUS15 = 0x40FU, // PCAN-PCI interface, channel 15
    PCAN_PCIBUS16 = 0x410U, // PCAN-PCI interface, channel 16

    PCAN_USBBUS1 = 0x51U,   // PCAN-USB interface, channel 1
    PCAN_USBBUS2 = 0x52U,   // PCAN-USB interface, channel 2
    PCAN_USBBUS3 = 0x53U,   // PCAN-USB interface, channel 3
    PCAN_USBBUS4 = 0x54U,   // PCAN-USB interface, channel 4
    PCAN_USBBUS5 = 0x55U,   // PCAN-USB interface, channel 5
    PCAN_USBBUS6 = 0x56U,   // PCAN-USB interface, channel 6
    PCAN_USBBUS7 = 0x57U,   // PCAN-USB interface, channel 7
    PCAN_USBBUS8 = 0x58U,   // PCAN-USB interface, channel 8
    PCAN_USBBUS9 = 0x509U,  // PCAN-USB interface, channel 9
    PCAN_USBBUS10 = 0x50AU, // PCAN-USB interface, channel 10
    PCAN_USBBUS11 = 0x50BU, // PCAN-USB interface, channel 11
    PCAN_USBBUS12 = 0x50CU, // PCAN-USB interface, channel 12
    PCAN_USBBUS13 = 0x50DU, // PCAN-USB interface, channel 13
    PCAN_USBBUS14 = 0x50EU, // PCAN-USB interface, channel 14
    PCAN_USBBUS15 = 0x50FU, // PCAN-USB interface, channel 15
    PCAN_USBBUS16 = 0x510U, // PCAN-USB interface, channel 16

    PCAN_PCCBUS1 = 0x61U, // PCAN-PC Card interface, channel 1
    PCAN_PCCBUS2 = 0x62U, // PCAN-PC Card interface, channel 2

    PCAN_LANBUS1 = 0x801U,  // PCAN-LAN interface, channel 1
    PCAN_LANBUS2 = 0x802U,  // PCAN-LAN interface, channel 2
    PCAN_LANBUS3 = 0x803U,  // PCAN-LAN interface, channel 3
    PCAN_LANBUS4 = 0x804U,  // PCAN-LAN interface, channel 4
    PCAN_LANBUS5 = 0x805U,  // PCAN-LAN interface, channel 5
    PCAN_LANBUS6 = 0x806U,  // PCAN-LAN interface, channel 6
    PCAN_LANBUS7 = 0x807U,  // PCAN-LAN interface, channel 7
    PCAN_LANBUS8 = 0x808U,  // PCAN-LAN interface, channel 8
    PCAN_LANBUS9 = 0x809U,  // PCAN-LAN interface, channel 9
    PCAN_LANBUS10 = 0x80AU, // PCAN-LAN interface, channel 10
    PCAN_LANBUS11 = 0x80BU, // PCAN-LAN interface, channel 11
    PCAN_LANBUS12 = 0x80CU, // PCAN-LAN interface, channel 12
    PCAN_LANBUS13 = 0x80DU, // PCAN-LAN interface, channel 13
    PCAN_LANBUS14 = 0x80EU, // PCAN-LAN interface, channel 14
    PCAN_LANBUS15 = 0x80FU, // PCAN-LAN interface, channel 15
    PCAN_LANBUS16 = 0x810U, // PCAN-LAN interface, channel 16
};

// Represent the PCAN error and status codes
//
enum PCANError
{
    PCAN_ERROR_OK = 0x00000U,                                                                                                               // No error
    PCAN_ERROR_XMTFULL = 0x00001U,                                                                                                          // Transmit buffer in CAN controller is full
    PCAN_ERROR_OVERRUN = 0x00002U,                                                                                                          // CAN controller was read too late
    PCAN_ERROR_BUSLIGHT = 0x00004U,                                                                                                         // Bus error: an error counter reached the 'light' limit
    PCAN_ERROR_BUSHEAVY = 0x00008U,                                                                                                         // Bus error: an error counter reached the 'heavy' limit
    PCAN_ERROR_BUSWARNING = PCAN_ERROR_BUSHEAVY,                                                                                            // Bus error: an error counter reached the 'warning' limit
    PCAN_ERROR_BUSPASSIVE = 0x40000U,                                                                                                       // Bus error: the CAN controller is error passive
    PCAN_ERROR_BUSOFF = 0x00010U,                                                                                                           // Bus error: the CAN controller is in bus-off state
    PCAN_ERROR_ANYBUSERR = (PCAN_ERROR_BUSWARNING | PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSOFF | PCAN_ERROR_BUSPASSIVE), // Mask for all bus errors
    PCAN_ERROR_QRCVEMPTY = 0x00020U,                                                                                                        // Receive queue is empty
    PCAN_ERROR_QOVERRUN = 0x00040U,                                                                                                         // Receive queue was read too late
    PCAN_ERROR_QXMTFULL = 0x00080U,                                                                                                         // Transmit queue is full
    PCAN_ERROR_REGTEST = 0x00100U,                                                                                                          // Test of the CAN controller hardware registers failed (no hardware found)
    PCAN_ERROR_NODRIVER = 0x00200U,                                                                                                         // Driver not loaded
    PCAN_ERROR_HWINUSE = 0x00400U,                                                                                                          // Hardware already in use by a Net
    PCAN_ERROR_NETINUSE = 0x00800U,                                                                                                         // A Client is already connected to the Net
    PCAN_ERROR_ILLHW = 0x01400U,                                                                                                            // Hardware handle is invalid
    PCAN_ERROR_ILLNET = 0x01800U,                                                                                                           // Net handle is invalid
    PCAN_ERROR_ILLCLIENT = 0x01C00U,                                                                                                        // Client handle is invalid
    PCAN_ERROR_ILLHANDLE = (PCAN_ERROR_ILLHW | PCAN_ERROR_ILLNET | PCAN_ERROR_ILLCLIENT),                                                   // Mask for all handle errors
    PCAN_ERROR_RESOURCE = 0x02000U,                                                                                                         // Resource (FIFO, Client, timeout) cannot be created
    PCAN_ERROR_ILLPARAMTYPE = 0x04000U,                                                                                                     // Invalid parameter
    PCAN_ERROR_ILLPARAMVAL = 0x08000U,                                                                                                      // Invalid parameter value
    PCAN_ERROR_UNKNOWN = 0x10000U,                                                                                                          // Unknown error
    PCAN_ERROR_ILLDATA = 0x20000U,                                                                                                          // Invalid data, function, or action
    PCAN_ERROR_ILLMODE = 0x80000U,                                                                                                          // Driver object state is wrong for the attempted operation
    PCAN_ERROR_CAUTION = 0x2000000U,                                                                                                        // An operation was successfully carried out, however, irregularities were registered
    PCAN_ERROR_INITIALIZE = 0x4000000U,                                                                                                     // Channel is not initialized [Value was changed from 0x40000 to 0x4000000]
    PCAN_ERROR_ILLOPERATION = 0x8000000U,                                                                                                   // Invalid operation [Value was changed from 0x80000 to 0x8000000]
};

// PCAN devices
//
enum PCANDevice
{
    PCAN_NONE = 0x00U,    // Undefined, unknown or not selected PCAN device value
    PCAN_PEAKCAN = 0x01U, // PCAN Non-PnP devices. NOT USED WITHIN PCAN-Basic API
    PCAN_ISA = 0x02U,     // PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus
    PCAN_DNG = 0x03U,     // PCAN-Dongle
    PCAN_PCI = 0x04U,     // PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express
    PCAN_USB = 0x05U,     // PCAN-USB and PCAN-USB Pro
    PCAN_PCC = 0x06U,     // PCAN-PC Card
    PCAN_VIRTUAL = 0x07U, // PCAN Virtual hardware. NOT USED WITHIN PCAN-Basic API
    PCAN_LAN = 0x08U,     // PCAN Gateway devices
};

// PCAN parameters
//
enum PCANParameter
{
    PCAN_DEVICE_ID = 0x01U,                // Device identifier parameter
    PCAN_5VOLTS_POWER = 0x02U,             // 5-Volt power parameter
    PCAN_RECEIVE_EVENT = 0x03U,            // PCAN receive event handler parameter
    PCAN_MESSAGE_FILTER = 0x04U,           // PCAN message filter parameter
    PCAN_API_VERSION = 0x05U,              // PCAN-Basic API version parameter
    PCAN_CHANNEL_VERSION = 0x06U,          // PCAN device channel version parameter
    PCAN_BUSOFF_AUTORESET = 0x07U,         // PCAN Reset-On-Busoff parameter
    PCAN_LISTEN_ONLY = 0x08U,              // PCAN Listen-Only parameter
    PCAN_LOG_LOCATION = 0x09U,             // Directory path for log files
    PCAN_LOG_STATUS = 0x0AU,               // Debug-Log activation status
    PCAN_LOG_CONFIGURE = 0x0BU,            // Configuration of the debugged information (LOG_FUNCTION_***)
    PCAN_LOG_TEXT = 0x0CU,                 // Custom insertion of text into the log file
    PCAN_CHANNEL_CONDITION = 0x0DU,        // Availability status of a PCAN-Channel
    PCAN_HARDWARE_NAME = 0x0EU,            // PCAN hardware name parameter
    PCAN_RECEIVE_STATUS = 0x0FU,           // Message reception status of a PCAN-Channel
    PCAN_CONTROLLER_NUMBER = 0x10U,        // CAN-Controller number of a PCAN-Channel
    PCAN_TRACE_LOCATION = 0x11U,           // Directory path for PCAN trace files
    PCAN_TRACE_STATUS = 0x12U,             // CAN tracing activation status
    PCAN_TRACE_SIZE = 0x13U,               // Configuration of the maximum file size of a CAN trace
    PCAN_TRACE_CONFIGURE = 0x14U,          // Configuration of the trace file storing mode (TRACE_FILE_***)
    PCAN_CHANNEL_IDENTIFYING = 0x15U,      // Physical identification of a USB based PCAN-Channel by blinking its associated LED
    PCAN_CHANNEL_FEATURES = 0x16U,         // Capabilities of a PCAN device (FEATURE_***)
    PCAN_BITRATE_ADAPTING = 0x17U,         // Using of an existing bit rate (PCAN-View connected to a channel)
    PCAN_BITRATE_INFO = 0x18U,             // Configured bit rate as Btr0Btr1 value
    PCAN_BITRATE_INFO_FD = 0x19U,          // Configured bit rate as TPCANBitrateFD string
    PCAN_BUSSPEED_NOMINAL = 0x1AU,         // Configured nominal CAN Bus speed as Bits per seconds
    PCAN_BUSSPEED_DATA = 0x1BU,            // Configured CAN data speed as Bits per seconds
    PCAN_IP_ADDRESS = 0x1CU,               // Remote address of a LAN channel as string in IPv4 format
    PCAN_LAN_SERVICE_STATUS = 0x1DU,       // Status of the Virtual PCAN-Gateway Service
    PCAN_ALLOW_STATUS_FRAMES = 0x1EU,      // Status messages reception status within a PCAN-Channel
    PCAN_ALLOW_RTR_FRAMES = 0x1FU,         // RTR messages reception status within a PCAN-Channel
    PCAN_ALLOW_ERROR_FRAMES = 0x20U,       // Error messages reception status within a PCAN-Channel
    PCAN_INTERFRAME_DELAY = 0x21U,         // Delay, in microseconds, between sending frames
    PCAN_ACCEPTANCE_FILTER_11BIT = 0x22U,  // Filter over code and mask patterns for 11-Bit messages
    PCAN_ACCEPTANCE_FILTER_29BIT = 0x23U,  // Filter over code and mask patterns for 29-Bit messages
    PCAN_IO_DIGITAL_CONFIGURATION = 0x24U, // Output mode of 32 digital I/O pin of a PCAN-USB Chip. 1: Output-Active 0 : Output Inactive
    PCAN_IO_DIGITAL_VALUE = 0x25U,         // Value assigned to a 32 digital I/O pins of a PCAN-USB Chip
    PCAN_IO_DIGITAL_SET = 0x26U,           // Value assigned to a 32 digital I/O pins of a PCAN-USB Chip - Multiple digital I/O pins to 1 = High
    PCAN_IO_DIGITAL_CLEAR = 0x27U,         // Clear multiple digital I/O pins to 0
    PCAN_IO_ANALOG_VALUE = 0x28U,          // Get value of a single analog input pin
    PCAN_FIRMWARE_VERSION = 0x29U,         // Get the version of the firmware used by the device associated with a PCAN-Channel
    PCAN_ATTACHED_CHANNELS_COUNT = 0x2AU,  // Get the amount of PCAN channels attached to a system
    PCAN_ATTACHED_CHANNELS = 0x2BU,        // Get information about PCAN channels attached to a system

    // DEPRECATED parameters
    //
    PCAN_DEVICE_NUMBER = PCAN_DEVICE_ID, // Deprecated parameter. Use PCAN_DEVICE_ID instead
};

// PCAN parameter values
//
enum PCANParamterValue
{
    PCAN_PARAMETER_OFF = 0x00U,                                               // The PCAN parameter is not set (inactive)
    PCAN_PARAMETER_ON = 0x01U,                                                // The PCAN parameter is set (active)
    PCAN_FILTER_CLOSE = 0x00U,                                                // The PCAN filter is closed. No messages will be received
    PCAN_FILTER_OPEN = 0x01U,                                                 // The PCAN filter is fully opened. All messages will be received
    PCAN_FILTER_CUSTOM = 0x02U,                                               // The PCAN filter is custom configured. Only registered messages will be received
    PCAN_CHANNEL_UNAVAILABLE = 0x00U,                                         // The PCAN-Channel handle is illegal, or its associated hardware is not available
    PCAN_CHANNEL_AVAILABLE = 0x01U,                                           // The PCAN-Channel handle is available to be connected (PnP Hardware: it means furthermore that the hardware is plugged-in)
    PCAN_CHANNEL_OCCUPIED = 0x02U,                                            // The PCAN-Channel handle is valid, and is already being used
    PCAN_CHANNEL_PCANVIEW = (PCAN_CHANNEL_AVAILABLE | PCAN_CHANNEL_OCCUPIED), // The PCAN-Channel handle is already being used by a PCAN-View application, but is available to connect

    LOG_FUNCTION_DEFAULT = 0x00U,    // Logs system exceptions / errors
    LOG_FUNCTION_ENTRY = 0x01U,      // Logs the entries to the PCAN-Basic API functions
    LOG_FUNCTION_PARAMETERS = 0x02U, // Logs the parameters passed to the PCAN-Basic API functions
    LOG_FUNCTION_LEAVE = 0x04U,      // Logs the exits from the PCAN-Basic API functions
    LOG_FUNCTION_WRITE = 0x08U,      // Logs the CAN messages passed to the CAN_Write function
    LOG_FUNCTION_READ = 0x10U,       // Logs the CAN messages received within the CAN_Read function
    LOG_FUNCTION_ALL = 0xFFFFU,      // Logs all possible information within the PCAN-Basic API functions

    TRACE_FILE_SINGLE = 0x00U,    // A single file is written until it size reaches PAN_TRACE_SIZE
    TRACE_FILE_SEGMENTED = 0x01U, // Traced data is distributed in several files with size PAN_TRACE_SIZE
    TRACE_FILE_DATE = 0x02U,      // Includes the date into the name of the trace file
    TRACE_FILE_TIME = 0x04U,      // Includes the start time into the name of the trace file
    TRACE_FILE_OVERWRITE = 0x80U, // Causes the overwriting of available traces (same name)

    FEATURE_FD_CAPABLE = 0x01U,    // Device supports flexible data-rate (CAN-FD)
    FEATURE_DELAY_CAPABLE = 0x02U, // Device supports a delay between sending frames (FPGA based USB devices)
    FEATURE_IO_CAPABLE = 0x04U,    // Device supports I/O functionality for electronic circuits (USB-Chip devices)

    SERVICE_STATUS_STOPPED = 0x01U, // The service is not running
    SERVICE_STATUS_RUNNING = 0x04U, // The service is running
};

// Other constants
//
enum PCANConstant
{
    MAX_LENGTH_HARDWARE_NAME = 33,  // Maximum length of the name of a device: 32 characters + terminator
    MAX_LENGTH_VERSION_STRING = 18, // Maximum length of a version string: 17 characters + terminator
};

// PCAN message types
//
enum PCANMessageType
{
    PCAN_MESSAGE_STANDARD = 0x00U, // The PCAN message is a CAN Standard Frame (11-bit identifier)
    PCAN_MESSAGE_RTR = 0x01U,      // The PCAN message is a CAN Remote-Transfer-Request Frame
    PCAN_MESSAGE_EXTENDED = 0x02U, // The PCAN message is a CAN Extended Frame (29-bit identifier)
    PCAN_MESSAGE_FD = 0x04U,       // The PCAN message represents a FD frame in terms of CiA Specs
    PCAN_MESSAGE_BRS = 0x08U,      // The PCAN message represents a FD bit rate switch (CAN data at a higher bit rate)
    PCAN_MESSAGE_ESI = 0x10U,      // The PCAN message represents a FD error state indicator(CAN FD transmitter was error active)
    PCAN_MESSAGE_ERRFRAME = 0x40U, // The PCAN message represents an error frame
    PCAN_MESSAGE_STATUS = 0x80U,   // The PCAN message represents a PCAN status message
};

// Frame Type / Initialization Mode
//
enum PCANFrameType
{
    PCAN_MODE_STANDARD = PCAN_MESSAGE_STANDARD,
    PAN_MODE_EXTENDED = PCAN_MESSAGE_EXTENDED,
};

// Baud rate codes = BTR0/BTR1 register values for the CAN controller.
// You can define your own Baud rate with the BTROBTR1 register.
// Take a look at www.peak-system.com for our free software "BAUDTOOL"
// to calculate the BTROBTR1 register for every bit rate and sample point.
//
enum PCANBaudRateCode
{
    PCAN_BAUD_1M = 0x0014U,   //   1 MBit/s
    PCAN_BAUD_800K = 0x0016U, // 800 kBit/s
    PCAN_BAUD_500K = 0x001CU, // 500 kBit/s
    PCAN_BAUD_250K = 0x011CU, // 250 kBit/s
    PCAN_BAUD_125K = 0x031CU, // 125 kBit/s
    PCAN_BAUD_100K = 0x432FU, // 100 kBit/s
    PCAN_BAUD_95K = 0xC34EU,  //  95,238 kBit/s
    PCAN_BAUD_83K = 0x852BU,  //  83,333 kBit/s
    PCAN_BAUD_50K = 0x472FU,  //  50 kBit/s
    PCAN_BAUD_47K = 0x1414U,  //  47,619 kBit/s
    PCAN_BAUD_33K = 0x8B2FU,  //  33,333 kBit/s
    PCAN_BAUD_20K = 0x532FU,  //  20 kBit/s
    PCAN_BAUD_10K = 0x672FU,  //  10 kBit/s
    PCAN_BAUD_5K = 0x7F7FU,   //   5 kBit/s
};

// Type of PCAN (Non-PnP) hardware
//
enum PCANHardwareType
{
    PCAN_TYPE_ISA = 0x01U,         // PCAN-ISA 82C200
    PCAN_TYPE_ISA_SJA = 0x09U,     // PCAN-ISA SJA1000
    PCAN_TYPE_ISA_PHYTEC = 0x04U,  // PHYTEC ISA
    PCAN_TYPE_DNG = 0x02U,         // PCAN-Dongle 82C200
    PCAN_TYPE_DNG_EPP = 0x03U,     // PCAN-Dongle EPP 82C200
    PCAN_TYPE_DNG_SJA = 0x05U,     // PCAN-Dongle SJA1000
    PCAN_TYPE_DNG_SJA_EPP = 0x06U, // PCAN-Dongle EPP SJA1000
};

////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////

typedef WORD TPCANHandle;        // Represents a PCAN hardware channel handle
typedef DWORD TPCANStatus;       // Represents a PCAN status/error code
typedef BYTE TPCANParameter;     // Represents a PCAN parameter to be read or set
typedef BYTE TPCANDevice;        // Represents a PCAN device
typedef BYTE TPCANMessageType;   // Represents the type of a PCAN message
typedef BYTE TPCANType;          // Represents the type of PCAN hardware to be initialized
typedef BYTE TPCANMode;          // Represents a PCAN filter mode
typedef WORD TPCANBaudrate;      // Represents a PCAN Baud rate register value
typedef LPSTR TPCANBitrateFD;    // Represents a PCAN-FD bit rate string
typedef UINT64 TPCANTimestampFD; // Represents a timestamp of a received PCAN FD message

////////////////////////////////////////////////////////////
// Structure definitions
////////////////////////////////////////////////////////////

// Represents a PCAN message
//
typedef struct tagTPCANMsg
{
    DWORD ID;                 // 11/29-bit message identifier
    TPCANMessageType MSGTYPE; // Type of the message
    BYTE LEN;                 // Data Length Code of the message (0..8)
    BYTE DATA[8];             // Data of the message (DATA[0]..DATA[7])
} TPCANMsg;

// Represents a timestamp of a received PCAN message
// Total Microseconds = micros + 1000 * millis + 0x100000000 * 1000 * millis_overflow
//
typedef struct tagTPCANTimestamp
{
    DWORD millis;         // Base-value: milliseconds: 0.. 2^32-1
    WORD millis_overflow; // Roll-arounds of millis
    WORD micros;          // Microseconds: 0..999
} TPCANTimestamp;

// Represents a PCAN message from a FD capable hardware
//
typedef struct tagTPCANMsgFD
{
    DWORD ID;                 // 11/29-bit message identifier
    TPCANMessageType MSGTYPE; // Type of the message
    BYTE DLC;                 // Data Length Code of the message (0..15)
    BYTE DATA[64];            // Data of the message (DATA[0]..DATA[63])
} TPCANMsgFD;

// Describes an available PCAN channel
//
typedef struct tagTPCANChannelInformation
{
    TPCANHandle channel_handle;                 // PCAN channel handle
    TPCANDevice device_type;                    // Kind of PCAN device
    BYTE controller_number;                     // CAN-Controller number
    DWORD device_features;                      // Device capabilities flag (see FEATURE_*)
    char device_name[MAX_LENGTH_HARDWARE_NAME]; // Device name
    DWORD device_id;                            // Device number
    DWORD channel_condition;                    // Availability status of a PCAN-Channel
} TPCANChannelInformation;

////////////////////////////////////////////////////////////
// PCAN-Basic API function declarations
////////////////////////////////////////////////////////////

/// <summary>
/// Initializes a PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Btr0Btr1">"The speed for the communication (BTR0BTR1 code)"</param>
/// <param name="HwType">"Non-PnP: The type of hardware and operation mode"</param>
/// <param name="IOPort">"Non-PnP: The I/O address for the parallel port"</param>
/// <param name="Interrupt">"Non-PnP: Interrupt number of the parallel port"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_Initialize(
    TPCANHandle Channel,
    TPCANBaudrate Btr0Btr1,
    TPCANType HwType,
    DWORD IOPort,
    WORD Interrupt);

/// <summary>
/// Initializes a FD capable PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a FD capable PCAN Channel"</param>
/// <param name="BitrateFD">"The speed for the communication (FD bit rate string)"</param>
/// <remarks>See PCAN_BR_* values
/// * Parameter and values must be separated by '='
/// * Couples of Parameter/value must be separated by ','
/// * Following Parameter must be filled out: f_clock, data_brp, data_sjw, data_tseg1, data_tseg2,
///   nom_brp, nom_sjw, nom_tseg1, nom_tseg2.
/// * Following Parameters are optional (not used yet): data_ssp_offset, nom_sam
///</remarks>
/// <example>f_clock=80000000,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1</example>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_InitializeFD(
    TPCANHandle Channel,
    TPCANBitrateFD BitrateFD);

/// <summary>
/// Uninitializes one or all PCAN Channels initialized by CAN_Initialize
/// </summary>
/// <remarks>Giving the TPCANHandle value "PCAN_NONEBUS",
/// uninitialize all initialized channels</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_Uninitialize(
    TPCANHandle Channel);

/// <summary>
/// Resets the receive and transmit queues of the PCAN Channel
/// </summary>
/// <remarks>
/// A reset of the CAN controller is not performed.
/// </remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_Reset(
    TPCANHandle Channel);

/// <summary>
/// Gets the current status of a PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_GetStatus(
    TPCANHandle Channel);

/// <summary>
/// Reads a CAN message from the receive queue of a PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsg structure buffer to store the CAN message"</param>
/// <param name="TimestampBuffer">"A TPCANTimestamp structure buffer to get
/// the reception time of the message. If this value is not desired, this parameter
/// should be passed as NULL"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_Read(
    TPCANHandle Channel,
    TPCANMsg *MessageBuffer,
    TPCANTimestamp *TimestampBuffer);

/// <summary>
/// Reads a CAN message from the receive queue of a FD capable PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a FD capable PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsgFD structure buffer to store the CAN message"</param>
/// <param name="TimestampBuffer">"A TPCANTimestampFD buffer to get
/// the reception time of the message. If this value is not desired, this parameter
/// should be passed as NULL"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_ReadFD(
    TPCANHandle Channel,
    TPCANMsgFD *MessageBuffer,
    TPCANTimestampFD *TimestampBuffer);

/// <summary>
/// Transmits a CAN message
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsg buffer with the message to be sent"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_Write(
    TPCANHandle Channel,
    TPCANMsg *MessageBuffer);

/// <summary>
/// Transmits a CAN message over a FD capable PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a FD capable PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsgFD buffer with the message to be sent"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_WriteFD(
    TPCANHandle Channel,
    TPCANMsgFD *MessageBuffer);

/// <summary>
/// Configures the reception filter.
/// </summary>
/// <remarks>The message filter will be expanded with every call to
/// this function. If it is desired to reset the filter, please use
/// the CAN_SetValue function</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="FromID">"The lowest CAN ID to be received"</param>
/// <param name="ToID">"The highest CAN ID to be received"</param>
/// <param name="Mode">"Message type, Standard (11-bit identifier) or
/// Extended (29-bit identifier)"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_FilterMessages(
    TPCANHandle Channel,
    DWORD FromID,
    DWORD ToID,
    TPCANMode Mode);

/// <summary>
/// Retrieves a PCAN Channel value
/// </summary>
/// <remarks>Parameters can be present or not according with the kind
/// of Hardware (PCAN Channel) being used. If a parameter is not available,
/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Parameter">"The TPCANParameter parameter to get"</param>
/// <param name="Buffer">"Buffer for the parameter value"</param>
/// <param name="BufferLength">"Size in bytes of the buffer"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_GetValue(
    TPCANHandle Channel,
    TPCANParameter Parameter,
    void *Buffer,
    DWORD BufferLength);

/// <summary>
/// Configures or sets a PCAN Channel value
/// </summary>
/// <remarks>Parameters can be present or not according with the kind
/// of Hardware (PCAN Channel) being used. If a parameter is not available,
/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Parameter">"The TPCANParameter parameter to set"</param>
/// <param name="Buffer">"Buffer with the value to be set"</param>
/// <param name="BufferLength">"Size in bytes of the buffer"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_SetValue(
    TPCANHandle Channel,
    TPCANParameter Parameter,
    void *Buffer,
    DWORD BufferLength);

/// <summary>
/// Returns a descriptive text of a given TPCANStatus error
/// code, in any desired language
/// </summary>
/// <remarks>The current languages available for translation are:
/// Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
/// Italian (0x10) and French (0x0C)</remarks>
/// <param name="Error">"A TPCANStatus error code"</param>
/// <param name="Language">"Indicates a 'Primary language ID'"</param>
/// <param name="Buffer">"Buffer for a null terminated char array"</param>
/// <returns>"A TPCANStatus error code"</returns>
TPCANStatus __stdcall CAN_GetErrorText(
    TPCANStatus Error,
    WORD Language,
    LPSTR Buffer);

/// <summary>
/// Finds a PCAN-Basic channel that matches with the given parameters
/// </summary>
/// <param name="Parameters">A comma separated string contained pairs of
/// parameter-name/value to be matched within a PCAN-Basic channel</param>
/// <param name="FoundChannel">Buffer for returning the PCAN-Basic channel,
/// when found</param>
/// <returns>A TPCANStatus error code</returns>
TPCANStatus __stdcall CAN_LookUpChannel(
    LPSTR Parameters,
    TPCANHandle *FoundChannel);
]]

M.PCANHandles = {
  NoneBus = 0x0,
  --ISA
  ISABus1 = 0x21,
  ISABus2 = 0x22,
  ISABus3 = 0x23,
  ISABus4 = 0x24,
  ISABus5 = 0x25,
  ISABus6 = 0x26,
  ISABus7 = 0x27,
  ISABus8 = 0x28,
  --Dongle
  DNGBus1 = 0x31,
  --PCI
  PCIBus1 = 0x41,
  PCIBus2 = 0x42,
  PCIBus3 = 0x43,
  PCIBus4 = 0x44,
  PCIBus5 = 0x45,
  PCIBus6 = 0x46,
  PCIBus7 = 0x47,
  PCIBus8 = 0x48,
  PCIBus9 = 0x409,
  PCIBus10 = 0x40A,
  PCIBus11 = 0x40B,
  PCIBus12 = 0x40C,
  PCIBus13 = 0x40D,
  PCIBus14 = 0x40E,
  PCIBus15 = 0x40F,
  PCIBus16 = 0x410,
  --USB
  USBBus1 = 0x51,
  USBBus2 = 0x52,
  USBBus3 = 0x53,
  USBBus4 = 0x54,
  USBBus5 = 0x55,
  USBBus6 = 0x56,
  USBBus7 = 0x57,
  USBBus8 = 0x58,
  USBBus9 = 0x509,
  USBBus10 = 0x50A,
  USBBus11 = 0x50B,
  USBBus12 = 0x50C,
  USBBus13 = 0x50D,
  USBBus14 = 0x50E,
  USBBus15 = 0x50F,
  USBBus16 = 0x510,
  --PC Card
  PCCBus1 = 0x61,
  PCCBus2 = 0x62,
  --LAN
  LANBus1 = 0x801,
  LANBus2 = 0x802,
  LANBus3 = 0x803,
  LANBus4 = 0x804,
  LANBus5 = 0x805,
  LANBus6 = 0x806,
  LANBus7 = 0x807,
  LANBus8 = 0x808,
  LANBus9 = 0x809,
  LANBus10 = 0x80a,
  LANBus11 = 0x80b,
  LANBus12 = 0x80c,
  LANBus13 = 0x80d,
  LANBus14 = 0x80e,
  LANBus15 = 0x80f,
  LANBus16 = 0x810
}

M.baudrates = {
  Baud1M = 0x0014,
  Baud800K = 0x0016,
  Baud500K = 0x001C,
  Baud250K = 0x011C,
  Baud125K = 0x031C,
  Baud100K = 0x432F,
  Baud95K = 0xC34E,
  Baud83K = 0x852B,
  Baud50K = 0x472F,
  Baud47K = 0x1414,
  Baud33K = 0x8B2F,
  Baud20K = 0x532F,
  Baud10K = 0x672FF,
  Baud5K = 0x7F7F
}

M.errorCodes = {
  OK = 0x00000,
  XMTFull = 0x00001,
  Overrun = 0x00002,
  BusLight = 0x00004,
  BusHeavy = 0x00008,
  BusWarning = 0x00008, --same as BusHeavy
  BusPassive = 0x40000,
  BusOff = 0x00010,
  QRCVEmpty = 0x00020,
  QOverrun = 0x00040,
  QXMTFull = 0x00080,
  RegTest = 0x00100,
  NoDriver = 0x00200,
  HWInUse = 0x00400,
  NetInUse = 0x00800,
  IllHw = 0x01400,
  IllNet = 0x01800,
  IllClient = 0x01C00,
  Resource = 0x02000,
  IllParamType = 0x04000,
  IllParamVal = 0x08000,
  Unknown = 0x10000,
  IllData = 0x20000,
  IllMode = 0x80000,
  Caution = 0x2000000,
  Initialize = 0x4000000,
  IllOperation = 0x8000000
}

M.messageType = {
  Standard = 0x00,
  RTR = 0x01,
  Extended = 0x02,
  FD = 0x04,
  BRS = 0x08,
  ESI = 0x10,
  Echo = 0x20,
  ErrFrame = 0x40,
  Status = 0x80
}

M.CANType = {
  ISA = 0x01,
  ISA_SJA = 0x09,
  ISA_PHYTEC = 0x04,
  DNG = 0x02,
  DNG_EPP = 0x03,
  DNG_SJA = 0x05,
  DNG_SJA_EPP = 0x06
}

M.errorCodeLookup = {}

M.isConnected = false

local canMessageCallbacks = {}

local function combineTwoBytes(b1, b2)
  return bor(lshift(b1, 8), b2)
end

local function lowByte(x)
  return band(x, 0xFF)
end

local function highByte(x)
  return rshift(x, 8)
end

local function twoBytes(x)
  return lowByte(x), highByte(x)
end

local function fourBytes(x)
  local lowerTwoBytes = band(x, 0xFFFF)
  local upperTwoBytes = rshift(x, 16)
  return lowByte(lowerTwoBytes), highByte(lowerTwoBytes), lowByte(upperTwoBytes), highByte(upperTwoBytes)
end

local function toBits(num, bits)
  -- returns a table of bits, most significant first.
  bits = bits or max(1, select(2, frexp(num)))
  local t = {} -- will contain the bits
  for b = bits, 1, -1 do
    t[b] = fmod(num, 2)
    num = floor((num - t[b]) / 2)
  end
  return t
end

local function dumpsByte(b)
  return "0b" .. table.concat(toBits(b, 8))
end

local function dumpsByteHex(b)
  return string.format("0x%x", b)
end

local function dumpsMsg(msg)
  local data = {}
  for i = 0, msg.LEN - 1 do
    data[i + 1] = msg.DATA[i]
  end
  return string.format("0x%x -> %s", msg.ID, dumps(data))
end

local function updateGFX(dt)
  if M.isConnected and next(canMessageCallbacks) ~= nil then
    for i = 1, 100 do
      local receiveResult, msg = M.receiveCANBus()
      if receiveResult == M.errorCodes.OK then
        for _, callback in pairs(canMessageCallbacks) do
          callback(msg)
        end
      else
        break
      end
    end
  end
end

local function sendCANMessage(messageId, data, tag)
  local sendResult = M.sendCANBusRaw(messageId, data)
  if sendResult ~= M.errorCodes.OK then
    log("E", "CANBusPeak.sendCANMessage", string.format("Non-OK send result for %q: %q", tag, M.errorCodeLookup[sendResult]))
  end
end

local sendMsg = ffi.new("TPCANMsg")
local function sendCANBusRaw(id, data, msgType, channel)
  channel = channel or M.PCANHandles.USBBus1
  msgType = msgType or M.messageType.Standard

  sendMsg.ID = id
  sendMsg.MSGTYPE = msgType

  for index, byte in ipairs(data) do
    sendMsg.DATA[index - 1] = byte --start data array at [0]
  end

  sendMsg.LEN = #data
  local result = M.canBusAPI.CAN_Write(channel, sendMsg)
  if result ~= M.errorCodes.OK then
    log("W", "CANBusPeak.receiveCANBus", "Non-OK receive status: " .. M.errorCodeLookup[result])
  end
  return result
end

local rcvTimestamp = ffi.new("TPCANTimestamp")
local function receiveCANBus(channel)
  channel = channel or M.PCANHandles.USBBus1
  local msg = ffi.new("TPCANMsg")
  local result = M.canBusAPI.CAN_Read(channel, msg, rcvTimestamp)
  if result ~= M.errorCodes.OK and result ~= M.errorCodes.QRCVEmpty then
    log("W", "CANBusPeak.receiveCANBus", "Non-OK receive status: " .. (M.errorCodeLookup[result] or result))
  end
  return result, msg
end

local function registerCANMessageCallback(id, callback)
  canMessageCallbacks[id] = callback
end

local function unregisterCANMessageCallback(id)
  canMessageCallbacks[id] = nil
end

local function initCANBus(channel, baudrate, hwType, ioPort, interrupt)
  M.isConnected = false
  channel = channel or M.PCANHandles.USBBus1
  baudrate = baudrate or M.baudrates.Baud500K
  hwType = hwType or 0
  ioPort = ioPort or 0
  interrupt = interrupt or 0
  local status = M.canBusAPI.CAN_Initialize(channel, baudrate, hwType, ioPort, interrupt)
  log("I", "CANBusPeak.initCANBus", "CANBus initialization result: " .. M.errorCodeLookup[status])
  if status == M.errorCodes.OK then
    M.isConnected = true
  end
  return status
end

local function onExtensionLoaded()
  log("I", "CANBusPeak.onExtensionLoaded", "PEAK System CANBus extension loaded")
  M.errorCodeLookup = {}
  for key, value in pairs(M.errorCodes) do
    M.errorCodeLookup[value] = key
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

M.initCANBus = initCANBus
M.sendCANBusRaw = sendCANBusRaw
M.sendCANMessage = sendCANMessage
M.receiveCANBus = receiveCANBus

M.registerCANMessageCallback = registerCANMessageCallback
M.unregisterCANMessageCallback = unregisterCANMessageCallback

M.lowByte = lowByte
M.highByte = highByte
M.twoBytes = twoBytes
M.fourBytes = fourBytes
M.combineTwoBytes = combineTwoBytes
M.dumpsMsg = dumpsMsg
M.dumpsByte = dumpsByte
M.dumpsByteHex = dumpsByteHex

return M
