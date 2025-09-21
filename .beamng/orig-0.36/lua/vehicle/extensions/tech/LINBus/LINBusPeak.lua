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
M.linBusAPI = ffi.load("PLinApi.dll")
ffi.cdef [[
//  PLinApi.h
//
//  Definition of the PLIN-API.
//
//
//    Principle:
//    ~~~~~~~~~~
//    The driver supports multiple clients (= Windows or DOS programs
//    that communicate with LIN-busses), and multiple LIN-Devices.
//    A LIN-Device represents two LIN-Hardware (each channel is a Hardware)
//    Multiple 'Clients' can be connected to one or more LIN-Hardware, which
//    itself have an interface to a physical LIN-channel of a device.
//
//    Features:
//    ~~~~~~~~~
//     - 1 Client can be connected to multiple Hardware
//     - 1 Hardware supports multiple clients
//     - When a Client sends a message to a Hardware, the message will not be routed
//       to other clients. The response of the Hardware is routed to the connected
//       clients depending on the registered Hardware message filter.
//     - each Client only receives the messages that pass its acceptance filter
//     - each Client has a Receive Queue to buffer received messages
//     - hClient: 'Client handle'. This number is used by the driver to
//                identify and manage a Client
//     - hHw:     'Hardware handle'. This number is used by the driver to
//                identify and manage a Hardware
//     - all handles are 1-based. 0 = illegal handle
//
//     All functions return a value of type TLINError
//
//     Authors: K.Wagner / P.Steil / M.Riedl
//
//     -----------------------------------------------------------------------
//     Copyright (C) 2008-2022 by PEAK-System Technik GmbH, Darmstadt/Germany
//     -----------------------------------------------------------------------
//
//     Last Change: 2021-12-14
///////////////////////////////////////////////////////////////////////////////

// Handle types
//
typedef unsigned char HLINCLIENT;                        // Client Handle
typedef unsigned short HLINHW;                            // Hardware Handle
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef const char *LPSTR;
typedef int TLINMsgErrors;
typedef WORD TLINHardwareParam;
typedef WORD TLINClientParam;
typedef BYTE TLINDirection;
typedef BYTE TLINChecksumType;
typedef BYTE TLINHardwareMode;
typedef BYTE TLINHardwareState;
typedef BYTE TLINMsgType;
typedef BYTE TLINSlotType;
typedef BYTE TLINScheduleState;
typedef DWORD TLINError;



#pragma pack(push, 8)                           // These Records are 8-Bytes aligned!

// Version Information structure
//
typedef struct {                                // Size = 8 bytes
    short Major;                                // #0 +0  Major part of a version number
    short Minor;                                // #1 +2  Minor part of a version number
    short Revision;                             // #2 +4  Revision part of a version number
    short Build;                                // #3 +6  Build part of a version number
}TLINVersion;

// A LIN Message to be sent
//
typedef  struct {                               // Size = 13 bytes
    BYTE FrameId;                               // #0 +0  Frame ID (6 bit) + Parity (2 bit)
    BYTE Length;                                // #1 +1  Frame Length (1..8)
    TLINDirection Direction;                    // #2 +2  Frame Direction (see Message Direction Types)
    TLINChecksumType ChecksumType;              // #3 +3  Frame Checksum type (see Message Checksum Types)
    BYTE Data[8];                               // #4 +4  Data bytes (0..7)
    BYTE Checksum;                              // #5 +12 Frame Checksum
} TLINMsg;

// A received LIN Message
//
typedef  struct {                               // Size = 40 bytes
    TLINMsgType Type;                           // #0 +0  Frame type (see Received Message Types)
    BYTE FrameId;                               // #1 +1  Frame ID (6 bit) + Parity (2 bit)
    BYTE Length;                                // #2 +2  Frame Length (1..8)
    TLINDirection Direction;                    // #3 +3  Frame Direction (see Message Direction Types)
    TLINChecksumType ChecksumType;              // #4 +4  Frame Checksum type (see Message Checksum Types)
    BYTE Data[8];                               // #5 +5  Data bytes (0..7)
    BYTE Checksum;                              // #6 +13 Frame Checksum
    TLINMsgErrors ErrorFlags;                   // #7 +16 Frame error flags (see Error flags for LIN Rcv Msgs)
    unsigned __int64 TimeStamp;                 // #8 +24 Timestamp in microseconds
    HLINHW hHw;                                 // #9 +32 Handle of the Hardware which received the message
} TLINRcvMsg;

// A LIN Frame Entry
//
typedef struct {                                // Size = 14 bytes
    BYTE FrameId;                               // #0 +0  Frame ID (without parity)
    BYTE Length;                                // #1 +1  Frame Length (1..8)
    TLINDirection Direction;                    // #2 +2  Frame Direction (see Message Direction Types)
    TLINChecksumType ChecksumType;              // #3 +3  Frame Checksum type (see Message Checksum Types)
    WORD Flags;                                 // #4 +4  Frame flags (see Frame flags for LIN Msgs)
    BYTE InitialData[8];                        // #5 +6  Data bytes (0..7)
} TLINFrameEntry;

// A LIN Schedule slot
//
typedef struct {                                // Size = 20 bytes
    TLINSlotType Type;                          // #0 +0  Slot Type (see Schedule Slot Types)
    WORD Delay;                                 // #1 +2  Slot Delay in Milliseconds
    BYTE FrameId[8];                            // #2 +4  Frame IDs (without parity)
    BYTE CountResolve;                          // #3 +12 ID count for sporadic frames
                                                //        Resolve schedule number for Event frames
    DWORD Handle;                               // #4 +16 Slot handle (read-only)
} TLINScheduleSlot;

// LIN Status data
//
typedef struct {                                // Size = 8 bytes
    TLINHardwareMode Mode;                      // #0 +0  Node state (see Hardware Operation Modes)
    TLINHardwareState Status;                   // #1 +1  Bus state (see Hardware Status)
    BYTE FreeOnSendQueue;                       // #2 +2  Count of free places in the Transmit Queue
    WORD FreeOnSchedulePool;                    // #3 +4  Free slots in the Schedule pool (see Minimum and Maximum values)
    WORD ReceiveBufferOverrun;                  // #4 +6  USB receive buffer overrun counter
} TLINHardwareStatus;

#pragma pack(pop)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes

//-----------------------------------------------------------------------------
// LIN_RegisterClient()
//   Registers a Client at the LIN Manager. Creates a Client handle and
//   allocates the Receive Queue (only one per Client). The hWnd parameter
//   can be zero for DOS Box Clients. The Client does not receive any
//   messages until LIN_RegisterFrameId() or LIN_SetClientFilter() is called.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errOutOfResource
//
TLINError __stdcall LIN_RegisterClient (
    LPSTR strName,              // Name of the Client
    DWORD hWnd,                 // Window handle of the Client (only for information purposes)
    HLINCLIENT *hClient);       // Pointer to the Client handle buffer


//-----------------------------------------------------------------------------
// LIN_RemoveClient()
//   Removes a Client from the Client list of the LIN Manager. Frees all
//   resources (receive queues, message counters, etc.). If the Client was
//   a Boss-Client for one or more Hardware, the Boss-Client property for
//   those Hardware will be set to INVALID_LIN_HANDLE.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient
//
TLINError __stdcall LIN_RemoveClient (
    HLINCLIENT hClient);        // Handle of the Client


//-----------------------------------------------------------------------------
// LIN_ConnectClient()
//   Connects a Client to a Hardware.
//   The Hardware is assigned by its Handle.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_ConnectClient (
    HLINCLIENT hClient,         // Connect this Client ...
    HLINHW hHw);                // to this Hardware


//-----------------------------------------------------------------------------
// LIN_DisconnectClient()
//   Disconnects a Client from a Hardware. This means: no more messages
//   will be received by this Client from this Hardware.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_DisconnectClient (
    HLINCLIENT hClient,         // Disconnect this Client ...
    HLINHW hHw);                // from this Hardware.


//-----------------------------------------------------------------------------
// LIN_ResetClient()
//   Flushes the Receive Queue of the Client and resets its counters.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient
//
TLINError __stdcall LIN_ResetClient (
    HLINCLIENT hClient);        // Handle of the Client


//-----------------------------------------------------------------------------
// LIN_SetClientParam()
//   Sets a Client parameter to a given value.
//
//   Allowed TLINClientParam    Parameter
//   values in wParam:          type:       Description:
//   -------------------------  ----------  ------------------------------------
//   clpReceiveStatusFrames     int         0 = Status Frames deactivated,
//                                          otherwise active
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterType, errWrongParameterValue, errIllegalClient
//
TLINError __stdcall LIN_SetClientParam (
    HLINCLIENT hClient,         // Client Handle
    TLINClientParam wParam,     // TLINClientParam parameter
    DWORD dwValue);             // Parameter value


//-----------------------------------------------------------------------------
// LIN_GetClientParam()
//   Gets a Client parameter.
//
//   Allowed TLINClientParam    Parameter
//   values in wParam:          type:       Description:
//   -------------------------  ----------  ------------------------------------
//   clpName                    char[]      Name of the Client
//   clpMessagesOnQueue         int         Unread messages in the Receive Queue
//   clpWindowHandle            int         Window handle of the Client application
//                                          (can be zero for DOS Box Clients)
//   clpConnectedHardware       HLINHW[]    Array of Hardware Handles connected by a Client
//                                          The first item in the array refers to the
//                                          amount of handles. So [*] = Total handles + 1
//   clpTransmittedMessages     int         Number of transmitted messages
//   clpReceivedMessages        int         Number of received messages
//   clpReceiveStatusFrames     int         0 = Status Frames deactivated, otherwise active
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterType, errWrongParameterValue, errIllegalClient,
//   errBufferInsufficient
//
TLINError __stdcall LIN_GetClientParam (
    HLINCLIENT hClient,         // Client Handle
    TLINClientParam wParam,     // TLINClientParam parameter
    void *pBuff,                // Buffer for the parameter value
    WORD wBuffSize);            // Size of the buffer in bytes


//-----------------------------------------------------------------------------
// LIN_SetClientFilter()
//   Sets the filter of a Client and modifies the filter of
//   the connected Hardware.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_SetClientFilter (
    HLINCLIENT hClient,         // Set for this Client
    HLINHW hHw,                 // within this Hardware
    unsigned __int64 iRcvMask); // this message filter: each bit corresponds
                                // to a Frame ID (0..63).


//-----------------------------------------------------------------------------
// LIN_GetClientFilter()
//   Gets the filter corresponding to a given Client-Hardware pair.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_GetClientFilter (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    unsigned __int64 *pRcvMask);// Buffer for the message filter: each bit
                                // corresponds to a Frame ID (0..63)


//-----------------------------------------------------------------------------
// LIN_Read()
//   Reads the next message/status information from a Client's Receive
//   Queue. The message will be written to 'pMsg'.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errRcvQueueEmpty
//
TLINError __stdcall LIN_Read (
    HLINCLIENT hClient,         // Client Handle
    TLINRcvMsg *pMsg);          // Buffer for the message


//-----------------------------------------------------------------------------
// LIN_ReadMulti()
//   Reads several received messages.
//   pMsgBuff must be an array of 'iMaxCount' entries (must have at least
//   a size of iMaxCount * sizeof(TLINRcvMsg) bytes).
//   The size 'iMaxCount' of the array = max. messages that can be received.
//   The real number of read messages will be returned in 'pCount'.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errRcvQueueEmpty
//
TLINError __stdcall LIN_ReadMulti (
    HLINCLIENT hClient,         // Client Handle
    TLINRcvMsg *pMsgBuff,       // Buffer for the messages
    int iMaxCount,              // Maximum number of messages to read
    int *pCount);               // Buffer for the real number of messages read


//-----------------------------------------------------------------------------
// LIN_Write()
//   The Client 'hClient' transmits a message 'pMsg' to the Hardware 'hHw'.
//   The message is written into the Transmit Queue of the Hardware.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalDirection, errIllegalLength
//
TLINError __stdcall LIN_Write (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    TLINMsg *pMsg);             // Message Buffer to be written


//-----------------------------------------------------------------------------
// LIN_InitializeHardware()
//   Initializes a Hardware with a given Mode and Baudrate.
//   REMARK: If the Hardware was initialized by another Client, the function
//   will re-initialize the Hardware. All connected clients will be affected.
//   It is the job of the user to manage the setting and/or configuration of
//   Hardware, e.g. by using the Boss-Client parameter of the Hardware.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalBaudrate
//
TLINError __stdcall LIN_InitializeHardware (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    TLINHardwareMode bMode,     // Hardware Mode (see Hardware Operation Modes)
    WORD wBaudrate);            // LIN Baudrate (see LIN_MIN_BAUDRATE and LIN_MAX_BAUDRATE)


//-----------------------------------------------------------------------------
// LIN_GetAvailableHardware()
//   Gets an array containing the handles of the current Hardware
//   available in the system.
//   The count of Hardware handles returned in the array is written in
//   'pCount'.
//
//   REMARK: To ONLY get the count of available Hardware, call this
//   function using 'pBuff' = NULL and wBuffSize = 0.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errBufferInsufficient
//
TLINError __stdcall LIN_GetAvailableHardware(
    HLINHW *pBuff,              // Buffer for the handles
    WORD wBuffSize,             // Size of the buffer in bytes
    int *pCount);               // Number of Hardware available


//-----------------------------------------------------------------------------
// LIN_SetHardwareParam()
//   Sets a Hardware parameter to a given value.
//
//   Allowed TLINHardwareParam  Parameter
//   values in wParam:          type:       Description:
//   -------------------------  ----------  -----------------------------------
//   hwpMessageFilter           unsigned    Hardware message filter. Each bit
//                              _int64      corresponds to a Frame ID (0..63)
//   hwpBossClient              HLINCLIENT  Handle of the new Boss-Client
//   hwpIdNumber                int         Identification number for a hardware
//   hwpUserData                BYTE[]      User data to write on a hardware. See LIN_MAX_USER_DATA
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterType, errWrongParameterValue, errIllegalClient,
//   errIllegalHardware
//
TLINError __stdcall LIN_SetHardwareParam (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    TLINHardwareParam wParam,   // TLINHardwareParam parameter
    void *pBuff,                // Buffer for the parameter value
    WORD wBuffSize);            // Size of the buffer


//-----------------------------------------------------------------------------
// LIN_GetHardwareParam()
//   Gets a Hardware parameter.
//
//   Allowed TLINHardwareParam  Parameter
//   values in wParam:          type:       Description:
//   -------------------------  ----------  -----------------------------------
//   hwpName                    char[]      Name of the Hardware. See LIN_MAX_NAME_LENGTH
//   hwpDeviceNumber            int         Index of the Device owner of the Hardware
//   hwpChannelNumber           int         Channel Index of the Hardware on the owner device
//   hwpConnectedClients        BYTE[*]     Array of Client Handles conencted to a Hardware
//                                          The first item in the array refers to the
//                                          amount of handles. So [*] = Total handles + 1
//   hwpMessageFilter           unsigned    Configured message filter. Each bit corresponds
//                               _int64     to a Frame ID (0..63)
//   hwpBaudrate                int         Configured baudrate
//   hwpMode                    int         0 = Slave, otehrwise Master
//   hwpFirmwareVersion         TLINVersion A TLINVersion structure containing the Firmware Version
//   hwpBufferOverrunCount      int         Receive Buffer Overrun Counter
//   hwpBossClient              HLINCLIENT  Handle of the current Boss-Client
//   hwpSerialNumber            int         Serial number of the Hardware
//   hwpVersion                 int         Version of the Hardware
//   hwpType                    int         Type of the Hardware
//   hwpQueueOverrunCount       int         Receive Queue Buffer Overrun Counter
//   hwpIdNumber                int         Identification number for a hardware
//   hwpUserData                BYTE[]      User data saved on the hardware. See LIN_MAX_USER_DATA
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterType, errWrongParameterValue, errIllegalHardware,
//   errBufferInsufficient
//
TLINError __stdcall LIN_GetHardwareParam (
    HLINHW hHw,                 // Hardware Handle
    TLINHardwareParam wParam,   // TLINHardwareParam parameter
    void *pBuff,                // Buffer for the parameter value
    WORD wBuffSize);            // Size of the buffer


//-----------------------------------------------------------------------------
// LIN_ResetHardware()
//   Flushes the queues of the Hardware and resets its counters.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_ResetHardware (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw);                // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_ResetHardwareConfig()
//   Deletes the current configuration of the Hardware and sets its defaults.
//   The Client 'hClient' must be registered and connected to the Hardware to
//   be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_ResetHardwareConfig (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw);                // Hardware handle

//-----------------------------------------------------------------------------
// LIN_IdentifyHardware()
//   Phisically identifies a LIN Hardware (a channel on a LIN Device) by
//   blinking its associated LED.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware
//
TLINError __stdcall LIN_IdentifyHardware (
    HLINHW hHw);                // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_RegisterFrameId()
//   Modifies the filter of a Client and, eventually, the filter of the
//   connected Hardware. The messages with FrameID 'bFromFrameId' to
//   'bToFrameId' will be received.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalFrameID
//
TLINError __stdcall LIN_RegisterFrameId (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    BYTE bFromFrameId,          // First ID of the frame range
    BYTE bToFrameId);           // Last ID of the frame range


//-----------------------------------------------------------------------------
// LIN_SetFrameEntry()
//   Configures a LIN Frame in a given Hardware. The Client 'hClient' must
//   be registered and connected to the Hardware to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalFrameID, errIllegalLength
//
TLINError __stdcall LIN_SetFrameEntry (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    TLINFrameEntry *pFrameEntry);   // Frame entry buffer


//-----------------------------------------------------------------------------
// LIN_GetFrameEntry()
//   Gets the configuration of a LIN Frame from a given Hardware.
//   The 'pFrameEntry.FrameId' must be set to the ID of the frame, whose
//   configuration should be returned.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware, errIllegalFrameID
//
TLINError __stdcall LIN_GetFrameEntry (
    HLINHW hHw,                     // Hardware Handle
    TLINFrameEntry *pFrameEntry);   // Frame Entry buffer


//-----------------------------------------------------------------------------
// LIN_UpdateByteArray()
//   Updates the data of a LIN Frame for a given Hardware. The Client
//   'hClient' must be registered and connected to the Hardware to be
//   accessed. 'pData' must have at least a size of 'bLen'.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalFrameID, errIllegalLength, errIllegalIndex,
//   errIllegalRange
//
TLINError __stdcall LIN_UpdateByteArray (
    HLINCLIENT hClient,         // Client handle
    HLINHW hHw,                 // Hardware Handle
    BYTE bFrameId,              // Frame ID
    BYTE bIndex,                // Index where the update data Starts (0..7)
    BYTE bLen,                  // Count of Data bytes to be updated.
    BYTE *pData);               // Data buffer


//-----------------------------------------------------------------------------
// LIN_StartKeepAlive()
//   Sets the Frame 'bFrameId' as Keep-Alive frame for the given Hardware and
//   starts to send it every 'wPeriod' Milliseconds. The Client 'hClient' must
//   be registered and connected to the Hardware to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalFrameID, errIllegalSchedulerState, errIllegalFrameConfiguration
//
TLINError __stdcall LIN_StartKeepAlive (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw,                 // Hardware Handle
    BYTE bFrameId,              // ID of the Keep-Alive Frame
    WORD wPeriod);              // Keep-Alive Interval in Milliseconds


//-----------------------------------------------------------------------------
// LIN_SuspendKeepAlive()
//   Suspends the sending of a Keep-Alive frame in the given Hardware.
//   The Client 'hClient' must be registered and connected to the Hardware
//   to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_SuspendKeepAlive (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw);                // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_ResumeKeepAlive()
//   Resumes the sending of a KeepAlive frame in the given Hardware.
//   The Client 'hClient' must be registered and connected to the Hardware
//   to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalSchedulerState, errIllegalFrameConfiguration
//
TLINError __stdcall LIN_ResumeKeepAlive (
    HLINCLIENT hClient,         // Client Handle
    HLINHW hHw);                // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_SetSchedule()
//   Configures the slots of a Schedule in a given Hardware. The Client
//   'hClient' must be registered and connected to the Hardware to be
//   accessed. The Slot handles will be returned in the parameter
//   "pSchedule" (Slots buffer), when this function successfully completes.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalScheduleNo, errIllegalSlotCount, errScheduleSlotPoolFull
//
TLINError __stdcall LIN_SetSchedule (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    int iScheduleNumber,            // Schedule number (see LIN_MIN_SCHEDULE_NUMBER
                                    // and LIN_MAX_SCHEDULE_NUMBER)
    TLINScheduleSlot *pSchedule,    // Slots buffer
    int iSlotCount);                // Count of Slots in the slots buffer


//-----------------------------------------------------------------------------
// LIN_GetSchedule()
//   Gets the slots of a Schedule from a given Hardware. The count of slots
//   returned in the array is written in 'pSlotCount'.
//
//   REMARK: To ONLY get the count of slots contained in the given Schedule,
//   call this function using 'pScheduleBuff' = NULL and iMaxSlotCount = 0.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware, errIllegalScheduleNo,
//   errIllegalSlotCount, errIllegalSchedule
//
TLINError __stdcall LIN_GetSchedule (
    HLINHW  hHw,                        // Hardware Handle
    int iScheduleNumber,                // Schedule Number (see LIN_MIN_SCHEDULE_NUMBER
                                        // and LIN_MAX_SCHEDULE_NUMBER)
    TLINScheduleSlot *pScheduleBuff,    // Slots Buffer.
    int iMaxSlotCount,                  // Maximum number of slots to read.
    int *pSlotCount);                   // Real number of slots read.


//-----------------------------------------------------------------------------
// LIN_DeleteSchedule()
//   Removes all slots contained by a Schedule of a given Hardware. The
//   Client 'hClient' must be registered and connected to the Hardware to
//   be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalScheduleNo, errIllegalSchedulerState
//
TLINError __stdcall LIN_DeleteSchedule (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    int iScheduleNumber);           // Schedule Number (see LIN_MIN_SCHEDULE_NUMBER
                                    // and LIN_MAX_SCHEDULE_NUMBER)


//-----------------------------------------------------------------------------
// LIN_SetScheduleBreakPoint()
//   Sets a 'breakpoint' on a slot from a Schedule in a given Hardware. The
//   Client 'hClient' must be registered and connected to the Hardware to
//   be accessed.
//
//  REMARK: Giving 'dwHandle' a value of 0 ('NULL'), causes the deletion of
//  the breakpoint.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_SetScheduleBreakPoint (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    int iBreakPointNumber,          // Breakpoint Number (0 or 1)
    DWORD dwHandle);                // Slot Handle


//-----------------------------------------------------------------------------
// LIN_StartSchedule()
//   Activates a Schedule in a given Hardware. The Client 'hClient' must
//   be registered and connected to the Hardware to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalScheduleNo, errIllegalHardwareMode, errIllegalSchedule
//
TLINError __stdcall LIN_StartSchedule (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    int iScheduleNumber);           // Schedule Number (see LIN_MIN_SCHEDULE_NUMBER
                                    // and LIN_MAX_SCHEDULE_NUMBER)


//-----------------------------------------------------------------------------
// LIN_SuspendSchedule()
//   Suspends an active Schedule in a given Hardware. The Client 'hClient'
//   must be registered and connected to the Hardware to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_SuspendSchedule (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw);                    // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_ResumeSchedule()
//   Restarts a configured Schedule in a given Hardware. The Client 'hClient'
//   must be registered and connected to the Hardware to be accessed.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalSchedule, errIllegalHardwareMode, errIllegalSchedulerState
//
TLINError __stdcall LIN_ResumeSchedule (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw);                    // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_XmtWakeUp()
//   Sends a wake-up impulse (single data byte 0xF0). The Client 'hClient'
//   must be registered and connected to the Hardware to be accessed.
//
// Remark: Only in Slave-mode. After sending a wake-up impulse a time
//         of 150 milliseconds is used as timeout.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_XmtWakeUp (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw);                    // Hardware Handle


//-----------------------------------------------------------------------------
// LIN_XmtDynamicWakeUp()
//   Sends a wake-up impulse (single data byte 0xF0) and specify a custom
//   bus-sleep timeout, in milliseconds. The Client 'hClient'
//   must be registered and connected to the Hardware to be accessed.
//
// Remark: Only in Slave-mode. The bus-sleep timeout is set to its default,
//         150 milliseconds, after the custom timeout is exhausted.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponse, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware
//
TLINError __stdcall LIN_XmtDynamicWakeUp (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    WORD wTimeOut);                 // Bus-sleep timeout


//-----------------------------------------------------------------------------
// LIN_StartAutoBaud()
//   Starts a process to detect the Baud rate of the LIN bus that is
//   connected to the indicated Hardware.
//   The Client 'hClient' must be registered and connected to the Hardware
//   to be accessed. The Hardware must be not initialized in order
//   to do an Auto-baudrate procedure.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalClient, errIllegalHardware,
//   errIllegalHardwareState
//
TLINError __stdcall LIN_StartAutoBaud (
    HLINCLIENT hClient,             // Client Handle
    HLINHW hHw,                     // Hardware Handle
    WORD wTimeOut);                 // Auto-baudrate Timeout in Milliseconds


//-----------------------------------------------------------------------------
// LIN_GetStatus()
//   Retrieves current status information from the given Hardware.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware
//
TLINError __stdcall LIN_GetStatus (
    HLINHW hHw,                         // Hardware handle
    TLINHardwareStatus *pStatusBuff);   // Status data buffer


//-----------------------------------------------------------------------------
// LIN_CalculateChecksum()
//   Calculates the checksum of a LIN Message and writes it into the
//   'Checksum' field of 'pMsg'.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalLength
//
TLINError __stdcall LIN_CalculateChecksum (
    TLINMsg *pMsg);                     // Message buffer


//-----------------------------------------------------------------------------
// LIN_GetVersion()
//   Returns a TLINVersion structure containing the PLIN-API DLL version.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue
//
TLINError __stdcall LIN_GetVersion (
    TLINVersion *pVerBuff               // Version buffer
);

//-----------------------------------------------------------------------------
// LIN_GetVersionInfo()
//   Returns a string containing Copyright information.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue
//
TLINError __stdcall LIN_GetVersionInfo (
    LPSTR  strTextBuff,                 // String buffer
    WORD wBuffSize);                    // Size in bytes of the buffer


//-----------------------------------------------------------------------------
// LIN_GetErrorText()
//   Converts the error code 'dwError' to a text containing an error
//   description in the language given as parameter (when available).
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errBufferInsufficient
//
TLINError __stdcall LIN_GetErrorText (
    TLINError dwError,                  // A TLINError code
    BYTE bLanguage,                     // Indicates a "Primary language ID"
    LPSTR strTextBuff,                  // Error string buffer
    WORD wBuffSize);                    // Buffer size in bytes


//-----------------------------------------------------------------------------
// LIN_GetPID()
//   Gets the 'FrameId with Parity' corresponding to the given
//   'pFrameId' and writes the result on it.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalFrameID
//
TLINError __stdcall LIN_GetPID(
    BYTE  *pFrameId);                   // Frame ID (0..LIN_MAX_FRAME_ID)


//-----------------------------------------------------------------------------
// LIN_GetTargetTime()
//   Gets the system time used by the LIN-USB adapter.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware
//
TLINError __stdcall  LIN_GetTargetTime (
    HLINHW hHw,                         // Hardware Handle
    unsigned __int64  *pTargetTime);    // TargetTime buffer


//-----------------------------------------------------------------------------
// LIN_SetResponseRemap()
//   Sets the Response Remap of a LIN Slave
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalFrameID, errIllegalClient, errIllegalHardware,
//   errMemoryAccess
//
TLINError __stdcall LIN_SetResponseRemap (
    HLINCLIENT  hClient,                // Client Handle
    HLINHW hHw,                         // Hardware Handle
    BYTE *pRemapTab);                   // Remap Response buffer


//-----------------------------------------------------------------------------
// LIN_GetResponseRemap()
//   Gets the Response Remap of a LIN Slave
//
//  REMARK: The Buffer must be at least 64 bytes length
//  the breakpoint.
//
// Possible DLL interaction errors:
//   errManagerNotLoaded, errManagerNotResponding, errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue, errIllegalHardware, errMemoryAccess
//
TLINError __stdcall LIN_GetResponseRemap (
    HLINHW hHw,                         // Hardware Handle
    BYTE *pRemapTab);                   // Remap Response buffer


//-----------------------------------------------------------------------------
// LIN_GetSystemTime()
//   Gets the current system time. The system time is returned by
//   Windows as the elapsed number of microseconds since system start.
//
// Possible DLL interaction errors:
//   errMemoryAccess
//
// Possible API errors:
//   errWrongParameterValue
//
TLINError __stdcall LIN_GetSystemTime(
    unsigned __int64  *pSystemTime);

]]

M.hardwareTypes = {
  LIN_HW_TYPE_USB = 1, -- LIN USB type // DEPRECATED
  LIN_HW_TYPE_USB_PRO = 1, -- PCAN-USB Pro LIN type
  LIN_HW_TYPE_USB_PRO_FD = 2, -- PCAN-USB Pro FD LIN type
  LIN_HW_TYPE_PLIN_USB = 3
}

M.modes = {
  modNone = 0, -- Hardware is not initialized
  modSlave = 1, -- Hardware working as Slave
  modMaster = 2 --Hardware working as Master
}

M.directions = {
  disabled = 0,
  publisher = 1,
  subscriber = 2,
  subscriberAutoLength = 3
}

M.checksumTypes = {
  custom = 0,
  classic = 1,
  enhanced = 2,
  auto = 3
}

M.errorCodes = {
  OK = 0,
  XmtQueueFull = 1,
  IllegalPeriod = 2,
  RcvQueueEmpty = 3,
  IllegalChecksumType = 4,
  errIllegalHardware = 5,
  IllegalClient = 6,
  WrongParameterType = 7,
  WrongParameterValue = 8,
  IllegalDirection = 9,
  IllegalLength = 10,
  IllegalBaudrate = 11,
  IllegalFrameID = 12,
  BufferInsufficient = 13,
  IllegalScheduleNo = 14,
  IllegalSlotCount = 15,
  IllegalIndex = 16,
  IllegalRange = 17,
  IllegalHardwareState = 18,
  IllegalSchedulerState = 19,
  IllegalFrameConfiguration = 20,
  ScheduleSlotPoolFull = 21,
  IllegalSchedule = 22,
  IllegalHardwareMode = 23,
  OutOfResource = 1001,
  ManagerNotLoaded = 1002,
  ManagerNotResponding = 1003,
  MemoryAccess = 1004,
  NotImplemented = 0xFFFE,
  Unknown = 0xFFFF
}

M.client = 0
M.hardware = 0
M.errorCodeLookup = {}

M.isConnected = false

local linMessageCallbacks = {}

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
  for i = 0, msg.Length - 1 do
    data[i + 1] = msg.Data[i]
  end
  return string.format("0x%x -> %s", msg.FrameId, dumps(data))
end

local function updateGFX(dt)
  if M.isConnected then
    local msgBuffer, msgCount = M.receiveLINBus()
    if msgCount > 0 then
      for i = 0, msgCount - 1 do
        local msg = msgBuffer[i]
        for _, callback in pairs(linMessageCallbacks) do
          callback(msg)
        end
      end
    end
  end
end

local function sendLINMessage(messageId, data, tag)
  local sendResult = M.sendLINBusRaw(messageId, data)
  if sendResult ~= M.errorCodes.OK then
    log("E", "LINBusPeak.sendLINMessage", string.format("Non-OK send result for %q: %q", tag, M.errorCodeLookup[sendResult]))
  end
end

local function sendLINBusRaw(frameId, direction, checksumType, length, data)
  local pid = ffi.new("BYTE[1]", frameId)
  local result = M.linBusAPI.LIN_GetPID(pid)
  pid = pid[0]

  local msg = ffi.new("TLINMsg")
  msg.FrameId = pid
  msg.Direction = direction
  msg.ChecksumType = checksumType
  msg.Length = length
  if data then
    for index, byte in ipairs(data) do
      msg.Data[index - 1] = byte --start data array at [0]
    end
  end

  M.linBusAPI.LIN_Write(M.client, M.hardware, msg)
end

local function requestLINData(frameId)
  return sendLINBusRaw(frameId, M.directions.subscriberAutoLength, M.checksumTypes.enhanced, 1)
end

local function receiveLINBus()
  local buffer = ffi.new("TLINRcvMsg[10]")
  local count = ffi.new("int[1]")
  local result = M.linBusAPI.LIN_ReadMulti(M.client, buffer, 10, count)
  count = count[0]

  return buffer, count
end

local function registerLINMessageCallback(id, callback)
  linMessageCallbacks[id] = callback
end

local function unregisterLINMessageCallback(id)
  linMessageCallbacks[id] = nil
end

local function initLINBus(hardware, baudrate, mode)
  M.isConnected = false
  M.hardware = hardware or M.hardwareTypes.LIN_HW_TYPE_USB_PRO
  baudrate = baudrate or 19200
  mode = mode or M.modes.modMaster

  M.client = ffi.new("HLINCLIENT[1]")
  local result = M.linBusAPI.LIN_RegisterClient("bng_lin", 0, M.client)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_RegisterClient failed with result: " .. M.errorCodeLookup[result])
    return result
  end
  M.client = M.client[0]

  result = M.linBusAPI.LIN_ConnectClient(M.client, M.hardware)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_ConnectClient failed with result: " .. M.errorCodeLookup[result])
    return result
  end

  result = M.linBusAPI.LIN_InitializeHardware(M.client, M.hardware, mode, baudrate)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_InitializeHardware failed with result: " .. M.errorCodeLookup[result])
    return result
  end

  result = M.linBusAPI.LIN_ResetClient(M.client)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_ResetClient failed with result: " .. M.errorCodeLookup[result])
    return result
  end

  result = M.linBusAPI.LIN_ResetHardware(M.client, M.hardware)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_ResetHardware failed with result: " .. M.errorCodeLookup[result])
    return result
  end

  result = M.linBusAPI.LIN_RegisterFrameId(M.client, M.hardware, 0, 63)
  if result ~= M.errorCodes.OK then
    log("E", "LINBusPeak.initLINBus", "LIN_RegisterFrameId failed with result: " .. M.errorCodeLookup[result])
    return result
  end

  if result == M.errorCodes.OK then
    M.isConnected = true
    log("I", "LINBusPeak.initLINBus", "LINBus initialized, client id: " .. M.client)
  end
  return result
end

local function onExtensionLoaded()
  log("I", "LINBusPeak.onExtensionLoaded", "PEAK System LINBus extension loaded")
  M.errorCodeLookup = {}
  for key, value in pairs(M.errorCodes) do
    M.errorCodeLookup[value] = key
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

M.initLINBus = initLINBus
M.sendLINBusRaw = sendLINBusRaw
M.sendLINMessage = sendLINMessage
M.requestLINData = requestLINData
M.receiveLINBus = receiveLINBus

M.registerLINMessageCallback = registerLINMessageCallback
M.unregisterLINMessageCallback = unregisterLINMessageCallback

M.lowByte = lowByte
M.highByte = highByte
M.twoBytes = twoBytes
M.fourBytes = fourBytes
M.combineTwoBytes = combineTwoBytes
M.dumpsMsg = dumpsMsg
M.dumpsByte = dumpsByte
M.dumpsByteHex = dumpsByteHex

return M
