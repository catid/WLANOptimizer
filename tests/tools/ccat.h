/** \file
    \brief CCat: The Cauchy Caterpillar
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of CCat nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CAT_CCAT_H
#define CAT_CCAT_H

/** \mainpage
    CCat : The Cauchy Caterpillar

    ccat.h provides a simple C API for streaming erasure codes with a
    fixed window.  This is designed for applications that do not need
    data reliability, but want to improve robustness to packetloss by
    using UDP sockets.  It supports up to 30% redundancy.

    Example applications:
    * VoIP
    * Video conferencing/live broadcast
    * Live telemetry/statistics feeds

    Usage:

    (1) Call ccat_create() to create a CCatCodec object.

    (2) When sending a packet, pass it to ccat_encode_original().

    (3) To encode recovery data, call ccat_encode_recovery(), which generates
    a packet that can be sent over the network to fill in for losses.

    (4) When receiving a packet, pass originals to ccat_decode_original().
    Pass encoded data to the ccat_decode_recovery() function.  When recovery
    occurs it will call the application's OnRecoveredData() callback.

    Thread-safety:

    Applications using the library can use different locks to protect the
    ccat_encode_*() functions and the ccat_decode_*() functions, because no data
    is shared between those.  Otherwise the library is not thread-safe and
    does require locking on the application-side.

    Packet de-duplication:

    CCat will not deliver two packets with the same sequence number.

    Packet re-ordering:

    CCat can deliver data out of order, so its output should be fed into
    a dejitter buffer.

    Alternatives:

    For faster streams, using Siamese FEC is recommended:
    https://github.com/catid/siamese/

    For fountain codes, using Wirehair FEC is recommended:
    https://github.com/catid/wirehair/
*/

/// Library version
#define CCAT_VERSION 3

// Tweak if the functions are exported or statically linked
//#define CCAT_DLL /* Defined when building/linking as DLL */
//#define CCAT_BUILDING /* Defined by the library makefile */

#if defined(CCAT_BUILDING)
# if defined(CCAT_DLL)
    #define CCAT_EXPORT __declspec(dllexport)
# else
    #define CCAT_EXPORT
# endif
#else
# if defined(CCAT_DLL)
    #define CCAT_EXPORT __declspec(dllimport)
# else
    #define CCAT_EXPORT extern
# endif
#endif

#include <stdint.h>


#ifdef __cplusplus
#define CCAT_CPP(x) x
extern "C" {
#else
#define CCAT_CPP(x)
#endif


//------------------------------------------------------------------------------
// Shared Constants/Datatypes

/// Maximum size of packets in bytes
#define CCAT_MAX_BYTES 65536

/// Maximum size of encoder window in packets
#define CCAT_MAX_WINDOW_PACKETS 192

/// Maximum value for recovery row
#define CCAT_MAX_RECOVERY_ROW 63

/// Minimum size of encoder window in milliseconds
#define CCAT_MIN_WINDOW_MSEC 10

/// These are the result codes that can be returned from the ccat_*() functions
typedef enum CCatResult_t
{
    /// Success code
    CCat_Success            = 0,

    /// A function parameter was invalid
    CCat_InvalidInput       = 1,

    /// An error occurred during the request
    CCat_Error              = 2,

    /// Out of memory
    CCat_OOM                = 3,

    /// Needs to be fed more data before it can recover more
    CCat_NeedsMoreData      = 4,

    CCatResult_Count, /* for asserts */
    CCatResult_Padding = 0x7fffffff /* int32_t padding */
} CCatResult;

/// Evaluates true if the result is a success code
#define CCAT_GOOD(result) (CCat_Success == (result))

/// Evaluates true if the result is a failure code
#define CCAT_FAILED(result) (CCat_Success != (result))

/// CCatCodec: Represents a UDP port socket running the CCat proxy
typedef struct CCatCodec_t { int impl; }* CCatCodec;

/// CCatAppContextPtr: Points to application context data
typedef void* CCatAppContext;

/// CCat Original Packet
typedef struct CCatOriginal_t
{
    /// Pointer to buffer containing original data
    const uint8_t* Data;

    /// Number of bytes in buffer
    unsigned Bytes;

    /// Packet sequence number.
    /// This must be an incrementing sequence number, starting from 0, which
    /// increments by 1 each time a packet is sent.
    /// Suggestion: Truncate this and reconstruct it using Counter.h
    uint64_t SequenceNumber;
} CCatOriginal;

/// CCat Recovery Packet generated by ccat_encode_recovery()
typedef struct CCatRecovery_t
{
    /// Sequence start parameter.
    /// Provided by ccat_encode_recovery().
    /// Suggestion: Truncate this and reconstruct it using Counter.h
    uint64_t SequenceStart;

    /// Pointer to buffer containing recovery data
    const uint8_t* Data;

    /// Number of bytes in buffer
    unsigned Bytes;

    /// Count parameter: Ranges from 1 ... CCAT_MAX_WINDOW_PACKETS
    /// Or from 1 ... CCatSettings::WindowPackets provided by the application,
    /// whichever is smaller.  This is filled in by ccat_encode_recovery().
    uint8_t Count;

    /// Recovery row parameter: Ranges from 0 ... CCAT_MAX_RECOVERY_ROW
    uint8_t RecoveryRow;
} CCatRecovery;

/// CCat Settings
typedef struct CCatSettings_t
{
    /// Maximum window size in packets
    unsigned WindowPackets CCAT_CPP( = 64 );

    /// Maximum memory of window in milliseconds
    unsigned WindowMsec CCAT_CPP( = 100 );

    /// Application context pointer provided to callbacks
    CCatAppContext AppContextPtr CCAT_CPP( = nullptr );

    /**
        OnRecoveredData()

        Provide a callback function to receive recovered data.

        This will be invoked during the call to ccat_decode_original()
        or ccat_decode_recovery().

        It is provided the AppContextPtr in the settings.
    */
    void (*OnRecoveredData)(
        CCatOriginal original, ///< Recovered original data
        CCatAppContext context ///< AppContextPtr
        ) CCAT_CPP( = nullptr );
} CCatSettings;


//------------------------------------------------------------------------------
// API

/**
    ccat_create()

    Creates a CCat codec object that can be used to send/receive data.
    The codec includes features for both encoding and decoding in one object.

    Returns CCat_Success on success, codecOut is set to the created codec.
    Returns other codes on failure, codecOut will be 0.
*/
CCAT_EXPORT CCatResult ccat_create(
    const CCatSettings* settings,
    CCatCodec* codecOut
);

/**
    ccat_encode_original()

    After (or just before) sending an original packet, call this function to
    add the packet to the streaming erasure code output.

    Returns CCat_Success on success.
    Returns CCat_InvalidInput if:
        original->Bytes == 0
        original->Bytes > CCAT_MAX_BYTES
        original->SequenceNumber is not the correct incrementing sequence number
        original->Data is null
    Returns CCat_OOM if out of memory.
*/
CCAT_EXPORT CCatResult ccat_encode_original(
    CCatCodec codec,
    const CCatOriginal* original
);

/**
    ccat_encode_recovery()

    Call this function to generate a recovery message that can fill in for
    missing packets.

    Returns CCat_Success on success.
    Returns CCat_NeedsMoreData if recovery packets cannot be produced.
    Returns other values on error.
*/
CCAT_EXPORT CCatResult ccat_encode_recovery(
    CCatCodec codec,
    CCatRecovery* recoveryOut
);

/**
    ccat_decode_original()

    When the app receives an original packet, pass it to this function.

    Returns CCat_Success on success.
    Returns other codes on failure.
*/
CCAT_EXPORT CCatResult ccat_decode_original(
    CCatCodec codec,
    const CCatOriginal* original
);

/**
    ccat_decode_recovery()

    Call this function to generate a recovery message that can fill in for
    missing packets.

    Returns CCat_Success on success.
    Returns other codes on failure.
*/
CCAT_EXPORT CCatResult ccat_decode_recovery(
    CCatCodec codec,
    const CCatRecovery* recovery
);

/**
    ccat_destroy()

    Returns CCat_Success on success.
    Returns other codes on failure.
*/
CCAT_EXPORT CCatResult ccat_destroy(
    CCatCodec codec
);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CAT_CCAT_H
