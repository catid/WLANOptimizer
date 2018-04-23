/** \file
    \brief WLANOptimzer Test: Tools
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of WLANOptimizer nor the names of its contributors may be
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

#pragma once

#include "../thirdparty/IncludeAsio.h"
#include "tools/Logger.h"
#include "tools/PacketAllocator.h"
#include "tools/ccat.h"
#include "tools/gf256.h"

#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <stdint.h>


//------------------------------------------------------------------------------
// Portability macros

// Specify an intentionally unused variable (often a function parameter)
#define WOPT_UNUSED(x) (void)(x);

// Compiler-specific debug break
#if defined(_DEBUG) || defined(DEBUG)
    #define WOPT_DEBUG
    #if defined(_WIN32)
        #define WOPT_DEBUG_BREAK() __debugbreak()
    #else // _WIN32
        #define WOPT_DEBUG_BREAK() __builtin_trap()
    #endif // _WIN32
    #define WOPT_DEBUG_ASSERT(cond) { if (!(cond)) { WOPT_DEBUG_BREAK(); } }
    #define WOPT_IF_DEBUG(x) x;
#else // _DEBUG
    #define WOPT_DEBUG_BREAK() ;
    #define WOPT_DEBUG_ASSERT(cond) ;
    #define WOPT_IF_DEBUG(x) ;
#endif // _DEBUG

// Compiler-specific force inline keyword
#if defined(_MSC_VER)
    #define WOPT_FORCE_INLINE inline __forceinline
#else // _MSC_VER
    #define WOPT_FORCE_INLINE inline __attribute__((always_inline))
#endif // _MSC_VER


namespace wopt {


//------------------------------------------------------------------------------
// Constants

#ifdef WOPT_DEBUG
static const logger::Level kMinLogLevel = logger::Level::Debug;
#else
static const logger::Level kMinLogLevel = logger::Level::Info;
#endif

static const unsigned kTickIntervalUsec = 20 * 1000; /// 100 ms interval


//------------------------------------------------------------------------------
// Threads

/// Set the current thread processor affinity
bool SetCurrentThreadAffinity(unsigned processorIndex);

enum class ThreadPriority
{
    High,
    Normal,
    Low,
    Idle
};

/// Set the thread priority of the current thread
bool SetCurrentThreadPriority(ThreadPriority prio);

/// Set the current thread name
void SetCurrentThreadName(const char* name);


//------------------------------------------------------------------------------
// BufferAllocator

/// Threadsafe version of pktalloc::Allocator
class BufferAllocator
{
public:
    /// Allocate a new buffer
    WOPT_FORCE_INLINE uint8_t* Allocate(unsigned bytes)
    {
        std::lock_guard<std::mutex> locker(AllocatorLock);
        return Allocator.Allocate(bytes);
    }

    /// Shrink the given buffer to size
    WOPT_FORCE_INLINE void Shrink(void* buffer, unsigned bytes)
    {
        std::lock_guard<std::mutex> locker(AllocatorLock);
        return Allocator.Shrink(reinterpret_cast<uint8_t*>(buffer), bytes);
    }

    /// Free the buffer
    WOPT_FORCE_INLINE void Free(void* buffer)
    {
        std::lock_guard<std::mutex> locker(AllocatorLock);
        return Allocator.Free(reinterpret_cast<uint8_t*>(buffer));
    }

    /// Get statistics for the allocator
    WOPT_FORCE_INLINE uint64_t GetUsedMemory() const
    {
        std::lock_guard<std::mutex> locker(AllocatorLock);
        return Allocator.GetMemoryAllocatedBytes();
    }

protected:
    /// Lock protecting the allocator
    mutable std::mutex AllocatorLock;

    /// Buffer allocator
    pktalloc::Allocator Allocator;
};


//------------------------------------------------------------------------------
// Portability Helpers

template<typename T, typename... Args>
WOPT_FORCE_INLINE std::unique_ptr<T> MakeUniqueNoThrow(Args&&... args) {
    return std::unique_ptr<T>(new(std::nothrow) T(std::forward<Args>(args)...));
}

template<typename T, typename... Args>
WOPT_FORCE_INLINE std::shared_ptr<T> MakeSharedNoThrow(Args&&... args) {
    return std::shared_ptr<T>(new(std::nothrow) T(std::forward<Args>(args)...));
}


//------------------------------------------------------------------------------
// POD Serialization

WOPT_FORCE_INLINE uint16_t ReadU16_LE(const uint8_t* data)
{
#ifdef GF256_ALIGNED_ACCESSES
    return ((uint16_t)data[1] << 8) | data[0];
#else
    return *(uint16_t*)data;
#endif
}

WOPT_FORCE_INLINE uint32_t ReadU24_LE(const uint8_t* data)
{
    return ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
}

/// This version uses one memory read on Intel but requires at least 4 bytes in the buffer
WOPT_FORCE_INLINE uint32_t ReadU24_LE_Min4Bytes(const uint8_t* data)
{
#ifdef GF256_ALIGNED_ACCESSES
    return ReadU24_LE(data);
#else
    return *(uint32_t*)data & 0xFFFFFF;
#endif
}

WOPT_FORCE_INLINE uint32_t ReadU32_LE(const uint8_t* data)
{
#ifdef GF256_ALIGNED_ACCESSES
    return ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
#else
    return *(uint32_t*)data;
#endif
}

WOPT_FORCE_INLINE uint64_t ReadU64_LE(const uint8_t* data)
{
#ifdef GF256_ALIGNED_ACCESSES
    return ((uint64_t)data[7] << 56) | ((uint64_t)data[6] << 48) | ((uint64_t)data[5] << 40) |
        ((uint64_t)data[4] << 32) | ((uint64_t)data[3] << 24) | ((uint64_t)data[2] << 16) |
        ((uint64_t)data[1] << 8) | data[0];
#else
    return *(uint64_t*)data;
#endif
}

WOPT_FORCE_INLINE void WriteU16_LE(uint8_t* data, uint16_t value)
{
#ifdef GF256_ALIGNED_ACCESSES
    data[1] = (uint8_t)(value >> 8);
    data[0] = (uint8_t)value;
#else
    *(uint16_t*)data = value;
#endif
}

WOPT_FORCE_INLINE void WriteU24_LE(uint8_t* data, uint32_t value)
{
    data[2] = (uint8_t)(value >> 16);
    WriteU16_LE(data, (uint16_t)value);
}

WOPT_FORCE_INLINE void WriteU24_LE_Min4Bytes(uint8_t* data, uint32_t value)
{
#ifdef GF256_ALIGNED_ACCESSES
    WriteU24_LE(data, value);
#else
    *(uint32_t*)data = value;
#endif
}

WOPT_FORCE_INLINE void WriteU32_LE(uint8_t* data, uint32_t value)
{
#ifdef GF256_ALIGNED_ACCESSES
    data[3] = (uint8_t)(value >> 24);
    data[2] = (uint8_t)(value >> 16);
    data[1] = (uint8_t)(value >> 8);
    data[0] = (uint8_t)value;
#else
    *(uint32_t*)data = value;
#endif
}

WOPT_FORCE_INLINE void WriteU64_LE(uint8_t* data, uint64_t value)
{
#ifdef GF256_ALIGNED_ACCESSES
    data[7] = (uint8_t)(value >> 56);
    data[6] = (uint8_t)(value >> 48);
    data[5] = (uint8_t)(value >> 40);
    data[4] = (uint8_t)(value >> 32);
    data[3] = (uint8_t)(value >> 24);
    data[2] = (uint8_t)(value >> 16);
    data[1] = (uint8_t)(value >> 8);
    data[0] = (uint8_t)value;
#else
    *(uint64_t*)data = value;
#endif
}


//------------------------------------------------------------------------------
// Timing

/// Platform independent high-resolution timers
uint64_t GetTimeUsec();
uint64_t GetTimeMsec();


//------------------------------------------------------------------------------
// Socket

typedef asio::ip::udp::endpoint UDPAddress;

// Derive from this to handle reads
class Socket
{
public:
    Socket();
    ~Socket()
    {
        Shutdown();
    }
    bool Initialize(uint16_t port);
    void Shutdown();

    /// Read handler
    virtual void OnRead(const UDPAddress& addr, uint64_t receiveUsec, uint8_t* buffer, unsigned bytes) = 0;

    /// Called whenever the ticker ticks
    virtual void OnTick() = 0;

    /// Send data
    void Send(const UDPAddress& addr, const uint8_t* data, unsigned bytes);

protected:
    static const unsigned kSendBufferSizeBytes = 64000;
    static const unsigned kRecvBufferSizeBytes = 64000;
    static const unsigned kMaxPacketBytes = 1400;

    /// Logging channel
    logger::Channel Logger;

    /// Bound port
    uint16_t Port;

    /// Address associated with the last packet we received (maybe not our peer)
    UDPAddress SourceAddress;

    /// Mutex to prevent API calls from being made concurrently
    std::mutex APILock;

    /// Asio context
    std::shared_ptr<asio::io_context> AsioContext;

    /// UDP socket
    std::unique_ptr<asio::ip::udp::socket> AsioUDPSocket;

    /// Allocator for read/write buffers
    BufferAllocator RWBufferAllocator;

    /// Should worker thread be terminated?
    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);

    /// Worker thread
    std::unique_ptr<std::thread> Thread;

    /// Timer for retries and timeouts (and to avoid using 100% CPU)
    std::unique_ptr<asio::steady_timer> AsioTicker;

    /// Strand that serializes connection requests
    std::shared_ptr<asio::io_context::strand> AsioStrand;


    /// Worker thread loop
    void workerLoop();

    /// Post a timer tick event
    void postNextTimer();

    /// Post next asynchronous read request
    void postNextRead(uint8_t* readBuffer);
};


} // namespace wopt
