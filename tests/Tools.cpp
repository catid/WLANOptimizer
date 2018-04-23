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

#include "Tools.h"

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#elif __MACH__
    #include <mach/mach_time.h>
    #include <mach/mach.h>
    #include <mach/clock.h>

    extern mach_port_t clock_port;
#else
    #include <time.h>
    #include <sys/time.h>
#endif


namespace wopt {

static logger::Channel ModuleLogger("Tools", kMinLogLevel);


//------------------------------------------------------------------------------
// Thread Tools

#ifdef _WIN32
const DWORD MS_VC_EXCEPTION = 0x406D1388;
#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
    DWORD dwType;       // Must be 0x1000.
    LPCSTR szName;      // Pointer to name (in user addr space).
    DWORD dwThreadID;   // Thread ID (-1=caller thread).
    DWORD dwFlags;      // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)
void SetCurrentThreadName(const char* threadName)
{
    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = threadName;
    info.dwThreadID = ::GetCurrentThreadId();
    info.dwFlags = 0;
#pragma warning(push)
#pragma warning(disable: 6320 6322)
    __try
    {
        RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
#pragma warning(pop)
}
#else
void SetCurrentThreadName(const char* threadName)
{
    pthread_setname_np(pthread_self(), threadName);
}
#endif

bool SetCurrentThreadPriority(ThreadPriority prio)
{
#ifdef _WIN32
    int winPrio = THREAD_PRIORITY_NORMAL;
    switch (prio)
    {
    case ThreadPriority::High: winPrio = THREAD_PRIORITY_ABOVE_NORMAL; break;
    case ThreadPriority::Low:  winPrio = THREAD_PRIORITY_BELOW_NORMAL; break;
    case ThreadPriority::Idle: winPrio = THREAD_PRIORITY_IDLE; break;
    default: break;
    }
    return 0 != ::SetThreadPriority(::GetCurrentThread(), winPrio);
#else
    int niceness = 0;
    switch (prio)
    {
    case ThreadPriority::High: niceness = 2; break;
    case ThreadPriority::Low:  niceness = -2; break;
    case ThreadPriority::Idle: niceness = 19; break;
    default: break;
    }
    return -1 != nice(niceness);
#endif
}

bool SetCurrentThreadAffinity(unsigned processorIndex)
{
#ifdef _WIN32
    return 0 != ::SetThreadAffinityMask(
        ::GetCurrentThread(), (DWORD_PTR)1 << (processorIndex & 63));
#elif !defined(ANDROID)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(processorIndex, &cpuset);
    return 0 == pthread_setaffinity_np(pthread_self(),
        sizeof(cpu_set_t), &cpuset);
#else
    return true; // FIXME: Unused on Android anyway
#endif
}


//------------------------------------------------------------------------------
// Timing

#ifdef _WIN32
static double PerfFrequencyInverse = 0.;

static void InitPerfFrequencyInverse()
{
    LARGE_INTEGER freq = {};
    if (!::QueryPerformanceFrequency(&freq) || freq.QuadPart == 0)
        return;
    PerfFrequencyInverse = 1000000. / (double)freq.QuadPart;
}
#elif __MACH__
static bool m_clock_serv_init = false;
static clock_serv_t m_clock_serv = 0;

static void InitClockServ()
{
    m_clock_serv_init = true;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &m_clock_serv);
}
#endif // _WIN32

uint64_t GetTimeUsec()
{
#ifdef _WIN32
    LARGE_INTEGER timeStamp = {};
    if (!::QueryPerformanceCounter(&timeStamp))
        return 0;
    if (PerfFrequencyInverse == 0.)
        InitPerfFrequencyInverse();
    return (uint64_t)(PerfFrequencyInverse * timeStamp.QuadPart);
#elif __MACH__
    if (!m_clock_serv_init)
        InitClockServ();

    mach_timespec_t tv;
    clock_get_time(m_clock_serv, &tv);

    return 1000000 * tv.tv_sec + tv.tv_nsec / 1000;
#else
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return 1000000 * tv.tv_sec + tv.tv_usec;
#endif
}

uint64_t GetTimeMsec()
{
    return GetTimeUsec() / 1000;
}


//------------------------------------------------------------------------------
// Socket

Socket::Socket()
    : Logger("Socket", kMinLogLevel)
{
}

bool Socket::Initialize(uint16_t port)
{
    try
    {
        Port = port;

        // Set logger prefix
        std::ostringstream oss;
        oss << "[Port " << std::to_string(port) << "] ";
        Logger.SetPrefix(oss.str());

        AsioContext = MakeSharedNoThrow<asio::io_context>();
        if (!AsioContext)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("OOM");
            goto OnError;
        }
        AsioContext->restart();

        // Create Asio objects
        AsioTicker = MakeUniqueNoThrow<asio::steady_timer>(*AsioContext);
        AsioUDPSocket = MakeUniqueNoThrow<asio::ip::udp::socket>(*AsioContext);
        AsioStrand = MakeUniqueNoThrow<asio::io_context::strand>(*AsioContext);
        if (!AsioUDPSocket || !AsioTicker || !AsioStrand)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("OOM");
            goto OnError;
        }

        postNextTimer();

        // Create an IPv4 socket
        const UDPAddress bindAddress(asio::ip::udp::v4(), port);

        asio::error_code error;
        AsioUDPSocket->open(bindAddress.protocol(), error);
        if (error)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("Socket open error: ", error.message());
            goto OnError;
        }

        AsioUDPSocket->bind(bindAddress, error);
        if (error)
        {
            //WOPT_DEBUG_BREAK();
            Logger.Error("Socket bind error: ", error.message());
            goto OnError;
        }

        AsioUDPSocket->set_option(asio::socket_base::send_buffer_size(kSendBufferSizeBytes), error);
        if (error)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("Socket set_option send_buffer_size error: ", error.message());
            goto OnError;
        }

        AsioUDPSocket->set_option(asio::socket_base::receive_buffer_size(kRecvBufferSizeBytes), error);
        if (error)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("Socket set_option receive_buffer_size error: ", error.message());
            goto OnError;
        }

        AsioUDPSocket->set_option(asio::socket_base::broadcast(true), error);
        if (error) {
            WOPT_DEBUG_BREAK();
            Logger.Error("Socket set_option broadcast error: ", error.message());
            goto OnError;
        }

        AsioUDPSocket->set_option(asio::socket_base::reuse_address(true), error);
        if (error)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("Socket reuse_address error: ", error.message());
            goto OnError;
        }

        Thread = MakeUniqueNoThrow<std::thread>(&Socket::workerLoop, this);
        if (!Thread)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("OOM");
            goto OnError;
        }

        uint8_t* readBuffer = RWBufferAllocator.Allocate(kMaxPacketBytes);
        if (!readBuffer)
        {
            WOPT_DEBUG_BREAK();
            Logger.Error("OOM");
            goto OnError;
        }

        postNextRead(readBuffer);

        return true;
    }
    catch (...)
    {
        WOPT_DEBUG_BREAK();
        Logger.Error("Exception during init");
        goto OnError;
    }

OnError:
    return false;
}

void Socket::Shutdown()
{
    std::lock_guard<std::mutex> locker(APILock);

    if (AsioContext) {
        // Note there is no need to stop or cancel timer ticks or sockets
        AsioContext->stop();
    }

    // Allow worker threads to stop.
    // Note this has to be done after cancel/close of sockets above because
    // otherwise Asio can hang waiting for any outstanding callbacks.
    Terminated = true;

    // Wait for thread to stop
    if (Thread)
    {
        try
        {
            if (Thread->joinable()) {
                Thread->join();
            }
        }
        catch (std::system_error& err)
        {
            Logger.Warning("Exception while joining thread: ", err.what());
        }
        Thread = nullptr;
    }

    // Destroy Asio objects
    AsioTicker = nullptr;
    AsioUDPSocket = nullptr;
    AsioContext = nullptr;
    AsioStrand = nullptr;

    Logger.Info("Queue memory used: ", RWBufferAllocator.GetUsedMemory() / 1000, " KB");
}

void Socket::workerLoop()
{
    SetCurrentThreadName("Socket:Worker");

    while (!Terminated) {
        AsioContext->run();
    }
}

void Socket::postNextTimer()
{
    AsioTicker->expires_after(std::chrono::microseconds(kTickIntervalUsec));

    AsioTicker->async_wait(([this](const asio::error_code& error)
    {
        if (!error)
        {
            AsioStrand->post([this]()
            {
                OnTick();
            });
            postNextTimer();
        }
    }));
}

void Socket::Send(const UDPAddress& addr, const uint8_t* data, unsigned bytes)
{
    uint8_t* buffer = RWBufferAllocator.Allocate(bytes);
    memcpy(buffer, data, bytes);

    AsioUDPSocket->async_send_to(
        asio::buffer(buffer, bytes),
        addr, [this, buffer](const asio::error_code&, std::size_t)
    {
        RWBufferAllocator.Free(buffer);
    });
}

void Socket::postNextRead(uint8_t* readBuffer)
{
    // Clear source address to allow us to check if it's filled later
    // during error handling
    SourceAddress = UDPAddress();

    // Dispatch asynchronous recvfrom()
    AsioUDPSocket->async_receive_from(
        asio::buffer(readBuffer, kMaxPacketBytes),
        SourceAddress,
        [this, readBuffer](const asio::error_code& error, size_t bytes_transferred)
    {
        const uint64_t receiveUsec = GetTimeUsec();

        uint8_t* nextReadBuffer = readBuffer;

        // If there was an error reported:
        if (error)
        {
            // If no source address was provided:
            if (SourceAddress.address().is_unspecified())
            {
                Logger.Warning("Socket broken based on recvfrom with no source address: ", error.message());
                return; // Stop here
            }

            // Otherwise we assume it is due to ICMP message from a peer:
            Logger.Debug("recvfrom failed (ICMP unrecognized source): ", error.message());

            // Fall-thru to continue reading datagrams
        }
        else if (bytes_transferred <= 0)
        {
            Logger.Warning("Socket closed");
            return; // Stop here
        }
        else if (!Terminated)
        {
            UDPAddress addr = SourceAddress;

            AsioStrand->post([receiveUsec, readBuffer, bytes_transferred, addr, this]()
            {
                OnRead(addr, receiveUsec, readBuffer, (unsigned)bytes_transferred);

                RWBufferAllocator.Free(readBuffer);
            });

            nextReadBuffer = RWBufferAllocator.Allocate(kMaxPacketBytes);
        }

        // Post next read buffer
        postNextRead(nextReadBuffer);
    });
}


} // namespace wopt
