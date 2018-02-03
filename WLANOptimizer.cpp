/** \file
    \brief WLAN Optimizer
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of WLAN Optimizer nor the names of its contributors may be
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

#include "WLANOptimizer.h"

#include <mutex>
#include <chrono>
using namespace std::chrono_literals;

static std::mutex APILock;


//------------------------------------------------------------------------------
// Windows Version

#ifdef _WIN32

#undef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN7 /* must be defined for wlanapi.h */
#include <wlanapi.h>
#include <objbase.h>
#pragma comment(lib, "wlanapi")
#pragma comment(lib, "ole32")

// Client handle shared between all calls.
// This is done because settings only persist until handle or app is closed.
static HANDLE m_clientHandle = nullptr;

// Set a Wlan setting
static OptimizeWLAN_Result SetWlanSetting(
    const GUID* guidPtr,
    WLAN_INTF_OPCODE opcode,
    bool enable)
{
    DWORD dataSize = 0;
    void* dataPtr = nullptr;
    WLAN_OPCODE_VALUE_TYPE opcodeType = wlan_opcode_value_type_invalid;
    DWORD queryResult = ::WlanQueryInterface(
        m_clientHandle,
        guidPtr,
        opcode,
        nullptr,
        &dataSize,
        &dataPtr,
        &opcodeType);
    if (queryResult != ERROR_SUCCESS ||
        dataSize < 1 ||
        !dataPtr ||
        opcodeType == wlan_opcode_value_type_invalid)
    {
        return OptimizeWLAN_ReadFailure;
    }

    const bool currentValue = *(BOOL*)dataPtr != 0;
    if (currentValue == enable)
        return OptimizeWLAN_Success;

    BOOL targetValue = enable ? TRUE : FALSE;

    // Note this function takes about 1 second to complete.
    const DWORD setResult = ::WlanSetInterface(
        m_clientHandle,
        guidPtr,
        opcode,
        (DWORD)sizeof(targetValue),
        &targetValue,
        nullptr);
    if (setResult != ERROR_SUCCESS)
    {
        if (setResult == ERROR_ACCESS_DENIED)
            return OptimizeWLAN_AccessDenied;
        return OptimizeWLAN_SetFailure;
    }

    dataSize = 0;
    dataPtr = nullptr;
    opcodeType = wlan_opcode_value_type_invalid;
    queryResult = ::WlanQueryInterface(
        m_clientHandle,
        guidPtr,
        opcode,
        nullptr,
        &dataSize,
        &dataPtr,
        &opcodeType);
    if (queryResult != ERROR_SUCCESS ||
        dataSize < 1 ||
        !dataPtr ||
        *(BOOL*)dataPtr != targetValue ||
        opcodeType == wlan_opcode_value_type_invalid)
    {
        return OptimizeWLAN_ReadFailure;
    }

    return OptimizeWLAN_Success;
}

#endif // _WIN32


//------------------------------------------------------------------------------
// OptimizeWLAN()

int OptimizeWLAN(int enable)
{
    std::lock_guard<std::mutex> locker(APILock);

#ifdef _WIN32

    if (!m_clientHandle)
    {
        const DWORD clientVersion = 2;
        DWORD negotiatedVersion = 0;
        const DWORD openResult = ::WlanOpenHandle(
            clientVersion,
            nullptr,
            &negotiatedVersion,
            &m_clientHandle);
        if (openResult != ERROR_SUCCESS || !m_clientHandle)
            return OptimizeWLAN_Unavailable;
    }

    OptimizeWLAN_Result result = OptimizeWLAN_Success;

    bool foundConnection = false;

    WLAN_INTERFACE_INFO_LIST* infoListPtr = nullptr;
    const DWORD enumResult = ::WlanEnumInterfaces(
        m_clientHandle,
        nullptr,
        &infoListPtr);
    if (enumResult == ERROR_SUCCESS && infoListPtr)
    {
        const int count = (int)infoListPtr->dwNumberOfItems;
        for (int i = 0; i < count; ++i)
        {
            const WLAN_INTERFACE_INFO* info = &infoListPtr->InterfaceInfo[i];

            // In my testing I found that only connected WiFi adapters are able to change this setting,
            // as otherwise WlanSetInterface() returns ERROR_INVALID_STATE.
            if (info->isState == wlan_interface_state_connected)
            {
                OptimizeWLAN_Result setResult;

                setResult = SetWlanSetting(
                    &info->InterfaceGuid,
                    wlan_intf_opcode_media_streaming_mode,
                    enable != 0);
                if (setResult != OptimizeWLAN_Success)
                    result = setResult;

                setResult = SetWlanSetting(
                    &info->InterfaceGuid,
                    wlan_intf_opcode_background_scan_enabled,
                    enable == 0);
                if (setResult != OptimizeWLAN_Success)
                    result = setResult;

                foundConnection = true;
            }
        }
    }

    if (infoListPtr != nullptr)
    {
        ::WlanFreeMemory(infoListPtr);
    }

    // Leak handle intentionally here - When the app closes it will be released.
    //::WlanCloseHandle(m_clientHandle, nullptr);

    if (!foundConnection)
    {
        return OptimizeWLAN_NoConnections;
    }

    return result;

#else //_WIN32

    (void)enable; // Unused

    // TBD: Are there also correctable issues on Mac/Linux?
    return OptimizeWLAN_Unavailable;

#endif //_WIN32
}


//------------------------------------------------------------------------------
// WLANOptimizerThread

class WLANOptimizerThread
{
public:
    /// Start the optimizer thread
    void Start();

    /// Stop the optimizer thread
    void Stop();

protected:
    /// Lock preventing thread safety issues around Start() and Stop()
    mutable std::mutex StartStopLock;

    /// Lock protecting WakeCondition
    mutable std::mutex WakeLock;

    /// Condition that indicates the thread should wake up
    std::condition_variable WakeCondition;

    /// Background thread
    std::shared_ptr<std::thread> Thread;

    /// Should thread terminate?
    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(true);


    /// Thread loop
    void Loop();
};

void WLANOptimizerThread::Start()
{
    std::lock_guard<std::mutex> startStopLocker(StartStopLock);

    if (!Thread)
    {
        Terminated = false;
        Thread = std::make_shared<std::thread>(&WLANOptimizerThread::Loop, this);
    }
}

void WLANOptimizerThread::Stop()
{
    std::lock_guard<std::mutex> startStopLocker(StartStopLock);

    if (Thread)
    {
        Terminated = true;

        // Make sure that queue notification happens after termination flag is set
        {
            std::unique_lock<std::mutex> wakeLocker(WakeLock);
            WakeCondition.notify_all();
        }

        try
        {
            if (Thread->joinable())
                Thread->join();
        }
        catch (std::system_error& /*err*/)
        {
        }
    }
    Thread = nullptr;
}

void WLANOptimizerThread::Loop()
{
    // Time between optimization attempts.  Retries because WiFi might reconnect.
    static const auto kOptimizeInterval = 11s; // chrono_literals

    while (!Terminated)
    {
        int optimizeResult = OptimizeWLAN(1);
        if (optimizeResult != OptimizeWLAN_Success &&
            optimizeResult != OptimizeWLAN_NoConnections)
        {
            break;
        }

        // unique_lock used since QueueCondition.wait requires it
        std::unique_lock<std::mutex> locker(WakeLock);

        if (!Terminated)
            WakeCondition.wait_for(locker, kOptimizeInterval);
    }
}


static WLANOptimizerThread m_WlanOptimizer;

void StartWLANOptimizerThread()
{
#ifdef _WIN32
    // Only run the thread on Windows because currently it has no optimizations
    // for other platforms.
    m_WlanOptimizer.Start();
#endif // _WIN32
}

void StopWLANOptimizerThread()
{
#ifdef _WIN32
    m_WlanOptimizer.Stop();
#endif // _WIN32
}
