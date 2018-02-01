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

static std::mutex APILock;


//------------------------------------------------------------------------------
// Windows Version

#ifdef _WIN32

#undef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN7 /* must be defined for wlanapi.h */
#include <wlanapi.h>
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
