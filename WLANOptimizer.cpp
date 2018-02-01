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

#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT _WIN32_WINNT_WIN7 /* must be defined for wlanapi.h */
#include <wlanapi.h>
#pragma comment(lib, "wlanapi")
#pragma comment(lib, "ole32")
#endif

// This stuff is used to generate the magic SDDL
#if 0

#include <Sddl.h>

void generateMagic()
{
    PSECURITY_DESCRIPTOR securityDescriptor = (PSECURITY_DESCRIPTOR)LocalAlloc(LMEM_FIXED, sizeof(SECURITY_DESCRIPTOR));
    PACL pAcl = (PACL)LocalAlloc(LMEM_FIXED, sizeof(PACL));
    SID_IDENTIFIER_AUTHORITY SIDAuthWorld = SECURITY_WORLD_SID_AUTHORITY;
    PSID pEveryoneSID = NULL;
    BOOL bRet, bRes = true;


    bRet = InitializeSecurityDescriptor(securityDescriptor, SECURITY_DESCRIPTOR_REVISION);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not initialize Security Descriptor! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor);

    bRet = AllocateAndInitializeSid(&SIDAuthWorld, 1,
        SECURITY_WORLD_RID,
        0, 0, 0, 0, 0, 0, 0,
        &pEveryoneSID);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not allocate and initialize SID! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor);

    bRet = SetSecurityDescriptorOwner(securityDescriptor, pEveryoneSID, TRUE);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not set Security Descriptor Owner! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor);

    DWORD cbAcl = sizeof(ACL) +
        (sizeof(ACCESS_ALLOWED_ACE)) + (GetLengthSid(securityDescriptor) - sizeof(DWORD));
    bRet = InitializeAcl(pAcl, cbAcl, ACL_REVISION);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not initialize ACL! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidAcl(pAcl);

    const DWORD aceval = WLAN_READ_ACCESS | WLAN_EXECUTE_ACCESS | WLAN_WRITE_ACCESS;
    bRet = AddAccessAllowedAce(pAcl, ACL_REVISION, aceval, securityDescriptor);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not add Access Allowed ACE! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor);

    bRet = SetSecurityDescriptorDacl(securityDescriptor, TRUE, pAcl, FALSE);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not set Security Descriptor DACL! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor);

    LPWSTR pStringSecurityDescriptor = nullptr;
    ULONG len = 0;
    bRet = ConvertSecurityDescriptorToStringSecurityDescriptorW(securityDescriptor,
        SDDL_REVISION_1,
        DACL_SECURITY_INFORMATION,
        &pStringSecurityDescriptor,
        &len
    );
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not set convert SID to String-SID! %sn", GetLastError());
        bRes = false;
    }

    DWORD secdesc2sz = (DWORD)sizeof(SECURITY_DESCRIPTOR);
    DWORD paclsz = cbAcl;
    DWORD ownsz = sizeof(SID);
    PSECURITY_DESCRIPTOR securityDescriptor2 = (PSECURITY_DESCRIPTOR)LocalAlloc(LMEM_FIXED, sizeof(SECURITY_DESCRIPTOR));

    bRet = InitializeSecurityDescriptor(securityDescriptor2, SECURITY_DESCRIPTOR_REVISION);
    if (!bRet)
    {
        //ST_TRACE(ST_LOG_LEVEL_WIFI_ERROR, L"Could not initialize Security Descriptor! %sn", GetLastError());
        bRes = false;
    }

    bRet = IsValidSecurityDescriptor(securityDescriptor2);

    BOOL x = MakeAbsoluteSD(
        securityDescriptor,
        securityDescriptor2,
        &secdesc2sz,
        pAcl,
        &paclsz,
        nullptr,
        nullptr,
        pEveryoneSID,
        &ownsz,
        nullptr,
        nullptr);
    DWORD err = ::GetLastError();
    int y = 0;
}

#endif

static std::mutex APILock;

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
        return OptimizeWLAN_Result::ReadFailure;
    }

    const bool currentValue = *(BOOL*)dataPtr != 0;
    if (currentValue == enable)
        return OptimizeWLAN_Result::Success;

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
            return OptimizeWLAN_Result::AccessDenied;
        return OptimizeWLAN_Result::SetFailure;
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
        return OptimizeWLAN_Result::ReadFailure;
    }

    return OptimizeWLAN_Result::Success;
}

// Does not actually work unfortunately.  Poorly documented and vague error code.
#if 0

static bool AllowUsersToEditSettings()
{
    //generateMagic();

    // Magic.
    static const LPCWSTR kEveryoneSDDL = L"D:(A;;0x00070023;;;S-1-1-0)";

    // TBD: Does this change persist between reboots?
    const DWORD streamingSecurityResult = ::WlanSetSecuritySettings(
        m_clientHandle,
        wlan_secure_media_streaming_mode_enabled,
        kEveryoneSDDL);
    if (streamingSecurityResult != ERROR_SUCCESS)
    {
        return false;
    }

    const DWORD bcSecurityResult = ::WlanSetSecuritySettings(
        m_clientHandle,
        wlan_secure_bc_scan_enabled,
        kEveryoneSDDL);
    if (bcSecurityResult != ERROR_SUCCESS)
    {
        return false;
    }

    return true;
}

#endif

OptimizeWLAN_Result OptimizeWLAN(bool enable)
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
            return OptimizeWLAN_Result::Unavailable;
    }

    //AllowUsersToEditSettings();

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
            if (info->isState == wlan_interface_state_connected)
            {
                OptimizeWLAN_Result result;

                result = SetWlanSetting(
                    &info->InterfaceGuid,
                    wlan_intf_opcode_media_streaming_mode,
                    enable);
                if (result != OptimizeWLAN_Result::Success)
                    return result;

                result = SetWlanSetting(
                    &info->InterfaceGuid,
                    wlan_intf_opcode_background_scan_enabled,
                    !enable);
                if (result != OptimizeWLAN_Result::Success)
                    return result;
            }
        }
    }

    if (infoListPtr != nullptr)
    {
        ::WlanFreeMemory(infoListPtr);
    }

    // Leak handle intentionally here - When the app closes it will be released.
    //::WlanCloseHandle(m_clientHandle, nullptr);

    return OptimizeWLAN_Result::Success;
#else //_WIN32
    // TBD: Are there also correctable issues on Mac/Linux?
    return OptimizeWLAN_Result::Unavailable;
    (void)enable; // Unused
#endif //_WIN32
}
