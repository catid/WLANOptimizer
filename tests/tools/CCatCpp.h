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

#pragma once

#include "ccat.h"

/// C++ convenience wrapper around the C API
/// Override OnRecoveredData() to receive data
class CauchyCaterpillar
{
public:
    // Initialize and pick window size in milliseconds
    bool Initialize(unsigned windowMsec = 100)
    {
        Destroy();

        CCatSettings settings;
        settings.AppContextPtr = this;
        settings.OnRecoveredData = [](CCatOriginal original, void* context)
        {
            CauchyCaterpillar* thiz = (CauchyCaterpillar*)context;
            thiz->OnRecoveredData(original);
        };
        settings.WindowMsec = windowMsec;
        settings.WindowPackets = CCAT_MAX_WINDOW_PACKETS;

        CCatResult result = ccat_create(&settings, &Codec);
        if (result != CCat_Success)
        {
            Error = true;
            return false;
        }

        Error = false;
        return true;
    }

    bool IsError() const
    {
        return Error;
    }

    void Destroy()
    {
        if (Codec)
        {
            ccat_destroy(Codec);
            Codec = nullptr;
        }
    }

    virtual ~CauchyCaterpillar()
    {
        Destroy();
    }

    // Received data handlers:

    void OnOriginal(const CCatOriginal& original)
    {
        CCatResult result = ccat_decode_original(Codec, &original);
        if (result != CCat_Success)
            Error = true;
    }

    void OnRecovery(const CCatRecovery& recovery)
    {
        CCatResult result = ccat_decode_recovery(Codec, &recovery);
        if (result != CCat_Success)
            Error = true;
    }

    // Outgoing data:

    void SendOriginal(const CCatOriginal& original)
    {
        CCatResult result = ccat_encode_original(Codec, &original);
        if (result != CCat_Success)
            Error = true;
    }

    bool SendRecovery(CCatRecovery& recovery)
    {
        CCatResult result = ccat_encode_recovery(Codec, &recovery);
        if (result != CCat_Success)
            Error = true;
        return result == CCat_Success;
    }

protected:
    virtual void OnRecoveredData(const CCatOriginal& original)
    {
        // Default does nothing
        (void)original;
    }

    bool Error = false;
    CCatCodec Codec = nullptr;
};
