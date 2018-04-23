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

#include "ccat.h"
#include "CCatCodec.h"
#include <new> // std::nothrow

namespace ccat {


extern "C" {


//------------------------------------------------------------------------------
// API

CCAT_EXPORT CCatResult ccat_create(
    const CCatSettings* settings,
    CCatCodec* codecOut
)
{
    if (!settings || !codecOut) {
        return CCat_InvalidInput;
    }

    *codecOut = nullptr;

    const int gf256Result = gf256_init();
    if (gf256Result != 0) {
        return CCat_Error;
    }

    // Allocate aligned object
    Codec* codec = new (std::nothrow) Codec;
    if (!codec) {
        return CCat_OOM;
    }

    const CCatResult initResult = codec->Create(*settings);
    if (initResult != CCat_Success) {
        return initResult;
    }

    *codecOut = reinterpret_cast<CCatCodec>( codec );
    return CCat_Success;
}

CCAT_EXPORT CCatResult ccat_encode_original(
    CCatCodec codec,
    const CCatOriginal* original
)
{
    Codec* session = reinterpret_cast<Codec*>(codec);
    if (!session) {
        return CCat_InvalidInput;
    }

    return session->EncodeOriginal(*original);
}

CCAT_EXPORT CCatResult ccat_encode_recovery(
    CCatCodec codec,
    CCatRecovery* recoveryOut
)
{
    Codec* session = reinterpret_cast<Codec*>(codec);
    if (!session) {
        return CCat_InvalidInput;
    }

    return session->EncodeRecovery(*recoveryOut);
}

CCAT_EXPORT CCatResult ccat_decode_original(
    CCatCodec codec,
    const CCatOriginal* original
)
{
    Codec* session = reinterpret_cast<Codec*>(codec);
    if (!session) {
        return CCat_InvalidInput;
    }

    return session->DecodeOriginal(*original);
}

CCAT_EXPORT CCatResult ccat_decode_recovery(
    CCatCodec codec,
    const CCatRecovery* recovery
)
{
    Codec* session = reinterpret_cast<Codec*>(codec);
    if (!session) {
        return CCat_InvalidInput;
    }

    CCatResult result = session->DecodeRecovery(*recovery);

    if (result == CCat_NeedsMoreData) {
        // If we need more data, just return a success code to simplify the API.
        return CCat_Success;
    }

    return result;
}

CCAT_EXPORT CCatResult ccat_destroy(
    CCatCodec codec
)
{
    Codec* session = reinterpret_cast<Codec*>(codec);
    if (!session) {
        return CCat_InvalidInput;
    }

    delete session;

    return CCat_Success;
}


} // extern "C"


} // namespace ccat
