/** \file
    \brief Strike Register
    \copyright Copyright (c) 2017-2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of StrikeRegister nor the names of its contributors may be
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

#include "StrikeRegister.h"

// Compiler-specific debug break
#if defined(_DEBUG) || defined(DEBUG)
    #define SECURITY_DEBUG
    #if defined(_WIN32)
        #define SECURITY_DEBUG_BREAK() __debugbreak()
    #else
        #define SECURITY_DEBUG_BREAK() __builtin_trap()
    #endif
    #define SECURITY_DEBUG_ASSERT(cond) { if (!(cond)) { SECURITY_DEBUG_BREAK(); } }
#else
    #define SECURITY_DEBUG_BREAK() do {} while (false);
    #define SECURITY_DEBUG_ASSERT(cond) do {} while (false);
#endif

namespace security {


//------------------------------------------------------------------------------
// StrikeRegister

void StrikeRegister::Reset(Counter64 newBase)
{
    WindowBase = newBase;
    WindowBitRotation = 0;
    for (unsigned ii = 0; ii < kWordCount; ++ii) {
        SlidingWindow[ii] = 0;
    }
}

bool StrikeRegister::IsDuplicate(Counter64 sequence) const
{
    // If we are before the sliding window:
    if (sequence < WindowBase) {
        return true; // Too old to check
    }

    const uint64_t distance = (sequence - WindowBase).ToUnsigned();

    // If it is too far ahead to be contained by the sliding window:
    if (distance >= kStrikeRegisterBits) {
        return false;
    }

    // Check if bit is set in the sliding window
    const unsigned bit = (distance + WindowBitRotation) % kStrikeRegisterBits;
    const bool dupe = (0 != (SlidingWindow[bit / 64] & ((uint64_t)1 << (bit % 64))));
    return dupe;
}

void StrikeRegister::Accept(Counter64 sequence)
{
    // If this new nonce is the largest so far:
    if (LargestSequence < sequence) {
        LargestSequence = sequence;
    }

    // If we are before the sliding window:
    if (sequence < WindowBase)
    {
        SECURITY_DEBUG_BREAK(); // Invalid input
        return; // Already accepted
    }

    uint64_t distance = (sequence - WindowBase).ToUnsigned();

    // If it is too far ahead to be contained by the sliding window:
    if (distance >= kStrikeRegisterBits)
    {
        // Calculate number of words we will have to slide
        const unsigned slideWords = static_cast<unsigned>((distance - kStrikeRegisterBits) / 64 + 1);

        // If we would clear the whole window:
        if (slideWords >= kWordCount)
        {
            Reset(sequence);
            SlidingWindow[0] = 1;
            return;
        }

        // Clear words that are being reused from the front
        SECURITY_DEBUG_ASSERT(WindowBitRotation % 64 == 0);
        unsigned word = WindowBitRotation / 64;
        for (unsigned ii = 0; ii < slideWords; ++ii)
        {
            SECURITY_DEBUG_ASSERT(word < kWordCount);
            SlidingWindow[word] = 0;

            ++word;
            if (word >= kWordCount)
                word = 0;
        }

        // Move ahead the window base until it covers the new nonce
        const unsigned slideBits = slideWords * 64;
        WindowBase += slideBits;

        // Rotate window by slide count
        WindowBitRotation += slideBits;
        if (WindowBitRotation >= kStrikeRegisterBits)
            WindowBitRotation -= kStrikeRegisterBits;

        // Shift distance down
        SECURITY_DEBUG_ASSERT(distance >= slideBits);
        distance -= slideBits;
    }

    // Set bit for this packet
    const unsigned bit = (distance + WindowBitRotation) % kStrikeRegisterBits;
    SlidingWindow[bit / 64] |= ((uint64_t)1 << (bit % 64));
}

Counter64 StrikeRegister::Expand(uint64_t partial, unsigned bytes) const
{
    SECURITY_DEBUG_ASSERT(bytes > 0 && bytes < 8);
    return CounterExpand(LargestSequence.ToUnsigned(), partial, bytes);
}


} // namespace security
