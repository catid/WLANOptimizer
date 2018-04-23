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

#pragma once

#include "Counter.h"

namespace security {


//------------------------------------------------------------------------------
// StrikeRegister

/// Set the anti-replay buffer size to match the protocol re-ordering limit.
/// Must be a power-of-two.
static const unsigned kStrikeRegisterBits = 4096;

/**
    This class implements anti-replay protection and compression for nonces.

    Replay refers to tampering where datagrams are replayed.  The receiver must
    keep a record of datagrams it has received in order to avoid accepting the
    same datagram twice.  It is designed for nonce sequence numbers that start
    at 0 and increment by 1 for each datagram (reliable or unreliable).

    IsDuplicate() checks incoming datagram nonces to see if they have already
    been seen.  Accept() will slide the window as needed and strike the nonce.

    This implementation of sliding window anti-replay protection avoids actually
    shifting the bitfield window as an optimization.  Instead WindowBitRotation
    is incremented in place of a shift.

    StrikeRegister will also decompress incoming nonce values from 1-7 bytes
    back to the original 8 byte (64-bit) sequence number based on recent values.
*/
class StrikeRegister
{
public:
    COUNTER_FORCE_INLINE StrikeRegister()
    {
        Reset();
    }

    /// Reset to new base
    void Reset(Counter64 newBase = 0);

    /// Check if we have already received the given sequence number.
    /// Returns true if datagram should be ignored.
    bool IsDuplicate(Counter64 sequence) const;

    /// Accept the given sequence number
    void Accept(Counter64 sequence);

    /// Get next expected sequence from peer
    COUNTER_FORCE_INLINE Counter64 GetNextExpected() const
    {
        return LargestSequence + 1;
    }

    /// Expand sequence number to full size given a number of bytes from 1..8
    /// and the largest sequence number accepted so far.
    Counter64 Expand(uint64_t partial, unsigned bytes) const;

protected:
    /// Sliding window
    static const unsigned kWordCount = (kStrikeRegisterBits + 63) / 64;
    uint64_t SlidingWindow[kWordCount];

    /// Base offset for the sliding window
    /// Note: This does not need to be a power of two
    Counter64 WindowBase = 0;

    /// Bit offset in the sliding window where WindowBase starts
    /// Note: This will always be a multiple of 64
    unsigned WindowBitRotation = 0;

    /// Largest sequence accepted so far from peer
    Counter64 LargestSequence = 0;
};


} // namespace security
