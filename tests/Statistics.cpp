/** \file
    \brief WLANOptimzer Test: Statistics
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

#include "Statistics.h"
#include "tools/TimeSync.h"

namespace wopt {

static logger::Channel Logger("Statistics", wopt::kMinLogLevel);


//------------------------------------------------------------------------------
// FastLossIndicator

FastLossIndicator::FastLossIndicator()
{
    for (unsigned i = 0; i < kBinCount; ++i)
    {
        GotCount[i] = 0;
        ExpectedCount[i] = 0;
    }
}

bool FastLossIndicator::UpdateIsReordered(const DatagramInfo& datagram)
{
    const int64_t deltaNonce = (int64_t)(datagram.Nonce - NextExpectedNonce);

    // If datagram is reordered:
    if (deltaNonce < 0) {
        return true;
    }

    NextExpectedNonce = datagram.Nonce + 1;

    // If at least one loss occurred:
    if (deltaNonce >= 1)
    {
        // If two losses occurred:
        if (deltaNonce >= 2) {
            DoubleLoss = true;
        }
        else
        {
            // Negative values will wrap high here
            const uint64_t deltaLastLoss = (uint64_t)(datagram.Nonce - LastLossNonce);
            LastLossNonce = datagram.Nonce;

            // Count two single losses in a small span as a double loss
            if (deltaLastLoss < kDoubleLossThreshold) {
                DoubleLoss = true;
            }
        }
    }

    // Accumulate data
    GotCount[0]++;
    ExpectedCount[0] += (unsigned)deltaNonce + 1;

    // If interval has ended:
    if (GotCount[0] >= kMinCount &&
        (int64_t)(datagram.ReceiveUsec - IntervalStartUsec) >= kMinIntervalUsec)
    {
        IntervalStartUsec = datagram.ReceiveUsec;

        // Average over the last kBinCount bins
        unsigned got = GotCount[0], expected = ExpectedCount[0];
        for (unsigned i = kBinCount - 1; i > 0; --i)
        {
            got += GotCount[i];
            expected += ExpectedCount[i];

            GotCount[i] = GotCount[i - 1];
            ExpectedCount[i] = ExpectedCount[i - 1];
        }
        GotCount[0] = 0;
        ExpectedCount[0] = 0;

        // Update loss rate
        WOPT_DEBUG_ASSERT(got <= expected);
        const float lossRate = (expected - got) / (float)expected;
        if (HighestLossRate < lossRate) {
            HighestLossRate = lossRate;
        }
    }

    return false;
}

void FastLossIndicator::ResetStats()
{
    DoubleLoss = false;
    LastLossNonce = 0;
    HighestLossRate = 0.f;
}


//------------------------------------------------------------------------------
// InterPacketGapEstimator

void InterPacketGapEstimator::OnDatagram(uint64_t receiveUsec)
{
    // Calculate time since last datagram was received (processed)
    const int64_t delta = (int64_t)(receiveUsec - LastReceiveUsec);

    // If time is rolling backwards:
    if (delta <= 0) {
        WOPT_DEBUG_BREAK(); // Should not happen on any modern PC
        return;
    }

    // If there was not a huge period of silence:
    if (delta < (int64_t)kBWMaxIntervalUsec &&
        LastReceiveUsec != 0)
    {
        const unsigned ipgUsec = (unsigned)delta;

        // Update maximum seen in interval
        if (MaxUsec < ipgUsec) {
            MaxUsec = ipgUsec;
        }
    }

    LastReceiveUsec = receiveUsec;
}

void InterPacketGapEstimator::OnIntervalEnd()
{
    // If no interval was started:
    if (MaxUsec == 0) {
        return;
    }

    // Smooth the maximum values seen with EWMA
    SmoothedMaxUsec = (SmoothedMaxUsec * 7 + MaxUsec) / 8;

    // Indicate that no interval has started yet
    MaxUsec = 0;
}


//------------------------------------------------------------------------------
// BandwidthEstimator

bool BandwidthEstimator::UpdateOnDatagram(bool reordered, const DatagramInfo& datagram)
{
    // If we have not received any data yet:
    if (Start.Nonce == 0)
    {
        // Initialize the start to this datagram - We ignore its size because
        // there is no previous packet timestamp to use to measure bandwidth.
        Start = datagram;
        Previous = datagram;
        return true;
    }

    // Update IPG
    InterPacketGap.OnDatagram(datagram.ReceiveUsec);

    // If datagram was received out of order:
    if (reordered)
    {
        // Count it for this interval but do not use it to end an interval
        IntervalBytes += datagram.Bytes;
        ++IntervalDatagramCount;
        return false;
    }

    bool resultUpdated = false;

    // Find minimum trip time points:
    const unsigned tripUsec = datagram.NetworkTripUsec;
    if (SeekingMinimum)
    {
        // If trip time increased, the Previous datagram was a local minimum:
        if (tripUsec > Previous.NetworkTripUsec)
        {
            // Calculate receive interval
            const unsigned intervalUsec = (unsigned)(Previous.ReceiveUsec - Start.ReceiveUsec);
            WOPT_DEBUG_ASSERT((int64_t)(Previous.ReceiveUsec - Start.ReceiveUsec) >= 0);

            // Check that enough data was received since the interval started.
            // Check that the interval is long enough to avoid OS jitter.
            // Check that it exceeds the smoothed running maximum trip time,
            // which accounts for regular latency spikes experienced on WiFi
            if ((IntervalInSeqCount >= kMinBWIntervalDatagrams &&
                //IntervalBytes >= kMinBWIntervalBytes &&
                 intervalUsec >= kMinBWIntervalUsec &&
                 intervalUsec >= InterPacketGap.Get() * 2) ||
                (intervalUsec >= kBWMaxIntervalUsec &&
                 Previous.Nonce > Start.Nonce + 2)) // Or timeout
            {
                OnInterval(intervalUsec);
                resultUpdated = true; // Updated!
            }

            // Found the minimum, now start seeking a maximum
            SeekingMinimum = false;
            IntervalMaxTripUsec = tripUsec;
        }
    }
    else if (tripUsec < Previous.NetworkTripUsec) {
        SeekingMinimum = true;
    }
    else if (tripUsec > IntervalMaxTripUsec) {
        IntervalMaxTripUsec = tripUsec;
    }

    Previous = datagram;

    // Accumulate bytes following the first packet in the interval.
    // The packet that starts an interval belongs to the previous interval
    IntervalBytes += datagram.Bytes;
    ++IntervalInSeqCount;
    ++IntervalDatagramCount;

    return resultUpdated;
}

void BandwidthEstimator::OnInterval(unsigned intervalUsec)
{
    // Calculate receive BPS
    const uint64_t scaledBytes = (uint64_t)IntervalBytes * 1000000;
    const unsigned receiveBPS = intervalUsec > 0 ? (unsigned)(scaledBytes / intervalUsec) : 0;

    // Calculate send BPS
    const Counter24 sendIntervalTS24 = Previous.SendTS24 - Start.SendTS24;
    const unsigned sendIntervalUsec = sendIntervalTS24.ToUnsigned() << kTime23LostBits;
    LatestGoodputBPS = sendIntervalUsec > 0 ? (unsigned)(scaledBytes / sendIntervalUsec) : 0;

    // Calculate PLR
    const int expected = (int)(int64_t)(Previous.Nonce - Start.Nonce);
    const int lossCount = expected - (int)IntervalInSeqCount;
    WOPT_DEBUG_ASSERT(lossCount >= 0);

    if (expected > 0 &&
        lossCount > 0)
    {
        LatestPLR = (unsigned)lossCount / (float)expected;

        // Smooth the received PLR using EWMA
        if (SmoothedPLR == 0.f) {
            SmoothedPLR = LatestPLR;
        }
        else {
            SmoothedPLR = (SmoothedPLR * 7.f + LatestPLR) / 8.f;
        }

        // Guesstimate how much BW was lost to packetloss
        const uint64_t scaledMaybeBytes = (uint64_t)(scaledBytes * lossCount) / IntervalDatagramCount;
        GuessedLossBPS = sendIntervalUsec > 0 ? (unsigned)(scaledMaybeBytes / sendIntervalUsec) : 0;
    }
    else
    {
        LatestPLR = 0.f;
        SmoothedPLR = (SmoothedPLR * 7.f + LatestPLR) / 8.f;
        GuessedLossBPS = 0;
    }

    // Smooth the receive BPS using EWMA
    if (SmoothedGoodputBPS == 0) {
        SmoothedGoodputBPS = LatestGoodputBPS;
    }
    else {
        SmoothedGoodputBPS = (SmoothedGoodputBPS * 7 + LatestGoodputBPS) / 8;
    }

    // Smooth the minimum trip time estimate - Rough
    if (SmoothedMinTripUsec == 0) {
        SmoothedMinTripUsec = Previous.NetworkTripUsec;
    }
    else {
        SmoothedMinTripUsec = (SmoothedMinTripUsec * 7 + Previous.NetworkTripUsec) / 8;
    }

    // Smooth the maximum trip time estimate - Rough
    if (SmoothedMaxTripUsec == 0) {
        SmoothedMaxTripUsec = IntervalMaxTripUsec;
    }
    else {
        SmoothedMaxTripUsec = (SmoothedMaxTripUsec * 7 + IntervalMaxTripUsec) / 8;
    }

    // Update statistics
    InterPacketGap.OnIntervalEnd();

    // Reset interval sampling
    Start = Previous;
    IntervalBytes = 0;
    IntervalInSeqCount = 0;
}


} // namespace wopt
