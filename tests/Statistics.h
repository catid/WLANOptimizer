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

#pragma once

#include "Tools.h"
#include "tools/Counter.h"

namespace wopt {

typedef uint64_t NonceT;


//------------------------------------------------------------------------------
// Constants

/// Heuristic: Maximum interval length.
/// This is about 2.5x higher than WiFi lag spikes
static const uint32_t kBWMaxIntervalUsec = 325 * 1000; ///< 325 ms

/// Heuristic: Minimum bandwidth sample interval duration in datagrams
static const unsigned kMinBWIntervalDatagrams = 10; ///< 10 datagrams

/// Heuristic: Minimum bandwidth sample interval duration in bytes.
//static const unsigned kMinBWIntervalBytes = 5000; ///< 5KB

/// Heuristic: Minimum bandwidth sample interval duration in microseconds.
/// Based on some common parameters:
/// - 10 ms of OS jitter
/// - 10 ms cellular network jitter (so 2x this for two samples at least)
/// - 4 ms cellular network jitter is also common
static const uint32_t kMinBWIntervalUsec = 22 * 1000; ///< 22 ms


//------------------------------------------------------------------------------
// DatagramInfo

/// Describes a received datagram for the receiver's bandwidth control algorithm
struct DatagramInfo
{
    /// Nonce of the datagram.  Used to detect data received out of order
    NonceT Nonce = 0; ///< Incremented once per packet

    /// Estimated one-way-delay (OWD) for this datagram in microseconds.
    /// This does not include processing time only network delay and perhaps
    /// some delays from the Operating System when it is heavily loaded.
    /// Set to 0 if trip time is not available
    unsigned NetworkTripUsec = 0;

    /// Send timestamp of the datagram in TS24 units.
    /// This is used for calculating relative delay between packet pairs, and
    /// for calculating bandwidth used at the sender side.
    Counter24 SendTS24 = 0; ///< TS24 units

    /// Timestamp this datagram was received.  Maybe slightly behind real time.
    /// This is useful for checking relative delay between packet pairs, and for
    /// calculating bandwidth at the receiver side.
    uint64_t ReceiveUsec = 0; ///< Microseconds

    /// Number of data bytes in the datagram (including headers).
    /// This is used for calculating send and receive bandwidth.
    unsigned Bytes = 0; ///< Bytes
};


//------------------------------------------------------------------------------
// InterPacketGapEstimator

/// Statistics tracking inter-packet gap (IPG)
class InterPacketGapEstimator
{
public:
    /// Handle a datagram being received.
    /// It is okay if the datagram is re-ordered
    void OnDatagram(uint64_t receiveUsec);

    /// Handle an interval ending
    void OnIntervalEnd();

    /// Get the smoothed maximum IPG
    unsigned Get() const
    {
        return SmoothedMaxUsec;
    }

protected:
    /// Time that last in-sequence datagram was received
    uint64_t LastReceiveUsec = 0;

    /// Maximum inter-packet gap in the past interval
    unsigned MaxUsec = 0;

    /// Smoothed maximum inter-packet gap seen over time
    unsigned SmoothedMaxUsec = 0;
};


//------------------------------------------------------------------------------
// BandwidthEstimator

/**
    BandwidthEstimator

    The goal is to use as few packets as possible to estimate the received
    bandwidth, so that statistics can be gathered more quickly, leading to
    faster reaction times in bandwidth control.

    The smallest delay in an interval is used to represent that interval
    for network trip time.  The sum of the bytes over that interval
    is used to calculate instantaneous bandwidth.

    The theory of the instantaneous statistics estimator is that packets travel
    through the network and OS in small bursts due to all sorts of effects.
    The sender side sends in small bursts to avoid CPU-intensive precise packet
    release timing that would require burning a whole core.  The burst interval
    is small enough to avoid impacting latency (e.g. 5 millisecond bursts) and
    packets may be sent in-between if the application calls tonk_flush().
    Bursts end up being detectable via the one-way delay.

    When receiving data below the channel bandwidth limit:

    The burst delay patterns of the sender will be visible in the received
    packets, and we should carefully decide where to begin/end bins of packets
    for gathering statistics.  This is the main case the algorithm handles.
    When receiving data above/near the channel bandwidth limit:

    Packets will be received in a roughly consistent rate, so almost any range
    of packets will be good for estimating the received bandwidth and delay.

    Example rising-delay bursts:
                                x
          x          x        x x
        x x          x        x x
        x x        x x      x x x
      x x x      x x x    x x x x
    x x x x    x x x x    x x x x ...
    Time ------|==========|---------->

    We want to measure at the starts of these bursts.  This fully
    covers the entire timeline, and each burst represents the received
    bandwidth accurately.

    The first few packets in a rising-delay burst have lower delay,
    which increases to a maximum.  This type of burst is caused by
    sending a burst of packets close together, which spread out due
    to network router queues along the route.

    Operating system jitter and latency spikes on sender/receiver will
    cause artificial delay, which will flush over time back to a minimum
    delay.  We want to reject these incorrect signals.

    WiFi adapters will periodically scan for networks, causing periodic
    128 millisecond delays.  To account for that, the smoothed maximum
    inter-packet gap (IPG) is continuously calculated, and statistics
    collection must wait for at least IPG * 2 to elapse.

    We collect data over an interval until:
    + At least 22 milliseconds has elapsed, to avoid OS/network jitter issues.
    We do not wait for RTT or OWD based delays because it may be a long link.
    + At least 10 packets have been received, for statistical significance.
    + At least 5000 bytes have been received, for minimum bandwidth.
    At low rates we wait at least 250 ms or at least the OWD (whichever is
    smaller) and for at least two packets.
*/
class BandwidthEstimator
{
public:
    /// Update on datagram received.
    /// Returns true if statistics were updated
    bool UpdateOnDatagram(bool reordered, const DatagramInfo& datagram);

    /// Returns BPS that we suspect the peer was sending in the last interval
    unsigned GetAttemptedBPS() const
    {
        return LatestGoodputBPS + GuessedLossBPS;
    }

    /// Smoothed estimate of goodput
    unsigned SmoothedGoodputBPS = 0;

    /// Pretty accurate measure of BPS the sender is achieving over a small
    /// recent time interval.  If there is packetloss, this is a smaller rate
    /// than the sender actually sent.  GetAttemptedBPS() can be used to
    /// estimate the original send rate during loss.
    unsigned LatestGoodputBPS = 0;

    /// Guessed BW before packetloss
    unsigned GuessedLossBPS = 0;

    /// Smoothed estimate of maximum trip time
    unsigned SmoothedMaxTripUsec = 0;

    /// Smoothed estimate of minimum trip time
    unsigned SmoothedMinTripUsec = 0;

    /// PLR
    float LatestPLR = 0.f;

    /// Smoothed PLR
    float SmoothedPLR = 0.f;

    /// Get maximum recently seen PLR
    float GetMaxPLR() const
    {
        return LatestPLR > SmoothedPLR ? LatestPLR : SmoothedPLR;
    }

protected:
    /// Interval start
    DatagramInfo Start;

    /// Seeking a minimum value
    bool SeekingMinimum = true;

    /// Previous in-order datagram info
    DatagramInfo Previous;

    /// Bytes before Minimum
    unsigned IntervalBytes = 0;

    /// Datagram count in interval
    unsigned IntervalDatagramCount = 0;

    /// Count before Minimum
    unsigned IntervalInSeqCount = 0;

    /// Maximum seen so far in this interval
    unsigned IntervalMaxTripUsec = 0;

    /// Statistics tracking inter-packet gap (IPG)
    InterPacketGapEstimator InterPacketGap;


    /// Called when interval ends
    void OnInterval(unsigned intervalUsec);
};


//------------------------------------------------------------------------------
// FastLossIndicator

/**
    FastLossIndicator

    For low/mid-rate streams, waiting for the statistics collection
    interval to end would take a while.  For example on WiFi there are
    large 100 ms lag spikes even though the delay may be 5 milliseconds.
    The BW statistics need to wait for those spikes, so it takes much
    longer to react to changes in the network, as a result.

    To react faster to congestion signaled by packetloss, a fast loss
    indicator used:

    (1) For low-rate data it checks if 2 datagrams have been lost or
    reordered within a span of 20 datagrams.  Or 3 datagrams in 10.
    This handles the general case of low-rate data where statistics
    are too slow to react.

    (2) For high-rate data it checks if more than 10% loss is detected
    within a span of 22 milliseconds.  This handles the case of links
    with high delay variance like WiFi, allowing Tonk to reduce its
    send rate faster than the statistics collection interval, which
    can be a quarter of a second in that case.
*/
class FastLossIndicator
{
public:
    FastLossIndicator();

    /// Update on datagram received.
    /// Returns true if datagram is out of sequence (reordered)
    bool UpdateIsReordered(const DatagramInfo& datagram);

    /// Reset the stats
    void ResetStats();

    /// Double loss event seen since the last Reset()?
    bool DoubleLoss = false;

    /// Set to the highest loss rate seen since reset
    float HighestLossRate = 0.f;

protected:
    /// Next nonce expected from peer
    NonceT NextExpectedNonce = 0;

    /// Last loss nonce
    NonceT LastLossNonce = 0;

    /// Two single losses within this threshold distance
    /// will also count as a DoubleLoss.
    static const uint64_t kDoubleLossThreshold = 10;

    /// Minimum interval before reporting a loss rate
    static const unsigned kMinIntervalUsec = 22 * 1000; ///< 22 ms

    /// Minimum count of datagrams received before reporting a loss
    static const unsigned kMinCount = 20;

    /// Start of interval in microseconds
    uint64_t IntervalStartUsec = 0;

    /// Number of bins to average over
    static const unsigned kBinCount = 2;

    /// Received count for each bin
    unsigned GotCount[kBinCount];

    /// Expected count of each bin
    unsigned ExpectedCount[kBinCount];
};


} // namespace wopt
