/** \file
    \brief WLANOptimzer Test: Connection
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

#include "../WLANOptimizer.h"
#include "tools/TimeSync.h"
#include "tools/StrikeRegister.h"
#include "tools/CCatCpp.h"
#include "Statistics.h"
#include "Tools.h"

namespace wopt {


//------------------------------------------------------------------------------
// Constants

// Interval between switching WLANOptimizer on/off
static const uint64_t kSwitchIntervalUsec = 1000 * 1000 * 20; // 20 sec

// Interval between broadcasting Discovery packets on the LAN
static const uint64_t kDiscoverIntervalUsec = 1000 * 1000; // 1 sec

// Timeout
static const uint64_t kTimeoutUsec = 10 * 1000 * 1000; // 10 sec

// Server port
static const uint16_t kServerPort = 6969;

static const uint8_t kOpcodeFlag_PreConnect = 0x80;

// <opcode(1)> <protocol magic(4)> <send timestamp(3)> <GUID(8)>
static const unsigned kDiscoveryMessageBytes = 1 + 4 + 3 + 8;

static const uint32_t kDiscoverMagic = 0x69adde69;
static const uint8_t kOpcodeDiscovery = 0 | kOpcodeFlag_PreConnect;
static const uint8_t kOpcodeDiscoveryReply = 1 | kOpcodeFlag_PreConnect;

// <opcode(1)> <packet nonce(2)> <send timestamp(3)> <best min ts(3)> <original sequence number(2)>
static const unsigned kOverheadBytes = 1 + 2 + 3 + 3 + 2;

// <data id(2)> <WLANOptimizerEnabled(1)>
static const unsigned kOriginalBytes = 2 + 1;
static const uint8_t kOpcodeOriginal = 0;

// <recovery row(1)> <count(1)> <recovery data(x)>
static const unsigned kRecoveryOverheadBytes = 1 + 1;
static const uint8_t kOpcodeRecovery = 1;


//------------------------------------------------------------------------------
// Statistics

struct Statistics
{
    std::string Name;
    float Min = 0.f;
    float Max = 0.f;
    float Average = 0.f;
    float StandardDeviation = 0.f;
    float WirePLR = 0.f;
    float EffectivePLR = 0.f;
    float Percentiles[1 + 9 + 1]; // Min, 10% - 90%, Max
    bool WLANOptimizerEnabled = false;
};

class IStatisticsReceiver
{
public:
    virtual void OnStats(const Statistics& stats) = 0;
};

class StatisticsCalculator
{
public:
    StatisticsCalculator(IStatisticsReceiver* receiver)
        : Receiver(receiver)
    {
    }

    Statistics Get()
    {
        std::lock_guard<std::mutex> locker(Lock);
        return Last;
    }

    void SetName(const std::string& name)
    {
        std::lock_guard<std::mutex> locker(Lock);
        Name = name;
    }

    void OnDatagram(const DatagramInfo& datagram);
    void OnOriginalData(Counter64 sequence);

protected:
    IStatisticsReceiver* Receiver;

    std::mutex Lock;
    Statistics Last;
    std::string Name;

    FastLossIndicator FastLoss;
    BandwidthEstimator BWEstimator;

    std::vector<unsigned> OWDSamples;
    unsigned MinOWD = 0, MaxOWD = 0;

    // Sum of all samples for average
    uint64_t AvgSum = 0;

    // Variance using Welford method:
    // http://jonisalonen.com/2013/deriving-welfords-method-for-computing-variance/
    float WelfordS = 0.f, WelfordM = 0.f;

    Counter64 FirstDataSequence = 0;
    Counter64 LargestDataSequence = 0;
    unsigned DataReceiveCount = 0;


    unsigned GetPercentileOWD(float percentile);
};


//------------------------------------------------------------------------------
// WLANOptimizer Switcher

// This class synchronizes switching the WLANOptimizer enable setting on and off
// together with remote peers.  When a remote peer switches, we switch to their
// setting and reset the timeout.
class Switcher
{
public:
    Switcher();
    ~Switcher();

    void Switch();

    void Enable(bool enabled);

    void Join();

    uint64_t MyGuid = 0;

protected:
    std::mutex Lock;

    uint64_t LastSwitchUsec = 0;
    bool LastEnable = false;

    std::unique_ptr<std::thread> AsyncWorker;
};


//------------------------------------------------------------------------------
// PCG PRNG

/// From http://www.pcg-random.org/
class PCGRandom
{
public:
    void Seed(uint64_t y, uint64_t x = 0)
    {
        State = 0;
        Inc = (y << 1u) | 1u;
        Next();
        State += x;
        Next();
    }

    uint32_t Next()
    {
        const uint64_t oldstate = State;
        State = oldstate * UINT64_C(6364136223846793005) + Inc;
        const uint32_t xorshifted = (uint32_t)(((oldstate >> 18) ^ oldstate) >> 27);
        const uint32_t rot = oldstate >> 59;
        return (xorshifted >> rot) | (xorshifted << ((uint32_t)(-(int32_t)rot) & 31));
    }

    uint64_t State = 0, Inc = 0;
};


//------------------------------------------------------------------------------
// TestConnection

class TestConnection : public CauchyCaterpillar
{
public:
    TestConnection(
        Socket* socket,
        Switcher* switcher,
        IStatisticsReceiver* receiver)
        : SocketPtr(socket)
        , SwitcherPtr(switcher)
        , StatsCalc(receiver)
    {
        Initialize();
    }

    void OnConnect(
        const UDPAddress& addr,
        Counter24 remote_ts,
        uint64_t nowUsec,
        uint64_t guid);
    bool IsSource(const UDPAddress& addr);

    void OnTick(uint64_t nowUsec);
    bool IsExpired(uint64_t nowUsec);

    void OnRead(uint64_t receiveUsec, uint8_t* buffer, unsigned bytes);

protected:
    Socket* SocketPtr;
    Switcher* SwitcherPtr;

    UDPAddress PeerAddress;

    TimeSynchronizer TimeSync;

    security::StrikeRegister Strikes;

    uint64_t LastReceiveUsec = 0;

    uint64_t NextNonce = 0;
    uint64_t NextOriginalSequence = 0;

    uint64_t LargestIncomingOriginal = 0;

    uint16_t NextDataPiece = 0;

    uint64_t PeerGuid = 0;

    StatisticsCalculator StatsCalc;

    PCGRandom LossPrng;


    void WriteNextData();
    void WriteRecovery();

    void OnRecoveredData(const CCatOriginal& original) override;

    void OnOriginalData(uint64_t sequence, uint16_t id, bool enabled);
};


//------------------------------------------------------------------------------
// TestSocket

class TestSocket : public Socket
{
public:
    TestSocket(IStatisticsReceiver* receiver)
        : Receiver(receiver)
    {
    }

protected:
    IStatisticsReceiver* Receiver;

    // Connection list
    std::vector<std::shared_ptr<TestConnection> > Connections;

    uint64_t LastDiscoverUsec = 0;

    Switcher WoptSwitcher;


    // Only accepts IP because I'm too lazy to do a resolver
    void Discover(const char* ip = nullptr);

    void OnTick() override;
    void OnRead(
        const UDPAddress& addr,
        uint64_t receiveUsec,
        uint8_t* buffer,
        unsigned bytes) override;
};


} // namespace wopt
