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

#include "Connection.h"

#include <algorithm>

namespace wopt {

static logger::Channel Logger("Connection", wopt::kMinLogLevel);


//------------------------------------------------------------------------------
// WLANOptimizer Switcher

static std::atomic<bool> WoptEnabled = false;

Switcher::Switcher()
{
    MyGuid = GetTimeUsec();
}

Switcher::~Switcher()
{
    Join();
}

void Switcher::Switch()
{
    const uint64_t nowUsec = GetTimeUsec();
    if (nowUsec - LastSwitchUsec < kSwitchIntervalUsec) {
        return;
    }
    Enable(!WoptEnabled);
}

void Switcher::Join()
{
    if (AsyncWorker) {
        if (AsyncWorker->joinable()) {
            AsyncWorker->join();
        }
        AsyncWorker = nullptr;
    }
}

void Switcher::Enable(bool enabled)
{
    if (WoptEnabled == enabled) {
        return;
    }

    Join();

    const uint64_t nowUsec = GetTimeUsec();
    LastSwitchUsec = nowUsec;

    AsyncWorker = MakeUniqueNoThrow<std::thread>([enabled] {
        const int optimizeResult = OptimizeWLAN(enabled ? 1 : 0);

        if (optimizeResult == OptimizeWLAN_Success) {
            Logger.Info("*** Toggled WLANOptimizer enabled = ", enabled);
        }
        else if (optimizeResult == OptimizeWLAN_NoConnections) {
            Logger.Info("*** No Wireless LAN connections detected to optimize.");
        }
        else {
            Logger.Info("*** WLANOptimizer failed with error code: ", optimizeResult);
        }

        WoptEnabled = enabled;
    });
}


//------------------------------------------------------------------------------
// Statistics

unsigned StatisticsCalculator::GetPercentileOWD(float percentile)
{
    const size_t count = OWDSamples.size();

    using offset_t = std::vector<unsigned>::size_type;

    offset_t goalOffset = (offset_t)(percentile * count);

    std::nth_element(OWDSamples.begin(), OWDSamples.begin() + goalOffset, OWDSamples.begin() + count,
        [](unsigned a, unsigned b)->bool {
        return a < b;
    });

    return OWDSamples[goalOffset];
}

void StatisticsCalculator::AddOWDSample(unsigned owdUsec)
{
    if (OWDSamples.empty())
    {
        MinOWD = MaxOWD = owdUsec;
    }
    else
    {
        if (MinOWD > owdUsec) {
            MinOWD = owdUsec;
        }
        if (MaxOWD < owdUsec) {
            MaxOWD = owdUsec;
        }
    }

    OWDSamples.push_back(owdUsec);

    if (OWDSamples.size() > 100)
    {
        std::lock_guard<std::mutex> locker(Lock);

        Last.Name = Name;

        Last.Max = MaxOWD / 1000.f;
        Last.Min = MinOWD / 1000.f;

        Last.Percentiles[0] = MinOWD / 1000.f;
        Last.Percentiles[10] = MaxOWD / 1000.f;
        for (int i = 1; i < 10; ++i)
        {
            Last.Percentiles[i] = GetPercentileOWD(i / 10.f) / 1000.f;
        }

        Last.WLANOptimizerEnabled = WoptEnabled;

        Receiver->OnStats(Last);

        OWDSamples.clear();
    }
}


//------------------------------------------------------------------------------
// TestConnection

void TestConnection::OnConnect(
    const UDPAddress& addr,
    Counter24 remote_ts,
    uint64_t nowUsec,
    uint64_t guid)
{
    LastReceiveUsec = nowUsec;
    PeerAddress = addr;
    PeerGuid = guid;

    TimeSync.OnAuthenticatedDatagramTimestamp(remote_ts, nowUsec);
}

bool TestConnection::IsSource(const UDPAddress& addr)
{
    return PeerAddress == addr;
}

void TestConnection::OnTick(uint64_t nowUsec)
{
    WriteNextData();

    if (OriginalSequence % 4 == 0) {
        WriteRecovery();
    }

    std::string name = PeerAddress.address().to_string() + ":" + std::to_string(PeerAddress.port());
    std::string title = name;
    title += " (WLANOptimizer ";
    if (WoptEnabled)
    {
        title += "ENABLED)";
    }
    else
    {
        title += "DISABLED)";
    }
    StatsCalc.SetName(title);
}

bool TestConnection::IsExpired(uint64_t nowUsec)
{
    return (nowUsec - LastReceiveUsec > kTimeoutUsec);
}

void TestConnection::WriteNextData()
{
    uint8_t data[kOverheadBytes + kOriginalBytes];

    // Write header
    data[0] = kOpcodeOriginal;
    WriteU16_LE(data + 1, (uint16_t)NextOutgoingSequence);
    WriteU24_LE(data + 3, TimeSync.LocalTimeToDatagramTS24(GetTimeUsec()));
    WriteU24_LE(data + 6, TimeSync.GetMinDeltaTS24().ToUnsigned());
    ++NextOutgoingSequence;

    // Write FEC header
    WriteU16_LE(data + 9, (uint16_t)OriginalSequence);

    // Write data
    WriteU16_LE(data + 11, NextDataPiece);
    data[13] = WoptEnabled ? 1 : 0;
    ++NextDataPiece;

    CCatOriginal original;
    original.Bytes = 2;
    original.Data = data + 11;
    original.SequenceNumber = OriginalSequence;
    ++OriginalSequence;

    SocketPtr->Send(PeerAddress, data, kOverheadBytes + kOriginalBytes);

    SendOriginal(original);
}

void TestConnection::WriteRecovery()
{
    CCatRecovery recovery;

    if (!SendRecovery(recovery) || recovery.Bytes > 128) {
        return;
    }

    uint8_t data[kOverheadBytes + kRecoveryOverheadBytes + 128];

    // Write header
    data[0] = kOpcodeRecovery;
    WriteU16_LE(data + 1, (uint16_t)NextOutgoingSequence);
    WriteU24_LE(data + 3, TimeSync.LocalTimeToDatagramTS24(GetTimeUsec()));
    WriteU24_LE(data + 6, TimeSync.GetMinDeltaTS24().ToUnsigned());
    ++NextOutgoingSequence;

    // Write recovery header
    data[9] = recovery.RecoveryRow;
    data[10] = recovery.Count;
    WriteU16_LE(data + 11, (uint16_t)recovery.SequenceStart);

    // Write recovery piece
    memcpy(data + 13, recovery.Data, recovery.Bytes);

    SocketPtr->Send(PeerAddress, data, kOverheadBytes + kRecoveryOverheadBytes + recovery.Bytes);
}

void TestConnection::OnRecoveredData(const CCatOriginal& original)
{
    if (original.Bytes >= 3)
    {
        uint16_t data_id = ReadU16_LE(original.Data);
        OnData(data_id, original.Data[2] != 0);
    }
}

void TestConnection::OnData(uint16_t id, bool enabled)
{
    //Logger.Info("Got data ", id, " enabled=", enabled);

    // Allow the peer with the larger GUID to win so we don't fight for control
    // of the WLANOptimizer enable switch.  We want both sides to turn it on/off
    // around the same time so that the effects can be seen when both sides turn
    // on the WLANOptimizer.
    if (SwitcherPtr->MyGuid < PeerGuid)
    {
        SwitcherPtr->Enable(enabled);
    }
}

void TestConnection::OnRead(uint64_t receiveUsec, uint8_t* data, unsigned bytes)
{
    //Logger.Info("Read: ", bytes);

    if (bytes < kOverheadBytes)
    {
        //Logger.Warning("Ignoring discovery/bogon");
        return;
    }

    if (0 != (data[0] & kOpcodeFlag_PreConnect)) {
        //Logger.Warning("Ignoring discovery/bogon");
        return;
    }

    Counter64 sequence = Strikes.Expand(ReadU16_LE(data + 1), 2);

    if (Strikes.IsDuplicate(sequence)) {
        Logger.Warning("Strikes.IsDuplicate ", sequence.ToUnsigned());
        return;
    }

    //Logger.Warning("ACCEPT ", sequence.ToUnsigned());

    LastReceiveUsec = receiveUsec;
    Strikes.Accept(sequence);

    Counter24 datagramTimestamp = ReadU24_LE(data + 3);
    Counter24 peerMinDeltaTS24 = ReadU24_LE(data + 6);

    TimeSync.OnPeerMinDeltaTS24(peerMinDeltaTS24);

    unsigned owdUsec = TimeSync.OnAuthenticatedDatagramTimestamp(datagramTimestamp, receiveUsec);

    if (owdUsec > 0) {
        StatsCalc.AddOWDSample(owdUsec);
    }

    const uint8_t opcode = data[0];

    if (opcode == kOpcodeOriginal &&
        bytes == kOverheadBytes + kOriginalBytes)
    {
        Counter64 recoveredSequenceStart = CounterExpand(
            LargestIncomingOriginal,
            ReadU16_LE(data + kOverheadBytes), 2);

        if (recoveredSequenceStart > LargestIncomingOriginal) {
            LargestIncomingOriginal = recoveredSequenceStart.ToUnsigned();
        }

        uint8_t* payloadData = data + kOverheadBytes + 2;
        unsigned payloadBytes = bytes - (kOverheadBytes + 2);

        CCatOriginal original;
        original.Bytes = payloadBytes;
        original.Data = payloadData;
        original.SequenceNumber = recoveredSequenceStart.ToUnsigned();

        OnData(ReadU16_LE(payloadData), payloadData[2] != 0);

        CauchyCaterpillar::OnOriginal(original);
    }
    else if (opcode == kOpcodeRecovery &&
        bytes > kOverheadBytes + kRecoveryOverheadBytes)
    {
        Counter64 recoveredSequenceStart = CounterExpand(
            LargestIncomingOriginal,
            ReadU16_LE(data + kOverheadBytes + 2), 2);

        if (recoveredSequenceStart > LargestIncomingOriginal) {
            LargestIncomingOriginal = recoveredSequenceStart.ToUnsigned();
        }

        CCatRecovery recovery;
        recovery.Data = data + kOverheadBytes + kRecoveryOverheadBytes;
        recovery.Bytes = bytes - (kOverheadBytes + kRecoveryOverheadBytes);
        recovery.RecoveryRow = data[kOverheadBytes];
        recovery.Count = data[kOverheadBytes + 1];
        recovery.SequenceStart = recoveredSequenceStart.ToUnsigned();

        CauchyCaterpillar::OnRecovery(recovery);
    }
}


//------------------------------------------------------------------------------
// TestSocket

void TestSocket::Discover(const char* ip)
{
    asio::ip::address ipAddr;

    if (!ip)
    {
        ipAddr = asio::ip::address_v4::broadcast();
    }
    else
    {
        // Convert IP string to IP address
        asio::error_code ec;
        ipAddr = asio::ip::make_address(ip, ec);
        if (ec) {
            WOPT_DEBUG_BREAK();
            Logger.Error("make_address error: ", ec.message());
            return;
        }
    }

    uint8_t data[kDiscoveryMessageBytes];
    data[0] = kOpcodeDiscovery;
    WriteU32_LE(data + 1, kDiscoverMagic);

    // Attach send timestamp
    WriteU24_LE(data + 5, TimeSynchronizer::LocalTimeToDatagramTS24(GetTimeUsec()));
    WriteU64_LE(data + 8, WoptSwitcher.MyGuid);

    UDPAddress addr(ipAddr, kServerPort);
    Send(addr, data, kDiscoveryMessageBytes);

    //Logger.Debug("Sending Discover to ", addr.address().to_string());
}

void TestSocket::OnRead(
    const wopt::UDPAddress& addr,
    uint64_t receiveUsec,
    uint8_t* data,
    unsigned bytes)
{
    // Handle packets from an established connection:
    for (auto& connection : Connections) {
        if (connection->IsSource(addr)) {
            connection->OnRead(receiveUsec, data, bytes);
            return;
        }
    }

    if (bytes != kDiscoveryMessageBytes) {
        return;
    }

    uint8_t opcode = data[0];
    if (opcode != kOpcodeDiscovery &&
        opcode != kOpcodeDiscoveryReply)
    {
        return;
    }

    uint32_t magic = ReadU32_LE(data + 1);
    if (magic != kDiscoverMagic) {
        return;
    }

    uint64_t guid = ReadU64_LE(data + 8);
    if (guid == WoptSwitcher.MyGuid) {
        return;
    }

    Counter24 remote_ts = ReadU24_LE(data + 5);

    // Make a new TestConnection in the list of connections
    std::shared_ptr<TestConnection> ptr = MakeSharedNoThrow<TestConnection>(this, &WoptSwitcher, Receiver);
    Connections.push_back(ptr);

    // Initialize the connection
    ptr->OnConnect(addr, remote_ts, receiveUsec, guid);

    // If this was a discovery packet:
    if (opcode == kOpcodeDiscovery)
    {
        // Respond with a reply so we won't cause an infinite ping-pong loop
        data[0] = kOpcodeDiscoveryReply;

        // Attach send timestamp
        WriteU24_LE(data + 5, TimeSynchronizer::LocalTimeToDatagramTS24(GetTimeUsec()));
        WriteU64_LE(data + 8, WoptSwitcher.MyGuid);

        // Reply to sender
        Send(addr, data, kDiscoveryMessageBytes);

        Logger.Debug("Received Discover from ", addr.address().to_string(), " : ", addr.port());
    }
    else
    {
        Logger.Debug("Received Discover Reply from ", addr.address().to_string());
    }
}

void TestSocket::OnTick()
{
    const uint64_t nowUsec = GetTimeUsec();

    int serverCount = (int)Connections.size();
    for (int i = 0; i < serverCount; ++i)
    {
        if (Connections[i]->IsExpired(nowUsec))
        {
            Logger.Warning("Connection timeout");
            Connections.erase(Connections.begin() + i);
            --i;
            --serverCount;
            continue;
        }

        Connections[i]->OnTick(nowUsec);
    }

    if (nowUsec - LastDiscoverUsec > kDiscoverIntervalUsec)
    {
        LastDiscoverUsec = nowUsec;
        Discover();
    }

    WoptSwitcher.Switch();
}


} // namespace wopt
