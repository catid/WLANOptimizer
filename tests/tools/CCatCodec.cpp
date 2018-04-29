/** \file
    \brief Cauchy Caterpillar : Codec Implementation
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

#include "CCatCodec.h"

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#elif __MACH__
    #include <mach/mach_time.h>
    #include <mach/mach.h>
    #include <mach/clock.h>

    extern mach_port_t clock_port;
#else
    #include <time.h>
    #include <sys/time.h>
#endif

namespace ccat {


//------------------------------------------------------------------------------
// AlignedLightVector

bool AlignedLightVector::Resize(
    pktalloc::Allocator* allocPtr,
    unsigned elements,
    pktalloc::Realloc behavior)
{
    PKTALLOC_DEBUG_ASSERT(Size <= Allocated);

    if (elements > Allocated)
    {
        Allocated = elements;
        DataPtr = allocPtr->Reallocate(DataPtr, elements, behavior);
        if (!DataPtr)
            return false;
    }

    Size = elements;
    return true;
}


//------------------------------------------------------------------------------
// Timing

#ifdef _WIN32
// Precomputed frequency inverse
static double PerfFrequencyInverseUsec = 0.;
static double PerfFrequencyInverseMsec = 0.;

static void InitPerfFrequencyInverse()
{
    LARGE_INTEGER freq = {};
    if (!::QueryPerformanceFrequency(&freq) || freq.QuadPart == 0) {
        return;
    }
    const double invFreq = 1. / (double)freq.QuadPart;
    PerfFrequencyInverseUsec = 1000000. * invFreq;
    PerfFrequencyInverseMsec = 1000. * invFreq;
    PKTALLOC_DEBUG_ASSERT(PerfFrequencyInverseUsec > 0.);
    PKTALLOC_DEBUG_ASSERT(PerfFrequencyInverseMsec > 0.);
}
#elif __MACH__
static bool m_clock_serv_init = false;
static clock_serv_t m_clock_serv = 0;

static void InitClockServ()
{
    m_clock_serv_init = true;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &m_clock_serv);
}
#endif // _WIN32

uint64_t GetTimeUsec()
{
#ifdef _WIN32
    LARGE_INTEGER timeStamp = {};
    if (!::QueryPerformanceCounter(&timeStamp)) {
        return 0;
    }
    if (PerfFrequencyInverseUsec == 0.) {
        InitPerfFrequencyInverse();
    }
    return (uint64_t)(PerfFrequencyInverseUsec * timeStamp.QuadPart);
#elif __MACH__
    if (!m_clock_serv_init) {
        InitClockServ();
    }

    mach_timespec_t tv;
    clock_get_time(m_clock_serv, &tv);

    return 1000000 * tv.tv_sec + tv.tv_nsec / 1000;
#else
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return 1000000 * tv.tv_sec + tv.tv_usec;
#endif
}

uint64_t GetTimeMsec()
{
#ifdef _WIN32
    LARGE_INTEGER timeStamp = {};
    if (!::QueryPerformanceCounter(&timeStamp)) {
        return 0;
    }
    if (PerfFrequencyInverseMsec == 0.) {
        InitPerfFrequencyInverse();
    }
    return (uint64_t)(PerfFrequencyInverseMsec * timeStamp.QuadPart);
#else
    // TBD: Optimize this?
    return GetTimeUsec() / 1000;
#endif
}


//------------------------------------------------------------------------------
// Codec : Create

CCatResult Codec::Create(const CCatSettings& settings)
{
    Settings = settings;
    Encoder::SettingsPtr = &Settings;
    Encoder::AllocPtr = &Alloc;
    Decoder::SettingsPtr = &Settings;
    Decoder::AllocPtr = &Alloc;

    if (Settings.WindowPackets < kMinEncoderWindowSize) {
        Settings.WindowPackets = kMinEncoderWindowSize;
    }
    if (Settings.WindowPackets > kMaxEncoderWindowSize) {
        Settings.WindowPackets = kMaxEncoderWindowSize;
    }

    if (Settings.WindowMsec < kMinWindowMsec) {
        Settings.WindowMsec = kMinWindowMsec;
    }
    if (Settings.WindowMsec > kMaxWindowMsec) {
        Settings.WindowMsec = kMaxWindowMsec;
    }

    return CCat_Success;
}


//------------------------------------------------------------------------------
// Encoder

CCatResult Encoder::EncodeOriginal(const CCatOriginal& original)
{
    // Validate input
    if (!original.Data ||
        original.Bytes <= 0 ||
        original.Bytes > kMaxPacketSize ||
        NextSequence != original.SequenceNumber)
    {
        PKTALLOC_DEBUG_BREAK();
        return CCat_InvalidInput;
    }

    // Pick window element
    EncoderWindowElement* element = &Window[NextIndex];
    if (++NextIndex >= kMaxEncoderWindowSize) {
        NextIndex = 0;
    }

    // Resize the target window element 
    const bool resizeResult = element->Data.Resize(
        AllocPtr,
        kEncodeOverhead + original.Bytes,
        pktalloc::Realloc::Uninitialized);
    if (!resizeResult) {
        return CCat_OOM;
    }

    // Write element data
    uint8_t* data = element->Data.GetPtr();
    memcpy(data + 2, original.Data, original.Bytes);
    WriteU16_LE(data, (uint16_t)(original.Bytes - 1));
    static_assert(kEncodeOverhead == 2, "Update this");

    // Record packet send time to expire old data from window
    const uint64_t nowUsec = GetTimeUsec();
    LastOriginalSendUsec = nowUsec;

    // Fill in element metadata
    element->SendUsec = nowUsec;

    // Keep track of count of packets stored
    if (Count < kMaxEncoderWindowSize) {
        ++Count;
    }

    // Update next column to assign
    if (++NextColumn >= kMatrixColumnCount) {
        NextColumn = 0;
    }

    // Update next sequence number
    ++NextSequence;

    return CCat_Success;
}

/**
    Encode algorithm:

    (1) Find the window of packets to include, based on the settings for
    max window size in packets and time.  This also provides the maximum
    size of the packet data so the recovery packet only needs to be
    allocated once.  It also enables us to check if this is an xor parity
    row ahead of doing any operations on the packet data.

    (2) Run forward through the encode window, and xor or muladd the
    original packet data into the recovery packet output.
*/
CCatResult Encoder::EncodeRecovery(CCatRecovery& recoveryOut)
{
    // Step (1): Find the set of packets to encode.

    const unsigned limitUsec = SettingsPtr->WindowMsec * 1000;
    const unsigned windowSize = SettingsPtr->WindowPackets;

    uint8_t column = NextColumn;
    unsigned index = NextIndex;
    unsigned maxBytes = 0;

    // Walk backward from the last written original packet:

    unsigned count = 0;
    while (count < Count)
    {
        // Iterate backwards
        unsigned prevIndex = index;
        if (prevIndex == 0) {
            prevIndex = kMaxEncoderWindowSize;
        }
        prevIndex--;

        EncoderWindowElement* element = &Window[prevIndex];
        const uint64_t deltaUsec = (uint64_t)(LastOriginalSendUsec - element->SendUsec).ToUnsigned();
        PKTALLOC_DEBUG_ASSERT(deltaUsec >= 0);

        // If packet is too old:
        if (deltaUsec > limitUsec) {
            break; // Stop here
        }

        // Include this packet
        index = prevIndex;
        if (column == 0) {
            column = kMatrixColumnCount;
        }
        column--;
        ++count;
        if (maxBytes < element->Data.GetSize()) {
            maxBytes = element->Data.GetSize();
        }

        // If window filled up:
        if (count >= windowSize) {
            break; // Stop here
        }
    }

    // If window is empty:
    if (count == 0)
    {
        recoveryOut.Data = nullptr;
        recoveryOut.Count = 0;
        recoveryOut.SequenceStart = 0;
        recoveryOut.Bytes = 0;
        recoveryOut.RecoveryRow = 0;

        return CCat_NeedsMoreData;
    }

    PKTALLOC_DEBUG_ASSERT(count <= CCAT_MAX_WINDOW_PACKETS);

    // Step (2): Write recovery packet

    PKTALLOC_DEBUG_ASSERT(NextSequence >= count);
    const Counter64 sequenceStart = NextSequence - count;

    // Handle 1x1 case by referencing the original data
    if (count == 1)
    {
        EncoderWindowElement* element = &Window[index];
        recoveryOut.Data = element->Data.GetPtr();
        recoveryOut.Count = 1;
        recoveryOut.SequenceStart = sequenceStart.ToUnsigned();
        recoveryOut.Bytes = element->Data.GetSize();
        recoveryOut.RecoveryRow = 0;

        return CCat_Success;
    }

    // Make space for the largest packet
    PKTALLOC_DEBUG_ASSERT(maxBytes > 0);
    RecoveryData.Resize(AllocPtr, maxBytes, pktalloc::Realloc::Uninitialized);
    uint8_t* output = RecoveryData.GetPtr();

    // This will reduce recovery rates but improves speed
#ifdef CCAT_MORE_PARITY_ROWS

    // Check if this is an xor parity row
    const bool isParityRow = (sequenceStart >= NextParitySequence);

    // Write metadata
    uint8_t row = 0;
    if (isParityRow) {
        NextParitySequence = sequenceStart + count;
    }
    else
    {
        row = NextRow;
        if (++NextRow >= kMatrixRowCount) {
            NextRow = 1;
        }
    }

#else // CCAT_MORE_PARITY_ROWS

    // Write metadata
    uint8_t row = 0;
    row = NextRow;
    if (++NextRow >= kMatrixRowCount) {
        NextRow = 0;
    }
    bool isParityRow = (row == 0);

#endif // CCAT_MORE_PARITY_ROWS

    recoveryOut.Data = output;
    recoveryOut.Count = static_cast<uint8_t>(count);
    recoveryOut.SequenceStart = sequenceStart.ToUnsigned();
    recoveryOut.Bytes = maxBytes;
    recoveryOut.RecoveryRow = row;

    // Unroll first column:
    {
        EncoderWindowElement* element = &Window[index];
        PKTALLOC_DEBUG_ASSERT(element->Data.GetSize() > 2);
        const uint8_t* data = element->Data.GetPtr();
        const unsigned dataBytes = element->Data.GetSize();

        // Write column
        if (isParityRow) {
            memcpy(output, data, dataBytes);
        }
        else
        {
            const uint8_t y = GetMatrixElement(row, column);

            gf256_mul_mem(output, data, y, dataBytes);
        }

        // Pad with zeros
        PKTALLOC_DEBUG_ASSERT(maxBytes >= dataBytes);
        memset(output + dataBytes, 0, maxBytes - dataBytes);
    }

    // For each remaining column:
    while (--count > 0)
    {
        if (++index >= kMaxEncoderWindowSize) {
            index = 0;
        }

        EncoderWindowElement* element = &Window[index];
        PKTALLOC_DEBUG_ASSERT(element->Data.GetSize() > 2);
        const uint8_t* data = element->Data.GetPtr();
        const unsigned dataBytes = element->Data.GetSize();

        // Write column
        if (isParityRow) {
            gf256_add_mem(output, data, dataBytes);
        }
        else
        {
            if (++column >= kMatrixColumnCount) {
                column = 0;
            }

            const uint8_t y = GetMatrixElement(row, column);

            gf256_muladd_mem(output, y, data, dataBytes);
        }
    }

    return CCat_Success;
}


//------------------------------------------------------------------------------
// Decoder

CCatResult Decoder::DecodeRecovery(const CCatRecovery& recovery)
{
    // Expand window based on recovery span.  If the recovery packet includes some
    // data that was lost, this will expand the window to the right
    const Expand expandResult = ExpandWindow(recovery.SequenceStart, recovery.Count);

    // If the recovery packet is not useful:
    if (expandResult == Expand::OutOfWindow) {
        return CCat_Success;
    }

    // If window was shifted by this recovery packet:
    if (expandResult == Expand::Shifted) {
        // It may have removed some older recovery packets that referenced
        // that data, so remove those from the list
        CleanupRecoveryList();
    }

    // Check how many lost packets are covered by this recovery packet
    const Counter64 sequenceStart = recovery.SequenceStart;
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase); // Should never happen
    const Counter64 sequenceEnd = recovery.SequenceStart + recovery.Count;
    const unsigned lost = GetLostInRange(sequenceStart, sequenceEnd);

    // If this packet covers no losses:
    if (0 == lost) {
        // Cannot help so ignore it
        return CCat_Success;
    }

    // If one lost packet can be recovered:
    if (1 == lost) {
        // This is the most common recovery scenario, so it is handled specially
        return SolveLostOne(recovery);
    }

    // Store recovery packet in the sorted list
    CCatResult result = StoreRecovery(recovery);
    if (result != CCat_Success) {
        return result;
    }

    // Check if we can recover any losses using multiple recovery packets
    return FindSolutions();
}

CCatResult Decoder::DecodeOriginal(const CCatOriginal& original)
{
    CCatResult result = CCat_Success;

    switch (ExpandWindow(original.SequenceNumber))
    {
    case Expand::Evacuated:
        // Store this original in the window
        result = StoreOriginal(original);

        // Recovery packets not available so solutions cannot be attempted
        break;

    case Expand::Shifted:
        // Clean up recovery list
        CleanupRecoveryList();

        // Store this original in the window
        result = StoreOriginal(original);

        // Shifted window with this original so no recovery packets reference it
        break;

    case Expand::InWindow:
        // Original received out of order: Store this original in the window
        result = StoreOriginal(original);
        if (result != CCat_Success) {
            return result;
        }

        // Look for a solution containing this original packet
        return FindSolutionsContaining(original.SequenceNumber);

    case Expand::OutOfWindow:
    default:
        // Original was out of the window so ignore it
        break;
    }

    return result;
}

Decoder::Expand Decoder::ExpandWindow(Counter64 sequenceStart, unsigned count)
{
    /*
        The extent of the recovery packet span often indicates a lost original
        on the leading edge, when a recovery packet follows a loss.  This lost
        original is outside of the current loss window, so we must shift the
        loss window to include it.
    */

    // If span contains data rolled out of the window:
    if (sequenceStart < SequenceBase || count > kDecoderWindowSize) {
        return Expand::OutOfWindow;
    }

    // If span only references data within the existing window:
    const Counter64 sequenceEnd = sequenceStart + count;
    if (sequenceEnd <= SequenceEnd) {
        return Expand::InWindow;
    }

    // Window sequence end is moved ahead
    SequenceEnd = sequenceEnd;
    const uint64_t span = (sequenceEnd - SequenceBase).ToUnsigned();

    // If the window does not need to be shifted:
    if (span <= kDecoderWindowSize) {
        return Expand::InWindow;
    }

    // If entire window has been evacuated:
    // The - 63 is because we would round up to the nearest word below.
    if (span >= kDecoderWindowSize * 2 - 63)
    {
        // Invariant: End - Base <= kDecoderWindowSize
        // This means we have evacuated the whole window.
        Lost.SetAll();
        SequenceBase = sequenceStart;
        //SequenceEnd = sequenceEnd; - Already set above

        // Reset packet ring buffer rotation back to front
        PacketsRotation = 0;

        // This does not seem to have a huge impact on overhead
#ifdef CCAT_FREE_UNUSED_PACKETS
        for (unsigned i = 0; i < kDecoderWindowSize; ++i)
        {
            AllocPtr->Free(Packets[i].Data);
            Packets[i].Data = nullptr;
        }
#endif // CCAT_FREE_UNUSED_PACKETS

        // Clear recovery list whenever window is cleared
        ClearRecoveryList();
        return Expand::Evacuated;
    }

    // Handle span between (kDecoderWindowSize, kDecoderWindowSize * 2) uninclusive:

    // Round up the minimum bit shift required to make room to the next word
    const unsigned minBitShift = (unsigned)span - kDecoderWindowSize;
    const unsigned roundWordShift = (minBitShift + 63) / 64;
    PKTALLOC_DEBUG_ASSERT(roundWordShift >= 1 && roundWordShift < Lost.kWords);

    // Shift words left to make room for all the new elements
    for (unsigned i = roundWordShift; i < Lost.kWords; ++i) {
        Lost.Words[i - roundWordShift] = Lost.Words[i];
    }

    // Mark all new elements as lost
    for (unsigned i = Lost.kWords - roundWordShift; i < Lost.kWords; ++i) {
        Lost.Words[i] = (uint64_t)~((int64_t)0); // All lost
    }

    const unsigned lostBits = roundWordShift * 64;
    PKTALLOC_DEBUG_ASSERT(lostBits < kDecoderWindowSize);

#ifdef CCAT_FREE_UNUSED_PACKETS
    unsigned element = PacketsRotation;
    for (unsigned i = 0; i < lostBits; ++i)
    {
        AllocPtr->Free(Packets[element].Data);
        Packets[element].Data = nullptr;
        ++element;
        if (element >= kDecoderWindowSize) {
            element -= kDecoderWindowSize;
        }
    }
#endif // CCAT_FREE_UNUSED_PACKETS

    SequenceBase += lostBits;
    PKTALLOC_DEBUG_ASSERT(SequenceEnd - SequenceBase <= kDecoderWindowSize);
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase);

    // Increment packet rotation modulo the window size
    PacketsRotation += lostBits;
    if (PacketsRotation >= kDecoderWindowSize) {
        PacketsRotation -= kDecoderWindowSize;
    }

    return Expand::Shifted;
}

void Decoder::ClearRecoveryList()
{
    // For each recovery packet:
    for (RecoveryPacket* recovery = RecoveryFirst, *next; recovery; recovery = next)
    {
        next = recovery->Next;

        // Free memory for this packet
        AllocPtr->Free(recovery->Data);
        AllocPtr->Destruct(recovery);
    }

    // Clear recovery list
    RecoveryFirst = nullptr;
    RecoveryLast = nullptr;
}

void Decoder::CleanupRecoveryList()
{
    const Counter64 sequenceBase = SequenceBase;

    // For each recovery packet:
    for (RecoveryPacket* recovery = RecoveryFirst, *next; recovery; recovery = next)
    {
        // If we found a recovery packet that is entirely within the window:
        if (recovery->SequenceStart >= sequenceBase)
        {
            // Set this as the new recovery list head,
            // as all remaining sorted rows will also be within the window.
            RecoveryFirst = recovery;
            recovery->Prev = nullptr;

            PKTALLOC_DEBUG_ASSERT(recovery->SequenceEnd <= SequenceEnd);
            PKTALLOC_DEBUG_ASSERT(RecoveryLast->SequenceEnd <= SequenceEnd);

            return;
        }
        next = recovery->Next;

        // Free memory for this packet
        AllocPtr->Free(recovery->Data);
        AllocPtr->Destruct(recovery);
    }

    // Clear recovery list
    RecoveryFirst = nullptr;
    RecoveryLast = nullptr;
}

CCatResult Decoder::StoreOriginal(const CCatOriginal& original)
{
    const Counter64 sequence = original.SequenceNumber;
    const unsigned element = (unsigned)(sequence - SequenceBase).ToUnsigned();
    PKTALLOC_DEBUG_ASSERT(element < kDecoderWindowSize);

    // If this element was not lost:
    if (!Lost.Check(element))
    {
        // We have already received it.  This happens if recovery succeeds and
        // then the original arrives later.
        return CCat_Success;
    }

    Lost.Clear(element);

    // Reallocate element memory
    OriginalPacket* packet = GetPacket(element);
    packet->Data = AllocPtr->Reallocate(
        packet->Data,
        2 + original.Bytes,
        pktalloc::Realloc::Uninitialized);

    if (!packet->Data) {
        return CCat_OOM;
    }

    // Copy original data here with length prepended
    WriteU16_LE(packet->Data, (uint16_t)(original.Bytes - 1));
    memcpy(packet->Data + 2, original.Data, original.Bytes);
    packet->Bytes = 2 + original.Bytes;

    return CCat_Success;
}

CCatResult Decoder::StoreRecovery(const CCatRecovery& recovery)
{
    // Allocate packet
    uint8_t* data = AllocPtr->Allocate(recovery.Bytes);

    if (!data) {
        return CCat_OOM;
    }

    RecoveryPacket* packet = AllocPtr->Construct<RecoveryPacket>();

    if (!packet) {
        AllocPtr->Free(data);
        return CCat_OOM;
    }

    const Counter64 sequenceStart = recovery.SequenceStart;
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase); // Should never happen
    const Counter64 sequenceEnd = recovery.SequenceStart + recovery.Count;

    // Write recovery data
    memcpy(data, recovery.Data, recovery.Bytes);
    packet->Bytes = recovery.Bytes;
    packet->Data = data;
    packet->SequenceStart = sequenceStart;
    packet->SequenceEnd = sequenceEnd;
    packet->MatrixRow = recovery.RecoveryRow;

    RecoveryPacket* prev = RecoveryLast;
    RecoveryPacket* next = nullptr;

    // Find insertion point
    while (prev)
    {
        // If packet should be inserted between prev, next
        if (prev->SequenceEnd < sequenceEnd) {
            break;
        }
        else if (prev->SequenceEnd == sequenceEnd &&
            prev->SequenceStart <= sequenceStart)
        {
            break;
        }

        next = prev;
        prev = prev->Prev;
    }

    // Insert between next and prev
    if (next)
    {
        if (sequenceStart > next->SequenceStart || sequenceEnd > next->SequenceEnd)
        {
            PKTALLOC_DEBUG_BREAK(); // Invalid input
            AllocPtr->Free(data);
            AllocPtr->Destruct(packet);

            return CCat_InvalidInput;
        }

        next->Prev = packet;
    }
    else {
        RecoveryLast = packet;
    }

    if (prev)
    {
        if (prev->SequenceStart > sequenceStart || prev->SequenceEnd > sequenceEnd)
        {
            PKTALLOC_DEBUG_BREAK(); // Invalid input
            AllocPtr->Free(data);
            AllocPtr->Destruct(packet);
            return CCat_InvalidInput;
        }

        prev->Next = packet;
    }
    else {
        RecoveryFirst = packet;
    }

    packet->Next = next;
    packet->Prev = prev;

    return CCat_Success;
}

/**
    Decoding algorithm

    Decoding is attempted whenever a recovery packet arrives and it references
    more than one lost original packet, or when an original packet arrives that
    is referenced by at least one recovery packet.

    If a recovery packet references no lost data then it will be ignored.
    If a recovery packet references just one lost original packet,
    then it will recover that original packet.  Optimizing for the single
    loss case helps because this is the most common recovery scenario.

    The decoding algorithm proceeds in three phases that are executed when new
    data arrives, and are repeated until no further solutions are found:

    1  Solution Search: Iteratively search for possible solutions.

    2  Solution Planning: Execute Gaussian elimination with partial pivoting.

    3  Execute Solution: Operate on the received data to recover lost data.

    GOTO 1


    Matrix View of Data:

    Columns are lost packets, and rows are recovery packets.

    The decoding algorithm views the received original data as columns in a
    matrix, where columns on the right contain the latest data.  It views the
    received recovery data as rows in the matrix, where rows on the bottom
    contain the latest data.

    Successfully received original data are eliminated from the columns, so
    the matrix columns correspond only to lost data.  Only rows that contain a
    loss are kept, so the matrix rows correspond only to useful recovery data.

    It's helpful to see that this matrix is banded like this example:

    11 22 33 44 00 00 00 00 00
    10 20 30 40 00 00 00 00 00
    00 55 66 77 88 99 00 00 00
    00 aa bb cc dd ee 00 00 00
    00 a1 b1 c1 d1 e1 00 00 00
    00 a2 b2 c2 d2 e2 00 00 00
    00 00 00 ff 01 02 03 04 00
    00 00 00 05 06 07 08 09 00
    00 00 00 00 0a 0b 0c 0d 0e

    Each element of the matrix is a field element in GF(256) between 0 and 255.
    This matrix may be invertible because the diagonal contains no 00 elements.
    If the matrix is invertible, then the received rows may be used to recover
    the original lost data.

    The banded structure comes from the recovery packet spans, which add new
    data on the right side and evacuate old data from the left side as they
    fall out of the window.  As a convolutional code, CCat multiples each
    packet by a value and sums the results together to produce each recovery
    packet, so these matrix elements contain the multiplied coefficients.


    (1) Solution Search Phase:

    Scan from the right of matrix for a submatrix that might be invertible.

    This phase is motivated by the expected receive pattern where data is mostly
    received in order but sometimes lost.  It will still work with reordered
    data but might miss some rare corner case chances to recover.

    The main idea is to always include and start from the right side of the
    matrix containing the latest received recovery data.  As new originals
    arrive or recovery data arrives or a previous solution is completed,
    we start over from the right side looking for a solution.

    To check if a matrix might be invertible it suffices to verify two simple
    conditions as recovery rows are added from the right side:

    (a) Is the diagonal non-zero?  This is confirmed by checking that the span
    right edge does not clip the diagonal as new rows are added.

    (b) For the rows added so far: Is the number of recovery rows matching
    the number of columns included in the submatrix described by these rows?

    To avoid checking these conditions for all received data every time new data
    arrives, the software will store information about the previous search and
    reuse it to exit early if possible.

    If data is received out of order then a situation like this can happen:

    Receive recovery rows protecting losses:
        {0, 1, 2, 3, 4}
        {0, 1, 2, 3, 4}
              {2, 3, 4}
              {2, 3, 4}
                 {3, 4, 5, 6, 7}
    ... Then receive original packet 3 out of order!

    Since we have already accepted the last recovery row, the iterative solver
    will not find the solution offered by the two short rows.  This seems like
    a situation that is not worth designing for because reordering is much more
    rare than packetloss and reordering that leads to this sort of situation is
    even more rare.


    (2) Solution Planning Phase:

    Phase (1) identified a submatrix to invert.  This phase attempts to invert
    the matrix using Gaussian Elimination (GE) with partial pivoting.
    This planning phase does not involve the actual packet data.

    Before performing GE, the optimized data structures for storing original
    and recovery data are transformed into arrays of rows and columns for the
    following steps.

    It then generates the matrix to invert in memory.  An optimization is to
    perform the first round of GE while generating the matrix to avoid one round
    of memory accesses.

    It verifies that a solution is possible with the given matrix by putting the
    matrix into upper-triangular form with GE.  Pivoting means that if a
    diagonal is zero during the operation, it will swap the row with a zero with
    another row below that does not have a zero in that column.

    An optimization is to have two versions of the GE algorithm:
    One that implements pivoting and one that assumes no pivoting.
    It can start with the faster version that does not handle pivoting,
    and then switch to the pivoting version if that one fails. 
    Since pivoting is only rarely needed, the common case can involve
    simpler code that can make the assumption that the matrix is banded.

    If no solution can be found, then solution should not be attempted again
    with only this same data.  New data should unlock further attempts.


    (3) Execute Solution:

    The result of phase (2) is a plan, specifically a set of math operations
    that can be performed on the recovery data to recover the original data.

    The first step is to resize all of the recovery data to the same maximum
    size padded with zeros.  Note that recovery data length is based on the
    set of original data in its window.  Since each recovery row protects
    a potentially different subset of the original data, the sizes must be
    made the same.  This is because all data will soon be mixed into all other
    data, which will grow all of the buffers to this largest size.

    Before executing that plan, the original data that has been received must
    be removed from the recovery data.  Since the loss rate is typically low
    compared to the data rate, this is actually the most expensive operation.
    The column value for each received original is multiplied by that original
    and added to each recovery row that references it, leaving behind only a
    combination of the lost columns in each row data.

    The same operations performed during GE on the matrix are then executed
    again on the actual recovery data.  This data reveals the original packets
    and the first bytes contain the length of the data, to allow the original
    packet length to be recovered.  These original packets are then reported
    to the application.

    The rows used for solution are erased from the recovery list and state is
    returned back to normal.  If there are more solutions they will be found
    on the right side of the matrix, so phase (1) is repeated until all
    solutions are found.
*/

CCatResult Decoder::SolveLostOne(const CCatRecovery& recovery)
{
    // Calculate element range
    const Counter64 sequenceStart = recovery.SequenceStart;
    const Counter64 sequenceEnd = recovery.SequenceStart + recovery.Count;
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase);
    const unsigned elementStart = (unsigned)(sequenceStart - SequenceBase).ToUnsigned();
    const unsigned elementEnd = (unsigned)(sequenceEnd - SequenceBase).ToUnsigned();
    PKTALLOC_DEBUG_ASSERT(elementEnd <= kDecoderWindowSize);

    // Find lost element
    const unsigned lostElement = Lost.FindFirstSet(elementStart, elementEnd);
    const Counter64 lostSequence = SequenceBase + lostElement;
    PKTALLOC_DEBUG_ASSERT(lostElement < kDecoderWindowSize);
    OriginalPacket* lostPacket = GetPacket(lostElement);

    // Reallocate data
    const unsigned recoveryBytes = recovery.Bytes;
    uint8_t* data = AllocPtr->Reallocate(
        lostPacket->Data,
        recoveryBytes,
        pktalloc::Realloc::Uninitialized);
    lostPacket->Data = data;

    if (!data) {
        return CCat_OOM;
    }

    memcpy(data, recovery.Data, recoveryBytes);

    // Calculate Packets[] element
    unsigned element = elementStart;
    element += PacketsRotation;
    if (element >= kDecoderWindowSize) {
        element -= kDecoderWindowSize;
    }

    // Calculate matrix column for loss
    unsigned column = (unsigned)(sequenceStart.ToUnsigned() % kMatrixColumnCount);
    const uint8_t row = recovery.RecoveryRow;
    uint8_t lostColumn = 0;

    // For each protected packet:
    for (Counter64 sequence = sequenceStart; sequence < sequenceEnd; ++sequence)
    {
        // If this is the lost sequence:
        if (sequence == lostSequence) {
            lostColumn = (uint8_t)column;
        }
        else
        {
            // Eliminate this original packet
            PKTALLOC_DEBUG_ASSERT(element < kDecoderWindowSize);
            OriginalPacket* original = &Packets[element];
            const uint8_t* originalData = original->Data;
            const unsigned originalBytes = original->Bytes;
            PKTALLOC_DEBUG_ASSERT(original->Bytes >= 2);

            if (!originalData || originalBytes > recoveryBytes) {
                PKTALLOC_DEBUG_BREAK(); // Invalid input
                return CCat_InvalidInput;
            }

            if (row == 0) {
                gf256_add_mem(data, originalData, originalBytes);
            }
            else
            {
                const uint8_t y = GetMatrixElement(row, (uint8_t)column);

                gf256_muladd_mem(data, y, originalData, originalBytes);
            }
        }

        // Next column
        ++column;
        if (column >= kMatrixColumnCount) {
            column -= kMatrixColumnCount;
        }
        ++element;
        if (element >= kDecoderWindowSize) {
            element -= kDecoderWindowSize;
        }
    }

    // If this is not a parity row:
    if (row != 0)
    {
        const uint8_t y_inv = gf256_inv(GetMatrixElement(row, lostColumn));

        gf256_mul_mem(data, data, y_inv, recovery.Bytes);
    }

    // Check size
    const unsigned originalBytes = (unsigned)ReadU16_LE(data) + 1;
    if (originalBytes > recoveryBytes)
    {
        PKTALLOC_DEBUG_BREAK(); // Invalid input.  Probably passed the wrong sequence numbers in?
        return CCat_InvalidInput;
    }

    lostPacket->Bytes = 2 + originalBytes;

    // Mark this element as received
    Lost.Clear(lostElement);

    // Report recovery
    CCatOriginal recoveredOriginal;
    recoveredOriginal.Data = data + 2;
    recoveredOriginal.Bytes = originalBytes;
    recoveredOriginal.SequenceNumber = lostSequence.ToUnsigned();
    SettingsPtr->OnRecoveredData(recoveredOriginal, SettingsPtr->AppContextPtr);

    // Check if any solutions are possible with this one
    return FindSolutionsContaining(lostSequence);
}

CCatResult Decoder::FindSolutionsContaining(const Counter64 sequence)
{
    // If there is no recovery list:
    if (!RecoveryLast) {
        return CCat_Success;
    }

    PKTALLOC_DEBUG_ASSERT(RecoveryFirst != nullptr);
    PKTALLOC_DEBUG_ASSERT(RecoveryFirst->SequenceStart <= sequence);

    // Receiving originals out of order is rare, so apply some heuristics:
    // If there is a recovery packet that now references just one loss,
    // then find a solution based on it.
    bool unreferenced = true;

    // Scan from right to left:
    for (RecoveryPacket* recovery = RecoveryLast; recovery; recovery = recovery->Prev)
    {
        // If this recovery packet cannot use it:
        if (sequence >= recovery->SequenceEnd) {
            continue; // Try next
        }

        // If this and remaining recovery packets do not contain it:
        if (sequence < recovery->SequenceStart) {
            break; // Stop here
        }

        // Find the number of lost in this recovery packet range
        const unsigned loss = GetLostInRange(recovery->SequenceStart, recovery->SequenceEnd);
        PKTALLOC_DEBUG_ASSERT(loss > 0 && loss <= kMatrixColumnCount);

        // If there is only one loss:
        if (loss == 1) {
            // Skip all this nonsense and solve immediately
            return Solve(recovery, recovery);
        }

        unreferenced = false;
    }

    // If it is unreferenced:
    if (unreferenced) {
        return CCat_Success;
    }

    // Find solutions since some recovery packets referenced the received data
    return FindSolutions();
}

CCatResult Decoder::FindSolutions()
{
    RecoveryPacket* next = RecoveryLast;
    if (!next) {
        return CCat_NeedsMoreData;
    }

    Counter64 nextSequenceStart = next->SequenceStart;
    unsigned nextLoss = GetLostInRange(nextSequenceStart, next->SequenceEnd);
    unsigned fill = 1;
    unsigned loss = nextLoss;
    unsigned rightZeros = 0;
    PKTALLOC_DEBUG_ASSERT(loss >= fill);

    // If a solution is possible:
    if (loss == fill) {
        return Solve(next, next);
    }

    RecoveryPacket* prev = next->Prev;

    // While there are rows to check to the left:
    while (prev)
    {
        // Increment number of rows filled in so far
        ++fill;

        // If the previous and next do not overlap:
        if (prev->SequenceEnd <= nextSequenceStart) {
            break; // Stop where disjoint
        }

        // Calculate overlapping loss count
        unsigned overlapLoss;
        PKTALLOC_DEBUG_ASSERT(prev->SequenceEnd <= next->SequenceEnd);
        if (prev->SequenceEnd < next->SequenceEnd) {
            overlapLoss = GetLostInRange(nextSequenceStart, prev->SequenceEnd);
        }
        else {
            overlapLoss = nextLoss;
        }

        // Accumulate the zeros on the right since this scan started
        PKTALLOC_DEBUG_ASSERT(nextLoss >= overlapLoss);
        rightZeros += nextLoss - overlapLoss;

        // If the number of zeros on the right crosses the diagonal:
        if (rightZeros >= fill) {
            /* Example:
                11 22 00 <- fill=2, rightZeros = 1
                00 22 33 <- fill=1
            */
            break; // No solution possible
        }

        // If there is no overlap with recovery data to the left:
        if (overlapLoss == 0) {
            break; // Stop at disjointed section
        }

        // Calculate added losses
        unsigned addedLoss = 0;
        const Counter64 prevSequenceStart = prev->SequenceStart;
        PKTALLOC_DEBUG_ASSERT(prevSequenceStart <= nextSequenceStart);
        if (prevSequenceStart < nextSequenceStart)
        {
            nextLoss = GetLostInRange(prevSequenceStart, prev->SequenceEnd);
            PKTALLOC_DEBUG_ASSERT(nextLoss >= overlapLoss);
            addedLoss = nextLoss - overlapLoss;
        }
        else {
            nextLoss = overlapLoss; // Used in next loop
        }

        // Added one recovery row and some number of losses
        loss += addedLoss;
        PKTALLOC_DEBUG_ASSERT(loss >= fill);

        // If loss size exceeds the limits of the codec:
        if (loss > kMaxRecoveryColumns) {
             break;
        }

        // If a solution is possible:
        if (loss == fill)
        {
            // Include recovery packets to the left that start on the same column
            for (unsigned rowsAdded = fill; rowsAdded < kMaxRecoveryRows; ++rowsAdded)
            {
                RecoveryPacket* additional = prev->Prev;

                if (!additional || additional->SequenceStart != prevSequenceStart) {
                    break;
                }

                prev = additional;
            }

            return Solve(prev, RecoveryLast);
        }

        // Move left to lower sequence numbers
        next = prev;
        nextSequenceStart = prevSequenceStart;
        prev = prev->Prev;
    }

    return CCat_NeedsMoreData;
}

CCatResult Decoder::Solve(RecoveryPacket* spanStart, RecoveryPacket* spanEnd)
{
    PKTALLOC_DEBUG_ASSERT(spanStart != nullptr && spanEnd != nullptr);

    CCatResult result;

    // Convert span to arrays of columns and rows
    result = ArraysFromSpans(spanStart, spanEnd);
    if (result != CCat_Success) {
        goto OnFail;
    }

    // Identify the matrix solution to use, or it will fail if no solution is possible
    result = PlanSolution();
    if (result != CCat_Success) {
        goto OnFail;
    }

    // Eliminate data that was not lost from the recovery rows (expensive)
    result = EliminateOriginals();
    if (result != CCat_Success) {
        goto OnFail;
    }

    // Produce the original data from recovery data, following plan
    ExecuteSolutionPlan();

    // Report recovered data to application
    result = ReportSolution();
    if (result != CCat_Success) {
        goto OnFail;
    }

    ++LargeRecoverySuccesses;

OnFail:
    // Release temporary space for this span and get resume point for search
    ReleaseSpan(spanStart, spanEnd, result);

    // If any failures occurred:
    if (result != CCat_Success)
    {
        ++LargeRecoveryFailures;
        return result;
    }

    return FindSolutions();
}

CCatResult Decoder::ArraysFromSpans(RecoveryPacket* spanStart, RecoveryPacket* spanEnd)
{
    SolutionBytes = 0;
    RowCount = 0;
    ColumnCount = 0;

    unsigned solutionBytes = 0;
    unsigned rowCount = 0;
    RecoveryPacket* recovery = spanStart;

    // Convert recovery row span into an array and calculate SolutionBytes
    while (recovery)
    {
        // Incorporate this row into the array
        PKTALLOC_DEBUG_ASSERT(recovery->MatrixRow < kMatrixRowCount);
        CauchyRows[rowCount] = recovery->MatrixRow;
        RowInfo[rowCount].Recovery = recovery;
        ++rowCount;
        PKTALLOC_DEBUG_ASSERT(rowCount <= kMaxRecoveryRows);

        // Update solution bytes
        if (solutionBytes < recovery->Bytes) {
            solutionBytes = recovery->Bytes;
        }

        if (spanEnd == recovery) {
            break;
        }

        recovery = recovery->Next;
        PKTALLOC_DEBUG_ASSERT(recovery);
    }
    PKTALLOC_DEBUG_ASSERT(solutionBytes > 2);
    SolutionBytes = solutionBytes;
    PKTALLOC_DEBUG_ASSERT(rowCount > 0);
    RowCount = rowCount;

    // Store original columns
    const Counter64 sequenceStart = spanStart->SequenceStart;
    const Counter64 sequenceEnd = spanEnd->SequenceEnd;
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase);
    const unsigned elementStart = (unsigned)(sequenceStart - SequenceBase).ToUnsigned();
    const unsigned elementEnd = (unsigned)(sequenceEnd - SequenceBase).ToUnsigned();
    PKTALLOC_DEBUG_ASSERT(elementStart < elementEnd);
    PKTALLOC_DEBUG_ASSERT(elementEnd <= kDecoderWindowSize);

    // Calculate matrix column for loss
    unsigned columnStart = (unsigned)(sequenceStart.ToUnsigned() % kMatrixColumnCount);
    // FIXME: I think this fails after 2^64 packets

    // For each element in the range:
    unsigned columnCount = 0;
    unsigned lossSearchStart = elementStart;
    for (;;)
    {
        unsigned nextLoss = Lost.FindFirstSet(lossSearchStart, elementEnd);

        // Constrain the loss index within the receive window.
        // It can fall out ahead if the window is partially empty.
        if (nextLoss >= elementEnd) {
            break;
        }

        // Translate lost element into its rotated position in the decoder window ring buffer
        unsigned element = nextLoss;
        element += PacketsRotation;
        if (element >= kDecoderWindowSize) {
            element -= kDecoderWindowSize;
            PKTALLOC_DEBUG_ASSERT(element < kDecoderWindowSize);
        }

        // Next window element
        unsigned column = columnStart + nextLoss - elementStart;
        if (column >= kMatrixColumnCount)
        {
            column -= kMatrixColumnCount;

            if (column >= kMatrixColumnCount) {
                column -= kMatrixColumnCount;
                PKTALLOC_DEBUG_ASSERT(column < kMatrixColumnCount);
            }
        }

        // Map column to original data
        ColumnInfo[columnCount].OriginalPtr = &Packets[element];
        ColumnInfo[columnCount].Sequence = sequenceStart + nextLoss - elementStart;
        PKTALLOC_DEBUG_ASSERT(column < kMatrixColumnCount);
        CauchyColumns[columnCount] = (uint8_t)column;
        ++columnCount;
        PKTALLOC_DEBUG_ASSERT(columnCount <= kMaxRecoveryColumns);

        // Start searching from next element
        lossSearchStart = nextLoss + 1;
    }
    PKTALLOC_DEBUG_ASSERT(columnCount > 0);
    ColumnCount = columnCount;

    // Sanity check the system state
    if (rowCount < columnCount) {
        PKTALLOC_DEBUG_BREAK(); // Should never happen
        return CCat_Error;
    }

    // For each row:
    for (unsigned i = 0; i < rowCount; ++i) {
        // Initialize pivot rows assuming no row swaps
        PivotRowIndex[i] = (uint8_t)i;
    }

    // For each column:
    for (unsigned i = 0; i < columnCount; ++i) {
        // Clear diagonal data in case we fail
        DiagonalData[i] = nullptr;
    }

    return CCat_Success;
}

CCatResult Decoder::PlanSolution()
{
    // Note we could allocate the matrix rows on aligned memory addresses,
    // but for CCat the matrix is banded and not aligned to the left
    // as later recovery packets evacuated data on the left of the window,
    // so there is no advantage to padding; on ARM just do it byte-wise.

    const unsigned rowCount = RowCount;
    const unsigned columnCount = ColumnCount;

    // Allocate matrix
    const bool resizeResult = Matrix.Resize(
        AllocPtr,
        rowCount * columnCount,
        pktalloc::Realloc::Uninitialized);

    if (!resizeResult) {
        return CCat_OOM;
    }

    uint8_t* matrix = Matrix.GetPtr();

    // Uninitialized rows will contain zeros, which is used later in ExecuteSolutionPlan()
    memset(matrix, 0, rowCount * columnCount);

    /*
        This loop accomplishes three things simultaneously in one sweep through the matrix:
        (1) Filling in the matrix.
        (2) Unrolling the first GE column elimination.
        (3) Filling in ColumnStart and ColumnEnd for RowInfo[].
    */

    // Unroll first GE loop and build matrix:
    uint8_t* pivot_data = matrix;
    unsigned pivotColumnEnd = columnCount;
    {
        RecoveryPacket* recovery = RowInfo[0].Recovery;
        const uint8_t generatorRow = CauchyRows[0];

        PKTALLOC_DEBUG_ASSERT(recovery->SequenceStart <= ColumnInfo[0].Sequence);
        PKTALLOC_DEBUG_ASSERT(recovery->SequenceEnd > ColumnInfo[0].Sequence);
        RowInfo[0].ColumnStart = 0;

        // Write element (0, 0)
        const uint8_t x_first = GetMatrixElement(generatorRow, CauchyColumns[0]);
        pivot_data[0] = x_first;

        // Divide remaining nonzero columns in this row by element (0, 0)
        for (unsigned column = 1; column < columnCount; ++column)
        {
            const Counter64 columnSequence = ColumnInfo[column].Sequence;
            PKTALLOC_DEBUG_ASSERT(columnSequence > ColumnInfo[column - 1].Sequence);
            if (columnSequence >= recovery->SequenceEnd)
            {
                pivotColumnEnd = column;
                break;
            }

            const uint8_t x = GetMatrixElement(generatorRow, CauchyColumns[column]);
            pivot_data[column] = gf256_div(x, x_first);
        }

        // Determine column extent of each row in this first pass also
        RowInfo[0].ColumnEnd = pivotColumnEnd;
    }

    uint8_t* elim_data = pivot_data;
    unsigned prevColumnStart = 0;

    // Build remainder of matrix:
    for (unsigned row = 1; row < rowCount; ++row)
    {
        elim_data += columnCount;

        RecoveryPacket* recovery = RowInfo[row].Recovery;
        const Counter64 rowSequenceStart = recovery->SequenceStart;
        const uint8_t generatorRow = CauchyRows[row];

#ifdef PKTALLOC_DEBUG
        // Verify that this row does not start earlier than prior row
        for (unsigned column = 0; column < prevColumnStart; ++column)
        {
            PKTALLOC_DEBUG_ASSERT(column == 0 || ColumnInfo[column].Sequence > ColumnInfo[column - 1].Sequence);
            PKTALLOC_DEBUG_ASSERT(rowSequenceStart > ColumnInfo[column].Sequence);
        }
#endif

        // Find first lost column in the recovery set:
        unsigned columnStart = columnCount;
        for (unsigned column = prevColumnStart; column < columnCount; ++column)
        {
            const Counter64 columnSequence = ColumnInfo[column].Sequence;
            PKTALLOC_DEBUG_ASSERT(column == 0 || columnSequence > ColumnInfo[column - 1].Sequence);

            if (rowSequenceStart <= columnSequence)
            {
                columnStart = column;
                break;
            }

            PKTALLOC_DEBUG_ASSERT(column < row); // Rows must contain the diagonal
        }

        PKTALLOC_DEBUG_ASSERT(columnStart < columnCount);
        RowInfo[row].ColumnStart = columnStart;
        prevColumnStart = columnStart;

        // Write first element
        const uint8_t x_first = GetMatrixElement(generatorRow, CauchyColumns[columnStart]);
        elim_data[columnStart] = x_first;

        // Find column extent of this row
        unsigned columnEnd = columnCount;
        unsigned column = columnStart + 1;

        // If this row is modified by the pivot 0 row:
        if (columnStart <= 0)
        {
            // Unroll case where first row is added into this one:
            for (; column < pivotColumnEnd; ++column)
            {
                PKTALLOC_DEBUG_ASSERT(ColumnInfo[column].Sequence > ColumnInfo[column - 1].Sequence);
                PKTALLOC_DEBUG_ASSERT(recovery->SequenceEnd >= ColumnInfo[column].Sequence);

                // Muladd pivot row into this one
                const uint8_t x = GetMatrixElement(generatorRow, CauchyColumns[column]);
                const uint8_t y = gf256_mul(pivot_data[column], x_first);

                elim_data[column] = gf256_add(x, y);

                PKTALLOC_DEBUG_ASSERT(elim_data[column] != 0);
            }
        }

        // Fill in remaining columns
        for (; column < columnCount; ++column)
        {
            const Counter64 columnSequence = ColumnInfo[column].Sequence;

            PKTALLOC_DEBUG_ASSERT(columnSequence > ColumnInfo[column - 1].Sequence);

            if (recovery->SequenceEnd <= columnSequence)
            {
                columnEnd = column;
                break;
            }

            elim_data[column] = GetMatrixElement(generatorRow, CauchyColumns[column]);
        }

        RowInfo[row].ColumnEnd = columnEnd;
    }

    // Resume Gaussian elimination from row 1
    return ResumeGaussianElimination(pivot_data, 1);
}

CCatResult Decoder::ResumeGaussianElimination(
    uint8_t* pivot_data,
    unsigned row)
{
    const unsigned rowCount = RowCount;
    const unsigned columnCount = ColumnCount;

    // Continue elimination for remaining pivots:
    for (; row < columnCount; ++row)
    {
        pivot_data += columnCount;

        const unsigned pivotColumnStart = RowInfo[row].ColumnStart;
        const unsigned pivotColumnEnd = RowInfo[row].ColumnEnd;

        // If the normal row order does not contain this:
        if (pivotColumnStart >= row) {
            PKTALLOC_DEBUG_BREAK(); // Should never happen
            return CCat_Error;
        }

        // Divide pivot row by diagonal
        {
            // If diagonal is zero:
            const uint8_t diag = pivot_data[row];
            if (diag == 0) {
                // Attempt to pivot rows around to find a non-zero diagonal
                return PivotedGaussianElimination(row);
            }

#ifndef GF256_ALIGNED_ACCESSES
            PKTALLOC_DEBUG_ASSERT(pivotColumnEnd >= row + 1);
            const unsigned count = pivotColumnEnd - row - 1;
            if (count >= 8)
            {
                uint8_t* data = pivot_data + row + 1;

                gf256_div_mem(data, data, diag, count);
            }
            else
#endif
            {
                for (unsigned column = row + 1; column < pivotColumnEnd; ++column) {
                    pivot_data[column] = gf256_div(pivot_data[column], diag);
                }
            }
        }

        // Add it to each remaining row that contains this column:
        uint8_t* elim_data = pivot_data;
        for (unsigned elim_row = row + 1; elim_row < rowCount; ++elim_row)
        {
            elim_data += columnCount;

            const unsigned columnStart = RowInfo[elim_row].ColumnStart;
            PKTALLOC_DEBUG_ASSERT(columnStart >= pivotColumnStart);

            if (columnStart > row) {
                // Following rows no longer contain this column also
                break;
            }

            PKTALLOC_DEBUG_ASSERT(RowInfo[elim_row].ColumnEnd >= pivotColumnEnd);

            // Muladd pivot row into this one
            const uint8_t elim_value = elim_data[row];

            if (elim_value == 0) {
                // No changes needed
                continue;
            }

#ifndef GF256_ALIGNED_ACCESSES
            PKTALLOC_DEBUG_ASSERT(pivotColumnEnd >= row + 1);
            const unsigned count = pivotColumnEnd - row - 1;

            if (count >= 8)
            {
                gf256_muladd_mem(
                    elim_data + row + 1,
                    elim_value,
                    pivot_data + row + 1,
                    count);
            }
            else
#endif
            {
                // elim_data[] += pivot[] * elim_value
                for (unsigned column = row + 1; column < pivotColumnEnd; ++column) {
                    elim_data[column] = gf256_add(elim_data[column], gf256_mul(pivot_data[column], elim_value));
                }
            }
        }
    }

    return CCat_Success;
}

CCatResult Decoder::PivotedGaussianElimination(unsigned pivotColumn)
{
    const unsigned rowCount = RowCount;
    const unsigned columnCount = ColumnCount;

    // Continue elimination for remaining pivots:
    for (;;)
    {
        uint8_t* pivot_data = nullptr;
        unsigned pivotColumnStart = 0, pivotColumnEnd = 0;

        // Find a workable pivot:
        for (unsigned i = pivotColumn; i < rowCount; ++i)
        {
            // Grab next row index to check
            const unsigned pivotRowIndex = PivotRowIndex[i];

            // Check if the diagonal is nonzero:
            uint8_t* data = Matrix.GetPtr() + pivotRowIndex * columnCount;
            const uint8_t diag = data[pivotColumn];
            if (diag == 0) {
                continue; // Try next row
            }

            // Check if column range covers the loss column:
            pivotColumnStart = RowInfo[pivotRowIndex].ColumnStart;
            PKTALLOC_DEBUG_ASSERT(pivotColumnStart <= pivotColumn);
            pivotColumnEnd = RowInfo[pivotRowIndex].ColumnEnd;
            PKTALLOC_DEBUG_ASSERT(pivotColumnEnd > pivotColumn);

            // Swap this pivot row into place
            if (i != pivotColumn)
            {
                const uint8_t temp = PivotRowIndex[pivotColumn];
                PivotRowIndex[pivotColumn] = (uint8_t)pivotRowIndex;
                PivotRowIndex[i] = temp;
            }

#ifndef GF256_ALIGNED_ACCESSES
            PKTALLOC_DEBUG_ASSERT(pivotColumnEnd >= pivotColumn + 1);
            const unsigned count = pivotColumnEnd - pivotColumn - 1;
            if (count >= 8)
            {
                uint8_t* ptr = data + pivotColumn + 1;

                gf256_div_mem(ptr, ptr, diag, count);
            }
            else
#endif
            {
                for (unsigned column = pivotColumn + 1; column < pivotColumnEnd; ++column) {
                    data[column] = gf256_div(data[column], diag);
                }
            }

            // Found it!
            pivot_data = data;
            break;
        }

        // If no rows found that can help:
        if (!pivot_data)
        {
            // Record failure point
            PKTALLOC_DEBUG_ASSERT(pivotColumn < ColumnCount);
            FailureSequence = ColumnInfo[pivotColumn].Sequence;
            return CCat_NeedsMoreData;
        }

        // If we are done:
        if (pivotColumn + 1 >= columnCount) {
            break;
        }

        // Add it to each remaining row that contains this column:
        for (unsigned i = pivotColumn + 1; i < rowCount; ++i)
        {
            // Grab next row index to eliminate
            const unsigned elimRowIndex = PivotRowIndex[i];

            // Check if column is zero:
            uint8_t* elim_data = Matrix.GetPtr() + elimRowIndex * columnCount;
            const uint8_t elim_value = elim_data[pivotColumn];

            if (elim_value == 0) {
                // No changes needed
                continue;
            }

            // Since we are adding the pivot to this row, it may expand left
            // or right with additional nonzero columns
            if (RowInfo[elimRowIndex].ColumnStart > pivotColumnStart) {
                RowInfo[elimRowIndex].ColumnStart = pivotColumnStart;
            }
            if (RowInfo[elimRowIndex].ColumnEnd < pivotColumnEnd) {
                RowInfo[elimRowIndex].ColumnEnd = pivotColumnEnd;
            }

            // Add pivot row to this one
#ifndef GF256_ALIGNED_ACCESSES
            PKTALLOC_DEBUG_ASSERT(pivotColumnEnd >= pivotColumn + 1);
            const unsigned count = pivotColumnEnd - pivotColumn - 1;
            if (count >= 8)
            {
                gf256_muladd_mem(
                    elim_data + pivotColumn + 1,
                    elim_value,
                    pivot_data + pivotColumn + 1,
                    count);
            }
            else
#endif
            {
                // elim_data[] += pivot[] * elim_value
                for (unsigned column = pivotColumn + 1; column < pivotColumnEnd; ++column) {
                    elim_data[column] = gf256_add(elim_data[column], gf256_mul(pivot_data[column], elim_value));
                }
            }
        }

        ++pivotColumn;
    }

    return CCat_Success;
}

CCatResult Decoder::EliminateOriginals()
{
    const unsigned solutionBytes = SolutionBytes;
    const unsigned columnCount = ColumnCount;

    // Find actual range of original data for the used recovery rows
    Counter64 sequenceEnd = RowInfo[0].Recovery->SequenceEnd;
    PKTALLOC_DEBUG_ASSERT(PivotRowIndex[0] == 0);

    // Allocate space for solutions from recovery data
    for (unsigned column = 0; column < columnCount; ++column)
    {
        const uint8_t rowIndex = PivotRowIndex[column];
        PKTALLOC_DEBUG_ASSERT(rowIndex < RowCount);
        RecoveryPacket* recovery = RowInfo[rowIndex].Recovery;

        // Find the largest sequence extent including received originals
        if (sequenceEnd < recovery->SequenceEnd) {
            sequenceEnd = recovery->SequenceEnd;
        }

        // Get recovery packet for this row
        uint8_t* data = recovery->Data;

        // Reallocate to larger size, keeping any data currently there
        data = AllocPtr->Reallocate(data, solutionBytes, pktalloc::Realloc::CopyExisting);

        // Clear data reference from recovery packet
        recovery->Data = nullptr;

        if (!data) {
            PKTALLOC_DEBUG_BREAK(); // Out of memory
            return CCat_OOM;
        }

        // Pad with zeros out to a consistent SolutionBytes size
        const unsigned bytes = recovery->Bytes;
        PKTALLOC_DEBUG_ASSERT(solutionBytes >= bytes);
        memset(data + bytes, 0, solutionBytes - bytes);

        // Use this buffer for the solution
        DiagonalData[column] = data;
    }

    // Store original columns
    Counter64 sequence = RowInfo[0].Recovery->SequenceStart;

    // Translate lost element into its rotated position in the decoder window ring buffer
    PKTALLOC_DEBUG_ASSERT(sequence >= SequenceBase);
    unsigned element = (unsigned)(sequence - SequenceBase).ToUnsigned();
    element += PacketsRotation;

    if (element >= kDecoderWindowSize)
    {
        element -= kDecoderWindowSize;
        PKTALLOC_DEBUG_ASSERT(element < kDecoderWindowSize);
    }

    // Calculate generator matrix column for first (lost or received) original packet
    unsigned generatorColumn = (unsigned)(sequence.ToUnsigned() % kMatrixColumnCount);
    // FIXME: I think this fails after 2^64 packets

    // Around each loss column:
    for (unsigned columnIndex = 0; columnIndex <= columnCount; ++columnIndex)
    {
        // Scan through the gaps before/between/after all losses
        Counter64 nextLostSequence;
        if (columnIndex < columnCount) {
            nextLostSequence = ColumnInfo[columnIndex].Sequence;
        }
        else {
            nextLostSequence = sequenceEnd;
        }

        // For each original packet we received before the loss:
        while (sequence < nextLostSequence)
        {
            // Get original
            OriginalPacket* original = &Packets[element];
            const uint8_t* originalData = original->Data;
            const unsigned originalBytes = original->Bytes;
            PKTALLOC_DEBUG_ASSERT(originalBytes >= 2);

            if (!originalData || originalBytes > solutionBytes) {
                PKTALLOC_DEBUG_BREAK(); // Invalid input
                return CCat_InvalidInput;
            }

            // For each recovery packet:
            for (unsigned pivotColumn = 0; pivotColumn < columnCount; ++pivotColumn)
            {
                const uint8_t rowIndex = PivotRowIndex[pivotColumn];
                const RecoveryPacket* recovery = RowInfo[rowIndex].Recovery;

                // If this original packet is not referenced by this row:
                if (sequence < recovery->SequenceStart ||
                    sequence >= recovery->SequenceEnd)
                {
                    // Note that sometimes the rows are switched around so
                    // we cannot abort early based on list sorting assumption.
                    continue;
                }

                // Eliminate the original data from this row
                uint8_t* resultData = DiagonalData[pivotColumn];
                const uint8_t generatorRow = CauchyRows[rowIndex];
                if (generatorRow == 0) {
                    gf256_add_mem(resultData, originalData, originalBytes);
                }
                else
                {
                    const uint8_t y = GetMatrixElement(generatorRow, (uint8_t)generatorColumn);

                    gf256_muladd_mem(resultData, y, originalData, originalBytes);
                }
            }

            // Next window element
            ++element;
            if (element >= kDecoderWindowSize) {
                element -= kDecoderWindowSize;
            }
            ++sequence;
            ++generatorColumn;
            if (generatorColumn >= kMatrixColumnCount) {
                generatorColumn -= kMatrixColumnCount;
            }
        }

        // Skip loss column
        ++element;
        if (element >= kDecoderWindowSize) {
            element -= kDecoderWindowSize;
        }
        ++sequence;
        ++generatorColumn;
        if (generatorColumn >= kMatrixColumnCount) {
            generatorColumn -= kMatrixColumnCount;
        }
    }

    return CCat_Success;
}

void Decoder::ExecuteSolutionPlan()
{
    const unsigned columnCount = ColumnCount;
    const unsigned solutionBytes = SolutionBytes;
    const uint8_t* matrix = Matrix.GetPtr();

    // Eliminate lower left triangle.  For each column:
    for (unsigned j = 0; j < columnCount; ++j)
    {
        void* block_j = DiagonalData[j];
        const uint8_t* matrix_j = matrix + PivotRowIndex[j] * columnCount;

        // Eliminate diagonal factor
        PKTALLOC_DEBUG_ASSERT(matrix_j[j] != 0);
        gf256_div_mem(block_j, block_j, matrix_j[j], solutionBytes);

        // For each row below diagonal:
        for (unsigned i = j + 1; i < columnCount; ++i)
        {
            const uint8_t* matrix_i = matrix + PivotRowIndex[i] * columnCount;

            gf256_muladd_mem(DiagonalData[i], matrix_i[j], block_j, solutionBytes);
        }
    }

    // Eliminate upper right triangle.  For each column:
    for (unsigned j = columnCount - 1; j >= 1; --j)
    {
        const void* block_j = DiagonalData[j];

        // For each row above diagonal:
        for (int i = j - 1; i >= 0; --i)
        {
            const uint8_t* matrix_i = Matrix.GetPtr() + PivotRowIndex[i] * columnCount;

            gf256_muladd_mem(DiagonalData[i], matrix_i[j], block_j, solutionBytes);
        }
    }

    // Mark all packets in range recovered
    const Counter64 sequenceStart = ColumnInfo[0].Sequence;
    const Counter64 sequenceEnd = ColumnInfo[ColumnCount - 1].Sequence + 1;
    PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase);
    const unsigned elementStart = (unsigned)(sequenceStart - SequenceBase).ToUnsigned();
    const unsigned elementEnd = (unsigned)(sequenceEnd - SequenceBase).ToUnsigned();
    PKTALLOC_DEBUG_ASSERT(elementEnd <= kDecoderWindowSize);

    Lost.ClearRange(elementStart, elementEnd);
}

CCatResult Decoder::ReportSolution()
{
    const unsigned columnCount = ColumnCount;
    const unsigned solutionBytes = SolutionBytes;
    void* appContextPtr = SettingsPtr->AppContextPtr;

    // For each solution:
    for (unsigned column = 0; column < columnCount; ++column)
    {
        // Fill in losses in the original window
        OriginalPacket* original = ColumnInfo[column].OriginalPtr;
        uint8_t* data = DiagonalData[column];
        PKTALLOC_DEBUG_ASSERT(original && data);

        // Decode length from the front overhead
        const unsigned originalBytes = ReadU16_LE(data) + 1;

        // Quick sanity check (catches a lot bugs):
        if (2 + originalBytes > solutionBytes)
        {
            /*******************************************************************

                If you get this assert it means either there is a bug in CCat or
                a bug in the application code.  The most common causes are:

                (1) Reusing the same packet sequence numbers.

                (2) Truncating the packet sequence number and rolling-over to
                produce the wrong packet number at decoder side.

                If the application code is clean please produce a dump file and
                send to the author: mrcatid@gmail.com  Thanks!

             ******************************************************************/
            PKTALLOC_DEBUG_BREAK();

            // Return invalid input and attempt to recover
            return CCat_InvalidInput;
        }

        // Free current original data pointer
        AllocPtr->Free(original->Data);

        // Store original data
        original->Data = data;
        original->Bytes = 2 + originalBytes; // Include size overhead

        // Clear diagonal data reference
        DiagonalData[column] = nullptr;

        // Report recovery success
        CCatOriginal recoveredOriginal;
        recoveredOriginal.Data = data + 2;
        recoveredOriginal.Bytes = originalBytes;
        recoveredOriginal.SequenceNumber = ColumnInfo[column].Sequence.ToUnsigned();
        SettingsPtr->OnRecoveredData(recoveredOriginal, appContextPtr);
    }

    return CCat_Success;
}

void Decoder::ReleaseSpan(
    RecoveryPacket* spanStart,
    RecoveryPacket* spanEnd,
    CCatResult solveResult)
{
    if (!spanStart) {
        return;
    }

    const unsigned columnCount = ColumnCount;

    // For each column:
    for (unsigned i = 0; i < columnCount; ++i)
    {
        AllocPtr->Free(DiagonalData[i]);

        DiagonalData[i] = nullptr;
    }

    // The minimum sequence number that we can expect to fix in the future
    const Counter64 futureMinSequence = RecoveryLast->SequenceStart;

    Counter64 poisonSequence = futureMinSequence;

    // If the solver will need more data to get past this point:
    if (solveResult == CCat_NeedsMoreData)
    {
        PKTALLOC_DEBUG_ASSERT(FailureSequence >= spanStart->SequenceStart);
        PKTALLOC_DEBUG_ASSERT(FailureSequence < spanEnd->SequenceEnd);

        // If we are unlikely to receive more recovery spans covering it:
        if (futureMinSequence > FailureSequence) {
            // Only clear recovery packets that reference the failure point
            poisonSequence = FailureSequence;
        }
        else {
            // With further recovery packets we might still solve this,
            // so keep what we have and wait for more to arrive.
            return;
        }
    }

    // Keep track of span edges
    RecoveryPacket* spanPrev = spanStart->Prev;
    RecoveryPacket* spanNext = spanEnd->Next;

    // Deallocate recovery packet span
    RecoveryPacket* recovery = spanStart;
    PKTALLOC_DEBUG_ASSERT(recovery);
    while (recovery)
    {
        // If this recovery packet does not include the poison sequence:
        if (recovery->SequenceStart > poisonSequence)
        {
            // Stop removing packets here
            spanNext = recovery;
            break;
        }

        RecoveryPacket* next = recovery->Next;

        // Free any unused recovery data
        AllocPtr->Free(recovery->Data);

        // Free all recovery objects in span
        AllocPtr->Destruct(recovery);

        if (spanEnd == recovery) {
            break;
        }

        recovery = next;
        PKTALLOC_DEBUG_ASSERT(recovery);
    }

    // Sequence number of left-most/right-most losses that were recovered
    const Counter64 leftLossSequence = ColumnInfo[0].Sequence;
    const Counter64 rightLossSequence = ColumnInfo[columnCount - 1].Sequence;
    RecoveryPacket* prev;
    RecoveryPacket* next;

    // Scan left to evacuate unneeded recovery data:
    next = nullptr;
    recovery = spanPrev;
    while (recovery)
    {
        prev = recovery->Prev;

        // If recovered span is disjoint with prev packet span:
        if (leftLossSequence >= recovery->SequenceEnd) {
            break; // Stop here
        }

        // Find remaining lost count in packet overlapping span
        const unsigned lost = GetLostInRange(recovery->SequenceStart, leftLossSequence);

        // If this recovery packet is still useful:
        if (lost != 0)
        {
            next = recovery;
            recovery = prev;
            continue;
        }

        // Unlink this packet
        if (!next) {
            spanPrev = prev;
        }
        else {
            if (prev) {
                prev->Next = next;
            }
            else {
                RecoveryFirst = next;
            }
            next->Prev = prev;
        }

        // Release this packet
        AllocPtr->Free(recovery->Data);
        AllocPtr->Destruct(recovery);

        recovery = prev;
    }

    // Scan right to evacuate unneeded recovery data:
    prev = nullptr;
    recovery = spanNext;
    while (recovery)
    {
        next = recovery->Next;

        // If recovered span is disjoint with next packet span:
        if (rightLossSequence < recovery->SequenceStart) {
            break; // Stop here
        }

        // Find remaining lost count in packet overlapping span
        const unsigned lost = GetLostInRange(rightLossSequence, recovery->SequenceEnd);

        // If this recovery packet is still useful:
        if (lost != 0)
        {
            prev = recovery;
            recovery = next;
            continue;
        }

        // Unlink this packet
        if (!prev) {
            spanNext = next;
        }
        else {
            if (next) {
                next->Prev = prev;
            }
            else {
                RecoveryLast = prev;
            }
            prev->Next = next;
        }

        // Release this packet
        AllocPtr->Free(recovery->Data);
        AllocPtr->Destruct(recovery);

        recovery = next;
    }

    // Fix linked list
    if (spanNext) {
        spanNext->Prev = spanPrev;
    }
    else {
        RecoveryLast = spanPrev;
    }
    if (spanPrev) {
        spanPrev->Next = spanNext;
    }
    else {
        RecoveryFirst = spanNext;
    }
}


} // namespace ccat
