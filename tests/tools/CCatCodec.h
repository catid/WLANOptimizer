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

#pragma once

#include "ccat.h"
#include "gf256.h"
#include "Counter.h"
#include "PacketAllocator.h"

#include <stdint.h> // uint32_t
#include <string.h> // memcpy
#include <new> // std::nothrow


namespace ccat {


//------------------------------------------------------------------------------
// Constants

/// Define this to use less memory but a bit more CPU
//#define CCAT_FREE_UNUSED_PACKETS

/// Max original columns in matrix
/// 1.3333.. * x = 256, x = 192, Enables up to 33% FEC
/// This is also a multiple of 64 which makes the most of bitfields
static const unsigned kMatrixColumnCount = 192;
static_assert(kMatrixColumnCount < 256, "Too large");

/// Max recovery rows in matrix
static const unsigned kMatrixRowCount = 256 - kMatrixColumnCount;

/// Limit the size of a recovery attempt
static const unsigned kMaxRecoveryColumns = 128;

/// Limit the size of involved recovery rows
static const unsigned kMaxRecoveryRows = kMaxRecoveryColumns + 32;
static_assert(kMaxRecoveryRows > kMaxRecoveryColumns, "Update this too");

/// Min encoder window size
static const unsigned kMinEncoderWindowSize = 1;

/// Max encoder window size
static const unsigned kMaxEncoderWindowSize = kMatrixColumnCount;
static_assert(kMaxEncoderWindowSize == CCAT_MAX_WINDOW_PACKETS, "Header mismatch");

/// Max decoder window size
static const unsigned kDecoderWindowSize = 2 * kMatrixColumnCount;

/// Max packet size
static const unsigned kMaxPacketSize = 65536;
static_assert(kMaxPacketSize == CCAT_MAX_BYTES, "Header mismatch");

/// Min window size in msec
static const unsigned kMinWindowMsec = 10;
static_assert(kMinWindowMsec == CCAT_MIN_WINDOW_MSEC, "Header mismatch");

/// Max window size in msec
static const unsigned kMaxWindowMsec = 2000 * 1000; // Int32 limits

/// Encode overhead
static const unsigned kEncodeOverhead = 2;


//------------------------------------------------------------------------------
// Timing

/// Platform independent high-resolution timers
uint64_t GetTimeUsec();
uint64_t GetTimeMsec();


//------------------------------------------------------------------------------
// POD Serialization

PKTALLOC_FORCE_INLINE uint16_t ReadU16_LE(const uint8_t* data)
{
#ifdef GF256_ALIGNED_ACCESSES
    return ((uint16_t)data[1] << 8) | data[0];
#else
    return *(uint16_t*)data;
#endif
}

PKTALLOC_FORCE_INLINE void WriteU16_LE(uint8_t* data, uint16_t value)
{
#ifdef GF256_ALIGNED_ACCESSES
    data[1] = (uint8_t)(value >> 8);
    data[0] = (uint8_t)value;
#else
    *(uint16_t*)data = value;
#endif
}


//------------------------------------------------------------------------------
// AlignedLightVector

/**
    Super light-weight replacement for std::vector class

    Features:
    + SIMD aligned buffer.
    + Tuned for CCat allocation needs.
    + Never shrinks memory usage.
    + Minimal well-defined API: Only functions used several times.
    + Growing the vector does not initialize the new elements for speed.
    + Does not throw on out-of-memory error.
*/
class AlignedLightVector
{
    /// Vector data
    uint8_t* DataPtr = nullptr;

    /// Size of vector: Count of elements in the vector
    unsigned Size = 0;

    /// Number of elements allocated
    unsigned Allocated = 0;

public:
    /// Resize to target size.
    /// Uses the provided allocator and does not automatically free on dtor
    bool Resize(
        pktalloc::Allocator* alloc,
        unsigned elements,
        pktalloc::Realloc behavior);

    /// Get current size (initially 0)
    PKTALLOC_FORCE_INLINE unsigned GetSize() const
    {
        return Size;
    }

    /// Return a pointer to an element
    PKTALLOC_FORCE_INLINE uint8_t* GetPtr(int index = 0) const
    {
        return DataPtr + index;
    }
};


//------------------------------------------------------------------------------
// Cauchy Matrix Math

/*
    GF(256) Cauchy Matrix Overview

    As described on Wikipedia, each element of a normal Cauchy matrix is defined as:

        a_ij = 1 / (x_i - y_j)
        The arrays x_i and y_j are vector parameters of the matrix.
        The values in x_i cannot be reused in y_j.

    Moving beyond the Wikipedia...

    (1) Number of rows (R) is the range of i, and number of columns (C) is the range of j.

    (2) Being able to select x_i and y_j makes Cauchy matrices more flexible in practice
        than Vandermonde matrices, which only have one parameter per row.

    (3) Cauchy matrices are always invertible, AKA always full rank, AKA when treated as
        as linear system y = M*x, the linear system has a single solution.

    (4) A Cauchy matrix concatenated below a square CxC identity matrix always has rank C,
        Meaning that any R rows can be eliminated from the concatenated matrix and the
        matrix will still be invertible.  This is how Reed-Solomon erasure codes work.

    (5) Any row or column can be multiplied by non-zero values, and the resulting matrix
        is still full rank.  This is true for any matrix, since it is effectively the same
        as pre and post multiplying by diagonal matrices, which are always invertible.

    (6) Matrix elements with a value of 1 are much faster to operate on than other values.
        For instance a matrix of [1, 1, 1, 1, 1] is invertible and much faster for various
        purposes than [2, 2, 2, 2, 2].

    (7) For GF(256) matrices, the symbols in x_i and y_j are selected from the numbers
        0...255, and so the number of rows + number of columns may not exceed 256.
        Note that values in x_i and y_j may not be reused as stated above.

    In summary, Cauchy matrices
        are preferred over Vandermonde matrices.  (2)
        are great for MDS erasure codes.  (3) and (4)
        should be optimized to include more 1 elements.  (5) and (6)
        have a limited size in GF(256), rows+cols <= 256.  (7)
*/

/*
    Selected Cauchy Matrix Form

    The matrix consists of elements a_ij, where i = row, j = column.
    a_ij = 1 / (x_i - y_j), where x_i and y_j are sets of GF(256) values
    that do not intersect.

    We select x_i and y_j to just be incrementing numbers for the
    purposes of this library.  Further optimizations may yield matrices
    with more 1 elements, but the benefit seems relatively small.

    The x_i values range from 0...(originalCount - 1).
    The y_j values range from originalCount...(originalCount + recoveryCount - 1).

    We then improve the Cauchy matrix by dividing each column by the
    first row element of that column.  The result is an invertible
    matrix that has all 1 elements in the first row.  This is equivalent
    to a rotated Vandermonde matrix, so we could have used one of those.

    The advantage of doing this is that operations involving the first
    row will be extremely fast (just memory XOR), so the decoder can
    be optimized to take advantage of the shortcut when the first
    recovery row can be used.

    First row element of Cauchy matrix for each column:
    a_0j = 1 / (x_0 - y_j) = 1 / (x_0 - y_j)

    Our Cauchy matrix sets first row to ones, so:
    a_ij = (1 / (x_i - y_j)) / a_0j
    a_ij = (y_j - x_0) / (x_i - y_j)
    a_ij = (y_j + x_0) div (x_i + y_j) in GF(256)
*/

// This function generates each matrix element based on x_i, x_0, y_j
// Note that for x_i == x_0, this will return 1, so it is better to unroll out the first row.
// This is specialized for x_0 = 0.  So x starts at 0 and y starts at x + CountXValues.
static GF256_FORCE_INLINE uint8_t GetMatrixElement(
    uint8_t recoveryRow,
    uint8_t originalColumn)
{
    const uint8_t x_i = recoveryRow;
    const uint8_t y_j = originalColumn + (uint8_t)kMatrixRowCount;
    PKTALLOC_DEBUG_ASSERT(x_i < y_j);
    const uint8_t result = gf256_div(y_j, gf256_add(x_i, y_j));
    PKTALLOC_DEBUG_ASSERT(result != 0);
    return result;
}


//------------------------------------------------------------------------------
// EncoderWindowElement

struct EncoderWindowElement
{
    // Send time for this packet
    Counter64 SendUsec = 0;

    // Data for packet that is prepended with data size
    AlignedLightVector Data;
};


//------------------------------------------------------------------------------
// Encoder

class Encoder
{
public:
    // Dependencies
    const CCatSettings* SettingsPtr = nullptr;
    pktalloc::Allocator* AllocPtr = nullptr;

    // API
    CCatResult EncodeOriginal(const CCatOriginal& original);
    CCatResult EncodeRecovery(CCatRecovery& recoveryOut);

private:
    /// Preallocated window of packets
    EncoderWindowElement Window[kMaxEncoderWindowSize];

    /// Next window index to write to
    unsigned NextIndex = 0;

    /// Count of window elements
    unsigned Count = 0;

    /// Recovery packet generated by EncodeRecovery()
    AlignedLightVector RecoveryData;

    /// Next original packet sequence number
    Counter64 NextSequence = 0;

    /// Next matrix column
    uint8_t NextColumn = 0;

    /// Next matrix row to generate in EncodeRecovery()
    uint8_t NextRow = 1;

    /// Next sequence number that will use xor parity
    Counter64 NextParitySequence = 0;

    /// Last time an original packet was passed to EncodeOriginal()
    Counter64 LastOriginalSendUsec = 0;
};


//------------------------------------------------------------------------------
// RecoveryPacket

struct RecoveryPacket
{
    /// Next recovery packet in the sorted list with a higher sequence number
    RecoveryPacket* Next = nullptr;

    /// Previous recovery packet in the sorted list with a lower sequence number
    RecoveryPacket* Prev = nullptr;

    /// Recovery packet data.  Allocated with PacketAllocator
    uint8_t* Data = nullptr;

    /// Bytes in packet data
    unsigned Bytes = 0;

    /// Start of recovery span
    Counter64 SequenceStart = 0;

    /// End of recovery span (non-inclusive) = Last sequence number + 1
    Counter64 SequenceEnd = 0;

    /// Matrix row number
    uint8_t MatrixRow = 0;
};


//------------------------------------------------------------------------------
// OriginalPacket

struct OriginalPacket
{
    /// Pointer to packet data prepended with length field
    uint8_t* Data = nullptr;

    /// Bytes of data including the prepended length field
    unsigned Bytes = 0;
};


//------------------------------------------------------------------------------
// Decoder

/**
    Tracks losses in the decoder original packet window in a compact, efficient
    way.  Alternative approaches are packed arrays that need to be recreated on
    new data, linked lists that have poor cache locality, or scanning for the
    sparse losses in the array of original data.  The Loss Window uses one bit
    for each original packet and provides methods that are useful for checking
    for possible solutions when a large number of recovery packets is received.

    Newer packets have higher sequence numbers.
    In the bitfield, the first word contains the lowest window sequence number.
    And the first bit of the first word indicates that the lowest sequence
    number is lost if it is a '1'.

    When the window is shifted to accomodate newer packets, it shifts the high
    bits into the lower bits.  Instead of shifting with every new packet, it
    will only shift in multiples of 64 bits to simplify this maintenance.
*/

class Decoder
{
public:
    const CCatSettings* SettingsPtr = nullptr;
    pktalloc::Allocator* AllocPtr = nullptr;
    // Note: Allocator automatically frees all allocated memory on dtor
    // so there is no need to explicitly free any memory on dtor.

    CCatResult DecodeOriginal(const CCatOriginal& original);
    CCatResult DecodeRecovery(const CCatRecovery& recovery);

    PKTALLOC_FORCE_INLINE Decoder()
    {
        // All packets are lost initially
        Lost.SetAll();
    }

private:
    //--------------------------------------------------------------------------
    // Original/recovery data:

    /// Bitfield - 1 bits mean a loss at that offset from SequenceBase.
    /// Bits we have not received yet will also be marked with a 1.
    pktalloc::CustomBitSet<kDecoderWindowSize> Lost;

    /// Ring buffer of packet data
    OriginalPacket Packets[kDecoderWindowSize];

    /// Rotation of packets ring buffer
    unsigned PacketsRotation = 0;

    /// First sequence number in the window
    Counter64 SequenceBase = 0;

    /// Largest sequence number in the window + 1
    Counter64 SequenceEnd = 0;

    /// Recovery packet with the smallest sequence number.
    /// Stores up to kMaxDecoderRows recovery rows
    RecoveryPacket* RecoveryFirst = nullptr;

    /// Recovery packet with the largest sequence number
    RecoveryPacket* RecoveryLast = nullptr;


    //--------------------------------------------------------------------------
    // Solver state for 2+ losses recovered at a time:

    /// Number of bytes used for each recovery packet in the matrix.
    /// This is also the maximum size of all recovery packets in the set
    unsigned SolutionBytes = 0;

    /// Number of rows in matrix <= kMaxRecoveryRows
    unsigned RowCount = 0;

    /// Solver state: Information about recovery data for each row
    struct {
        /// Recovery packet for this row
        RecoveryPacket* Recovery;

        /// First lost column this one has
        unsigned ColumnStart;

        /// One beyond the last lost column this one covers
        unsigned ColumnEnd;
    } RowInfo[kMaxRecoveryColumns];

    /// Number of columns in matrix <= kMaxRecoveryColumns
    unsigned ColumnCount = 0;

    /// Solver state: Information about original data for each column
    struct {
        /// Sequence number for this original packet
        Counter64 Sequence;

        /// Pointer to original packet we will modify in-place
        OriginalPacket* OriginalPtr;
    } ColumnInfo[kMaxRecoveryRows];

    /// Generator row values
    uint8_t CauchyRows[kMaxRecoveryRows];

    /// Generator column values
    uint8_t CauchyColumns[kMaxRecoveryColumns];

    /// Solution matrix
    AlignedLightVector Matrix;

    /// Pivot row index for each matrix column
    uint8_t PivotRowIndex[kMaxRecoveryColumns];

    /// Data that starts out as per-row data but becomes solved column data
    uint8_t* DiagonalData[kMaxRecoveryColumns];


    //--------------------------------------------------------------------------
    // Statistics:

    /// Sequence number we failed to recover
    Counter64 FailureSequence = 0;

    /// Number of 2x2 or larger solves that succeeded
    uint64_t LargeRecoverySuccesses = 0;

    /// Number of 2x2 or larger solves that failed
    uint64_t LargeRecoveryFailures = 0;


    //--------------------------------------------------------------------------
    // Original/recovery data:

    /// Expand window to contain given original data span.
    enum class Expand
    {
        InWindow,
        OutOfWindow,
        Evacuated,
        Shifted
    };
    Expand ExpandWindow(Counter64 sequenceStart, unsigned count = 1);

    /// Clear recovery list
    void ClearRecoveryList();

    /// Remove recovery packets from the front that reference unavailable data
    void CleanupRecoveryList();

    /// Look up packet at a given 0-based element.
    /// Applies Rotation to the ring buffer to arrive at the actual location.
    PKTALLOC_FORCE_INLINE OriginalPacket* GetPacket(unsigned element)
    {
        PKTALLOC_DEBUG_ASSERT(element < kDecoderWindowSize);
        PKTALLOC_DEBUG_ASSERT(PacketsRotation < kDecoderWindowSize);

        // Increment element by rotation modulo window size
        element += PacketsRotation;
        if (element >= kDecoderWindowSize)
            element -= kDecoderWindowSize;

        return &Packets[element];
    }

    /// Store original data in window
    CCatResult StoreOriginal(const CCatOriginal& original);

    /// Insert recovery packet into sorted list
    CCatResult StoreRecovery(const CCatRecovery& recovery);

    PKTALLOC_FORCE_INLINE unsigned GetLostInRange(
        Counter64 sequenceStart,
        Counter64 sequenceEnd)
    {
        PKTALLOC_DEBUG_ASSERT(sequenceStart >= SequenceBase);
        PKTALLOC_DEBUG_ASSERT(sequenceEnd <= SequenceEnd);
        const unsigned start = (unsigned)(sequenceStart - SequenceBase).ToUnsigned();
        const unsigned end = (unsigned)(sequenceEnd - SequenceBase).ToUnsigned();
        return Lost.RangePopcount(start, end);
    }

    /// Solve common case when recovery row is only missing one original
    CCatResult SolveLostOne(const CCatRecovery& recovery);

    /// Check for a recovery packet in the list containing the given sequence
    /// number which now references just one loss.  Then call FindSolutions().
    CCatResult FindSolutionsContaining(const Counter64 sequence);


    //--------------------------------------------------------------------------
    // Solver state for 2+ losses recovered at a time:

    /// Find solutions starting from the right (latest) side of the matrix
    CCatResult FindSolutions();

    /// Solve the given span
    CCatResult Solve(RecoveryPacket* spanStart, RecoveryPacket* spanEnd);

    /// Generate arrays from spans as a first step to planning a solution
    CCatResult ArraysFromSpans(RecoveryPacket* spanStart, RecoveryPacket* spanEnd);

    /// Plan out steps that will recover the original data.
    /// Return CCat_Success if solution is possible (99.9% of the time).
    /// Return CCat_NeedsMoreData if given rows are insufficient to find a solution
    CCatResult PlanSolution();

    /// Gaussian elimination to put matrix in upper triangular form
    CCatResult ResumeGaussianElimination(
        uint8_t* pivot_data,
        unsigned pivotColumn);

    /// If a diagonal was zero, attempt to re-order the rows to solve it.
    CCatResult PivotedGaussianElimination(unsigned pivotColumn);

    /// Eliminate received original data from recovery rows used for solution.
    /// This is done after PlanSolution() and before ExecuteSolution()
    CCatResult EliminateOriginals();

    /// Execute solution plan to recover the original data
    void ExecuteSolutionPlan();

    /// Report solution to app
    CCatResult ReportSolution();

    /// Release exhausted recovery span
    void ReleaseSpan(
        RecoveryPacket* spanStart,
        RecoveryPacket* spanEnd,
        CCatResult solveResult);
};


//------------------------------------------------------------------------------
// Codec

class Codec
    : public Encoder
    , public Decoder
{
public:
    CCatResult Create(const CCatSettings& settings);

private:
    CCatSettings Settings;
    pktalloc::Allocator Alloc;
};


} // namespace ccat
