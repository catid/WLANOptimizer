/** \file
    \brief Custom Memory Allocator for Packet Data
    \copyright Copyright (c) 2017-2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of PacketAllocator nor the names of its contributors may be
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

#include "PacketAllocator.h"

#include <cstring> // memcpy
#include <cstdlib> // calloc

#if defined(PKTALLOC_ENABLE_ALLOCATOR_INTEGRITY_CHECKS) && defined(PKTALLOC_DEBUG)
    #define ALLOC_DEBUG_INTEGRITY_CHECK() IntegrityCheck();
#else // PKTALLOC_ENABLE_ALLOCATOR_INTEGRITY_CHECKS
    #define ALLOC_DEBUG_INTEGRITY_CHECK() do {} while (false);
#endif // PKTALLOC_ENABLE_ALLOCATOR_INTEGRITY_CHECKS

namespace pktalloc {


//------------------------------------------------------------------------------
// SIMD-Safe Aligned Memory Allocations

static inline uint8_t* SIMDSafeAllocate(size_t size)
{
    uint8_t* data = (uint8_t*)calloc(1, kAlignmentBytes + size);
    if (!data) {
        return nullptr;
    }
    unsigned offset = (unsigned)((uintptr_t)data % kAlignmentBytes);
    data += kAlignmentBytes - offset;
    data[-1] = (uint8_t)offset;
    return data;
}

static inline void SIMDSafeFree(void* ptr)
{
    if (!ptr) {
        return;
    }
    uint8_t* data = (uint8_t*)ptr;
    unsigned offset = data[-1];
    if (offset >= kAlignmentBytes) {
        PKTALLOC_DEBUG_BREAK(); // Should never happen
        return;
    }
    data -= kAlignmentBytes - offset;
    free(data);
}


//------------------------------------------------------------------------------
// Allocator

Allocator::Allocator()
{
    static_assert(kAlignmentBytes == kUnitSize, "update SIMDSafeAllocate");

    PreferredWindows.SetSize_NoCopy(kPreallocatedWindows);

    HugeChunkStart = SIMDSafeAllocate(kWindowSizeBytes * kPreallocatedWindows);
    if (HugeChunkStart)
    {
        uint8_t* windowStart = HugeChunkStart;

        // For each window to preallocate:
        for (unsigned i = 0; i < kPreallocatedWindows; ++i)
        {
            WindowHeader* window = (WindowHeader*)windowStart;
            windowStart += kWindowSizeBytes;

            window->Used.ClearAll();
            window->FreeUnitCount = kWindowMaxUnits;
            window->ResumeScanOffset = 0;
            window->Preallocated = true;
            window->FullListIndex = kNotInFullList;

            PreferredWindows.GetRef(i) = window;
        }
    }

    ALLOC_DEBUG_INTEGRITY_CHECK();
}

Allocator::~Allocator()
{
    for (unsigned i = 0, count = PreferredWindows.GetSize(); i < count; ++i)
    {
        WindowHeader* window = PreferredWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window != nullptr);
        if (window && !window->Preallocated) {
            SIMDSafeFree(window);
        }
    }
    for (unsigned i = 0, count = FullWindows.GetSize(); i < count; ++i)
    {
        WindowHeader* window = FullWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window != nullptr);
        if (window && !window->Preallocated) {
            SIMDSafeFree(window);
        }
    }
    SIMDSafeFree(HugeChunkStart);
}

unsigned Allocator::GetMemoryUsedBytes() const
{
    unsigned sum = 0;
    for (unsigned i = 0, count = PreferredWindows.GetSize(); i < count; ++i)
    {
        WindowHeader* window = PreferredWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window != nullptr);
        if (window) {
            sum += kWindowMaxUnits - window->FreeUnitCount;
        }
    }
    for (unsigned i = 0, count = FullWindows.GetSize(); i < count; ++i)
    {
        WindowHeader* window = FullWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window != nullptr);
        if (window) {
            sum += kWindowMaxUnits - window->FreeUnitCount;
        }
    }
    return sum * kUnitSize;
}

unsigned Allocator::GetMemoryAllocatedBytes() const
{
    return (unsigned)((PreferredWindows.GetSize() + FullWindows.GetSize()) * kWindowMaxUnits * kUnitSize);
}

bool Allocator::IntegrityCheck() const
{
#ifdef PKTALLOC_SHRINK
    PKTALLOC_DEBUG_ASSERT(PreferredWindows.GetSize() >= EmptyWindowCount);
#endif // PKTALLOC_SHRINK

    unsigned emptyCount = 0;
    unsigned preallocatedCount = 0;

    // Check preferred windows list:
    for (unsigned i = 0, icount = PreferredWindows.GetSize(); i < icount; ++i)
    {
        WindowHeader* window = PreferredWindows.GetRef(i);
        if (!window || window->FullListIndex != kNotInFullList)
        {
            PKTALLOC_DEBUG_BREAK(); // Null found
            return false;
        }
        for (unsigned j = 0, jcount = PreferredWindows.GetSize(); j < jcount; ++j)
        {
            WindowHeader* other = PreferredWindows.GetRef(j);
            if (window == other && i != j)
            {
                PKTALLOC_DEBUG_BREAK(); // Duplicate found
                return false;
            }
        }
        if (window->FreeUnitCount <= 0 || window->FreeUnitCount > kWindowMaxUnits)
        {
            PKTALLOC_DEBUG_BREAK(); // Invalid FreeUnitCount
            return false;
        }
        if (window->ResumeScanOffset > kWindowMaxUnits)
        {
            PKTALLOC_DEBUG_BREAK(); // Invalid ResumeScanOffset
            return false;
        }
        const unsigned setCount = window->Used.RangePopcount(0, kWindowMaxUnits);
        if (setCount != kWindowMaxUnits - window->FreeUnitCount)
        {
            PKTALLOC_DEBUG_BREAK(); // Bitfield does not match FreeUnitCount
            return false;
        }
        if (window->Preallocated) {
            ++preallocatedCount;
        }
        else if (window->FreeUnitCount == kWindowMaxUnits) {
            ++emptyCount;
        }
    }

    // Check full windows list:
    for (unsigned i = 0, icount = FullWindows.GetSize(); i < icount; ++i)
    {
        WindowHeader* window = FullWindows.GetRef(i);

        if (!window || window->FullListIndex != (int)i) {
            PKTALLOC_DEBUG_BREAK(); // Null pointer
            return false;
        }
        for (unsigned j = 0, jcount = PreferredWindows.GetSize(); j < jcount; ++j)
        {
            WindowHeader* other = PreferredWindows.GetRef(j);
            if (window == other)
            {
                PKTALLOC_DEBUG_BREAK(); // Duplicate
                return false;
            }
        }
        for (unsigned j = 0, jcount = FullWindows.GetSize(); j < jcount; ++j)
        {
            WindowHeader* other = FullWindows.GetRef(j);
            if (window == other && i != j)
            {
                PKTALLOC_DEBUG_BREAK(); // Duplicate
                return false;
            }
        }
#if 0
        // TBD: Investigate this one further
        if (window->FreeUnitCount > kPreferredThresholdUnits) {
            PKTALLOC_DEBUG_BREAK(); // Invalid FreeUnitCount
            return false;
        }
#endif
        if (window->ResumeScanOffset > kWindowMaxUnits) {
            PKTALLOC_DEBUG_BREAK(); // Invalid ResumeScanOffset
            return false;
        }
        const unsigned setCount = window->Used.RangePopcount(0, kWindowMaxUnits);
        if (setCount != kWindowMaxUnits - window->FreeUnitCount) {
            PKTALLOC_DEBUG_BREAK(); // Bitfield does not match FreeUnitCount
            return false;
        }
        if (window->Preallocated) {
            ++preallocatedCount;
        }
    }

    if (preallocatedCount != kPreallocatedWindows) {
        PKTALLOC_DEBUG_BREAK(); // Lost a preallocated window
        return false;
    }
#ifdef PKTALLOC_SHRINK
    if (emptyCount != EmptyWindowCount) {
        PKTALLOC_DEBUG_BREAK(); // EmptyWindowCount does not match 
        return false;
    }
#endif // PKTALLOC_SHRINK
    return true;
}

uint8_t* Allocator::Allocate(unsigned bytes)
{
    if (bytes <= 0) {
        return nullptr;
    }

    ALLOC_DEBUG_INTEGRITY_CHECK();

    // Calculate number of units required by this allocation
    // Note: +1 for the AllocationHeader
    const unsigned units = (bytes + kUnitSize - 1) / kUnitSize + 1;

    if (units > kFallbackThresholdUnits) {
        return fallbackAllocate(bytes);
    }

    // Check preferred windows list:
    const unsigned preferredCount = PreferredWindows.GetSize();
    for (int i = (int)preferredCount - 1; i >= 0; --i)
    {
        WindowHeader* window = PreferredWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window);

        if (window->FreeUnitCount < units) {
            continue;
        }

        // Walk the holes in the bitmask:
        UsedMaskT* usedPtr = &window->Used;
        unsigned regionStart = window->ResumeScanOffset;
        while (regionStart < UsedMaskT::kValidBits)
        {
            regionStart = usedPtr->FindFirstClear(regionStart);
            unsigned regionEnd = regionStart + units;

            // If we ran out of space:
            if (regionEnd > UsedMaskT::kValidBits) {
                break;
            }

            regionEnd = usedPtr->FindFirstSet(regionStart + 1, regionEnd);
            PKTALLOC_DEBUG_ASSERT(regionEnd > regionStart);
            PKTALLOC_DEBUG_ASSERT(regionEnd <= UsedMaskT::kValidBits);

            if (regionEnd - regionStart < units)
            {
                regionStart = regionEnd + 1;
                continue;
            }
            regionEnd = regionStart + units;

            // Carve out region
            uint8_t* region = (uint8_t*)window + kWindowHeaderBytes + regionStart * kUnitSize;
            AllocationHeader* regionHeader = (AllocationHeader*)region;
#ifdef PKTALLOC_DEBUG
            regionHeader->Canary = AllocationHeader::kCanaryExpected;
#endif // PKTALLOC_DEBUG
            regionHeader->Header = window;
            regionHeader->UsedUnits = units;

            // Update window header
#ifdef PKTALLOC_SHRINK
            if (window->FreeUnitCount >= kWindowMaxUnits &&
                !window->Preallocated)
            {
                PKTALLOC_DEBUG_ASSERT(EmptyWindowCount > 0);
                --EmptyWindowCount;
            }
#endif // PKTALLOC_SHRINK
            window->FreeUnitCount -= units;
            usedPtr->SetRange(regionStart, regionStart + units);
            window->ResumeScanOffset = regionStart + units;

            // Any prior windows that failed to allocate are moved to full list
            unsigned fullCount = preferredCount - 1 - i;

            // Move this window to the full list if we cannot make another allocation of the same size
            if (regionStart + units * 2 > kWindowMaxUnits) {
                // Include this one too
                ++fullCount;
            }

            moveLastFewWindowsToFull(fullCount);

            uint8_t* data = region + kUnitSize;
#ifdef PKTALLOC_SCRUB_MEMORY
            memset(data, 0, (units - 1) * kUnitSize);
#endif // PKTALLOC_SCRUB_MEMORY
            PKTALLOC_DEBUG_ASSERT((uintptr_t)data % kUnitSize == 0);
            PKTALLOC_DEBUG_ASSERT((uint8_t*)regionHeader >= (uint8_t*)regionHeader->Header + kWindowHeaderBytes);
            PKTALLOC_DEBUG_ASSERT(regionHeader->GetUnitStart() < kWindowMaxUnits);
            PKTALLOC_DEBUG_ASSERT(regionHeader->GetUnitStart() + regionHeader->UsedUnits <= kWindowMaxUnits);
            return data;
        }
    }

    // Move all preferred windows to full since none of them worked out
    moveLastFewWindowsToFull(preferredCount);

    return allocateFromNewWindow(units);
}

void Allocator::moveLastFewWindowsToFull(unsigned count)
{
    const unsigned existingFullCount = FullWindows.GetSize();
    FullWindows.SetSize_Copy(existingFullCount + count);

    const unsigned existingPreferredCount = PreferredWindows.GetSize();
    PKTALLOC_DEBUG_ASSERT(existingPreferredCount >= count);

    // For each window to move:
    for (unsigned i = 0; i < count; ++i)
    {
        const unsigned windowIndex = existingPreferredCount - 1 - i;
        PKTALLOC_DEBUG_ASSERT(windowIndex < PreferredWindows.GetSize());
        WindowHeader* window = PreferredWindows.GetRef(windowIndex);
        PKTALLOC_DEBUG_ASSERT(window);

        // Move the window to the full list
        FullWindows.GetRef(existingFullCount + i) = window;
        window->FullListIndex = existingFullCount + i;
    }

    PreferredWindows.SetSize_Copy(existingPreferredCount - count);

    ALLOC_DEBUG_INTEGRITY_CHECK();
}

uint8_t* Allocator::allocateFromNewWindow(unsigned units)
{
    ALLOC_DEBUG_INTEGRITY_CHECK();

    uint8_t* headerStart = SIMDSafeAllocate(kWindowSizeBytes);
    if (!headerStart) {
        return nullptr; // Allocation failure
    }

    // Update window header
    WindowHeader* window = (WindowHeader*)headerStart;
    window->Used.ClearAll();
    window->Used.SetRange(0, units);
    window->FreeUnitCount = kWindowMaxUnits - units;
    window->ResumeScanOffset = units;
    window->Preallocated = false;
    PKTALLOC_DEBUG_ASSERT(PreferredWindows.GetSize() == 0);
    window->FullListIndex = kNotInFullList;
    PreferredWindows.Append(window);

    // Carve out region
    AllocationHeader* regionHeader = (AllocationHeader*)(headerStart + kWindowHeaderBytes);
#ifdef PKTALLOC_DEBUG
    regionHeader->Canary = AllocationHeader::kCanaryExpected;
#endif // PKTALLOC_DEBUG
    regionHeader->Header = window;
    regionHeader->UsedUnits = units;

    uint8_t* data = (uint8_t*)regionHeader + kUnitSize;
#ifdef PKTALLOC_SCRUB_MEMORY
    memset(data, 0, (units - 1) * kUnitSize);
#endif // PKTALLOC_SCRUB_MEMORY
    PKTALLOC_DEBUG_ASSERT((uintptr_t)data % kUnitSize == 0);

    ALLOC_DEBUG_INTEGRITY_CHECK();

    return data;
}

uint8_t* Allocator::Reallocate(uint8_t* ptr, unsigned bytes, Realloc behavior)
{
    ALLOC_DEBUG_INTEGRITY_CHECK();

    if (!ptr) {
        return Allocate(bytes);
    }
    if (bytes <= 0) {
        Free(ptr);
        return nullptr;
    }
    PKTALLOC_DEBUG_ASSERT((uintptr_t)ptr % kUnitSize == 0);

    AllocationHeader* regionHeader = (AllocationHeader*)(ptr - kUnitSize);
#ifdef PKTALLOC_DEBUG
    if (regionHeader->Canary != AllocationHeader::kCanaryExpected) {
        PKTALLOC_DEBUG_BREAK(); // Buffer overflow detected
        return nullptr;
    }
    if (regionHeader->IsFreed()) {
        PKTALLOC_DEBUG_BREAK(); // Double-free
        return nullptr;
    }
#endif // PKTALLOC_DEBUG

    const unsigned existingUnits = regionHeader->UsedUnits;
#ifndef PKTALLOC_DISABLE
    PKTALLOC_DEBUG_ASSERT(!regionHeader->Header || existingUnits <= kFallbackThresholdUnits);
#endif // PKTALLOC_DISABLE

    // If the existing allocation is big enough:
    const unsigned requestedUnits = (bytes + kUnitSize - 1) / kUnitSize + 1;
    if (requestedUnits <= existingUnits) {
        return ptr; // No change needed
    }

    // Allocate new data
    uint8_t* newPtr = Allocate(bytes);
    if (!newPtr) {
        return nullptr;
    }

    // Copy old data
    if (behavior == Realloc::CopyExisting) {
        memcpy(newPtr, ptr, (existingUnits - 1) * kUnitSize);
    }

    Free(ptr);

    ALLOC_DEBUG_INTEGRITY_CHECK();

    return newPtr;
}

void Allocator::Shrink(uint8_t* ptr, unsigned bytes)
{
    ALLOC_DEBUG_INTEGRITY_CHECK();

    if (!ptr) {
        return;
    }
    PKTALLOC_DEBUG_ASSERT((uintptr_t)ptr % kUnitSize == 0);

    AllocationHeader* regionHeader = (AllocationHeader*)(ptr - kUnitSize);
#ifdef PKTALLOC_DEBUG
    if (regionHeader->Canary != AllocationHeader::kCanaryExpected) {
        PKTALLOC_DEBUG_BREAK(); // Buffer overflow detected
        return;
    }
#endif // PKTALLOC_DEBUG
    if (regionHeader->IsFreed()) {
        PKTALLOC_DEBUG_BREAK(); // Double-free
        return;
    }

    // Calculate number of units required by this allocation
    // Note: +1 for the AllocationHeader
    const unsigned unitsNeeded = (bytes + kUnitSize - 1) / kUnitSize + 1;
    const unsigned unitsCurrent = regionHeader->UsedUnits;

    // If the allocation can shrink:
    if (unitsNeeded < unitsCurrent)
    {
        WindowHeader* windowHeader = regionHeader->Header;
        if (!windowHeader) {
            // Fallback allocation: We cannot resize this region
            return;
        }

        PKTALLOC_DEBUG_ASSERT((uint8_t*)regionHeader >= (uint8_t*)regionHeader->Header + kWindowHeaderBytes);
        const unsigned regionStart = regionHeader->GetUnitStart();
        PKTALLOC_DEBUG_ASSERT(regionStart < kWindowMaxUnits);
        PKTALLOC_DEBUG_ASSERT(regionStart + regionHeader->UsedUnits <= kWindowMaxUnits);

        const unsigned regionEndNew = regionStart + unitsNeeded;
        PKTALLOC_DEBUG_ASSERT(regionEndNew <= kWindowMaxUnits);
        PKTALLOC_DEBUG_ASSERT((regionStart + unitsCurrent) > regionEndNew);

        // Resume scanning from this hole next time
        if (windowHeader->ResumeScanOffset > regionEndNew) {
            windowHeader->ResumeScanOffset = regionEndNew;
        }

        // Clear the units we gave up
        windowHeader->Used.ClearRange(regionEndNew, regionStart + unitsCurrent);

        // Give back the unit count
        windowHeader->FreeUnitCount += unitsCurrent - unitsNeeded;

        // Update unit count
        regionHeader->UsedUnits = unitsNeeded;

        // Note that this will not move a full window to the empty list,
        // but when this allocation is freed it may be added later.
    }

    ALLOC_DEBUG_INTEGRITY_CHECK();
}

void Allocator::Free(uint8_t* ptr)
{
    ALLOC_DEBUG_INTEGRITY_CHECK();

    if (!ptr) {
        return;
    }
    PKTALLOC_DEBUG_ASSERT((uintptr_t)ptr % kUnitSize == 0);

    AllocationHeader* regionHeader = (AllocationHeader*)(ptr - kUnitSize);
#ifdef PKTALLOC_DEBUG
    if (regionHeader->Canary != AllocationHeader::kCanaryExpected) {
        PKTALLOC_DEBUG_BREAK(); // Buffer overflow detected
        return;
    }
#endif // PKTALLOC_DEBUG
    if (regionHeader->IsFreed()) {
        PKTALLOC_DEBUG_BREAK(); // Double-free
        return;
    }

    WindowHeader* window = regionHeader->Header;
    if (!window)
    {
        regionHeader->UsedUnits = 0; // Mark freed
        fallbackFree(ptr);
        return;
    }

    const unsigned units = regionHeader->UsedUnits;
    PKTALLOC_DEBUG_ASSERT(units >= 2 && units <= kFallbackThresholdUnits);
    regionHeader->UsedUnits = 0; // Mark freed

    PKTALLOC_DEBUG_ASSERT((uint8_t*)regionHeader >= (uint8_t*)regionHeader->Header + kWindowHeaderBytes);
    unsigned regionStart = regionHeader->GetUnitStart();
    PKTALLOC_DEBUG_ASSERT(regionStart < kWindowMaxUnits);
    PKTALLOC_DEBUG_ASSERT(regionStart + units <= kWindowMaxUnits);

    unsigned regionEnd = regionStart + units;

    // Resume scanning from this hole next time
    if (window->ResumeScanOffset > regionStart) {
        window->ResumeScanOffset = regionStart;
    }

    // Clear the units it was using
    window->Used.ClearRange(regionStart, regionEnd);

    // Give back the unit count
    window->FreeUnitCount += units;

    // If we may want to promote this to Preferred:
    if (window->FreeUnitCount >= kPreferredThresholdUnits &&
        window->FullListIndex != kNotInFullList)
    {
        // Restart scanning from the front
        window->ResumeScanOffset = 0;

        // Swap last item in full list into the slot
        const unsigned fullCount = FullWindows.GetSize();
        const unsigned fullIndex = (unsigned)window->FullListIndex;
        PKTALLOC_DEBUG_ASSERT(fullIndex < fullCount);
        PKTALLOC_DEBUG_ASSERT(fullCount > 0);
        if (fullIndex < fullCount - 1)
        {
            WindowHeader* replacement = FullWindows.GetRef(fullCount - 1);
            FullWindows.GetRef(fullIndex) = replacement;
            replacement->FullListIndex = fullIndex;
        }
        FullWindows.SetSize_Copy(fullCount - 1);

        window->FullListIndex = kNotInFullList;
        PreferredWindows.Append(window);
    }

#ifdef PKTALLOC_SHRINK
    // If we should do some bulk cleanup:
    if (window->FreeUnitCount >= kWindowMaxUnits &&
        !window->Preallocated &&
        ++EmptyWindowCount >= kEmptyWindowCleanupThreshold)
    {
        freeEmptyWindows();
    }
#endif // PKTALLOC_SHRINK

    ALLOC_DEBUG_INTEGRITY_CHECK();
}

#ifdef PKTALLOC_SHRINK

void Allocator::freeEmptyWindows()
{
    if (EmptyWindowCount <= kEmptyWindowMinimum) {
        return;
    }

    ALLOC_DEBUG_INTEGRITY_CHECK();

    // Check full windows list:
    unsigned count = PreferredWindows.GetSize();
    for (unsigned i = 0; i < count;)
    {
        WindowHeader* window = PreferredWindows.GetRef(i);
        PKTALLOC_DEBUG_ASSERT(window && window->FullListIndex == kNotInFullList);

        // If this window cannot be reclaimed:
        if (window->FreeUnitCount < kWindowMaxUnits ||
            window->Preallocated)
        {
            ++i;
            continue;
        }

        SIMDSafeFree(window);

        PKTALLOC_DEBUG_ASSERT(count > 0);
        --count;

        PreferredWindows.GetRef(i) = PreferredWindows.GetRef(count);

        PKTALLOC_DEBUG_ASSERT(EmptyWindowCount > 0);
        if (--EmptyWindowCount <= kEmptyWindowMinimum) {
            break;
        }
    }
    PreferredWindows.SetSize_Copy(count);

    ALLOC_DEBUG_INTEGRITY_CHECK();
}

#endif // PKTALLOC_SHRINK

uint8_t* Allocator::fallbackAllocate(unsigned bytes)
{
    // Calculate number of units required by this allocation
    // Note: +1 for the AllocationHeader
    const unsigned units = (bytes + kUnitSize - 1) / kUnitSize + 1;

    uint8_t* ptr = SIMDSafeAllocate(kUnitSize * units);
    if (!ptr) {
        return nullptr;
    }

    AllocationHeader* regionHeader = (AllocationHeader*)ptr;
#ifdef PKTALLOC_DEBUG
    regionHeader->Canary = AllocationHeader::kCanaryExpected;
#endif // PKTALLOC_DEBUG
    regionHeader->Header = nullptr;
    regionHeader->UsedUnits = units;

    return ptr + kUnitSize;
}

void Allocator::fallbackFree(uint8_t* ptr)
{
    PKTALLOC_DEBUG_ASSERT(ptr);
    SIMDSafeFree(ptr - kUnitSize);
}


} // namespace pktalloc
