/** \file
    \brief WLAN Optimizer
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of WLAN Optimizer nor the names of its contributors may be
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

enum class OptimizeWLAN_Result
{
    Success,

    Unavailable,        // Feature is not available on the operating system
    AccessDenied,       // Must be run as Administrator
    SecurityFailure,    // WlanSetSecuritySettings failed
    SetFailure,         // WlanSetInterface failed
    ReadFailure,        // Readback failed
};

/**
    OptimizeWLAN()

    Optimize WiFi settings for low-latency.

    **************************************************
    This function takes about 1 second to execute.
    This setting change requires Administrator access.
    This setting change resets when the app closes.
    **************************************************

    This fixes a common issue on Windows laptops where the adapter scans for
    networks while a connection is active, causing 100+ millisecond delays.
    This makes a huge difference in performance for real-time multiplayer!

    enable: Set to true to enable the optimizations.
    Set to false to disable the optimizations when finished.

    Returns true on success, false on failure.
*/
OptimizeWLAN_Result OptimizeWLAN(bool enable);
