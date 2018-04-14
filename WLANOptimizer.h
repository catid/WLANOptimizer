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

#ifndef WLAN_OPTIMIZER_H
#define WLAN_OPTIMIZER_H

/**
    Easy setup:

    Add the include header somewhere:

        #include "WLANOptimizer.h"

    Call the start function:

        StartWLANOptimizerThread();

    And that's it.
*/

/// Result codes for OptimizeWLAN()
typedef enum OptimizeWLAN_Result_t
{
    OptimizeWLAN_Success,         ///< Successfully applied changes

    OptimizeWLAN_NoConnections,   ///< No active WiFi connections to modify

    OptimizeWLAN_Unavailable,     ///< Feature is not available on the operating system
    OptimizeWLAN_AccessDenied,    ///< Must be run as Administrator
    OptimizeWLAN_SecurityFailure, ///< WlanSetSecuritySettings failed
    OptimizeWLAN_SetFailure,      ///< WlanSetInterface failed
    OptimizeWLAN_ReadFailure,     ///< Readback failed
} OptimizeWLAN_Result;

/**
    OptimizeWLAN()

    Optimize WiFi settings for low-latency.

    ***************************************************************
    * This function takes about 1 second to execute.
    * This settings change resets when the app closes.
    * This can only change WiFi adapters with an active connection.
    * This settings change resets if the WiFi adapter reconnects.
    ***************************************************************

    Note: This settings change DOES NOT require that the application has
    Administrator access (e.g. no need to elevate with UAC), but it does
    require that the logged-in user has an attached Administrator token
    (e.g. they could elevate with UAC if they wanted to).

    This fixes a common issue on Windows laptops where the adapter scans for
    networks while a connection is active, causing 100+ millisecond delays.
    This makes a huge difference in performance for real-time multiplayer!

    enable: Set to 1 to enable the optimizations.
    Set to 0 to disable the optimizations when finished.

    Returns OptimizeWLAN_Success on success.
    Returns other values on failure.

    If it returns OptimizeWLAN_NoConnections then the application may want
    to try again later when a connection is available.  This can also be
    used to check when a connection becomes available.
*/
int OptimizeWLAN(int enable);


/**
    StartWLANOptimizerThread()

    Running OptimizeWLAN() in a thread solves all the edge cases:

    (1) OptimizeWLAN() takes 1 second to run, so this avoids blocking anything.
    (2) If WiFi reconnects the settings are lost and must be reapplied.
    (3) If WiFi connects for the first time after app start, this will catch it.

    This is a background thread that runs the WLANOptimizer every 11 seconds.
    If we see some kind of lag spike every 11 seconds this might be the culprit.

    The goal is to keep refreshing the WiFi settings that disable background
    scanning and enable media streaming mode, so that if the app starts and no
    WiFi connection is available initially, or if WiFi goes down and connects
    again while the app is running, the app will continue to have low latency.

    It runs in a background thread because the function takes 1 second to run.
    This is how I'm using WLANOptimizer in my own netcode.
*/

void StartWLANOptimizerThread();
void StopWLANOptimizerThread();

#endif // WLAN_OPTIMIZER_H
