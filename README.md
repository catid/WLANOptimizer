# WLANOptimizer
Single-header C++ library that fixes WiFi performance issues for online gaming and other low-latency real-time network traffic.

Presents the OptimizeWLAN() function to optimize WiFi settings for low-latency.

* This function takes about 1 second to execute.
* This setting change requires Administrator access.
* This setting change resets when the app closes.

This fixes a common issue on Windows laptops where the adapter scans for
networks while a connection is active, causing 100+ millisecond delays.
This makes a huge difference in performance for real-time multiplayer!

On my laptop it reduces the latency from 130 ms delay spikes to 10 ms or less - 10x improvement.

Example code:

```
    OptimizeWLAN_Result result = OptimizeWLAN(enable);
    if (result != OptimizeWLAN_Result::Success)
        return false;
    return true;
```
