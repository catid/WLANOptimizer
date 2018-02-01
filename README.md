# WLANOptimizer
Single-header C library that fixes WiFi performance issues for online gaming and other low-latency real-time network traffic.

This fixes a common issue on Windows laptops where the adapter scans for
networks while a connection is active, causing 100+ millisecond delays.
This makes a huge difference in performance for real-time multiplayer!

On my laptop it reduces the latency from 130 ms delay spikes to 10 ms or less - 10x improvement.

Note: This settings change DOES NOT require Administrator access.

Easy setup:

```
    Add the include header somewhere:

        #include "WLANOptimizer.h"

    Call the start function:

        StartWLANOptimizerThread();

```

And that's it.


#### Credits

Software by Christopher A. Taylor mrcatid@gmail.com

Please reach out if you need support or would like to collaborate on a project.
