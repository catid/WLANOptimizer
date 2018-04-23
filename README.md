# WLANOptimizer
Single-header C library that fixes WiFi performance issues for online gaming and other low-latency real-time network traffic.

This fixes a common issue on Windows laptops where the adapter scans for
networks while a connection is active, causing 100+ millisecond delays.
This makes a huge difference in performance for real-time multiplayer!

On my laptop it reduces the latency from 130 ms delay spikes to 10 ms or less - 10x improvement.

Notably this also improves my BitTorrent download speed by 4x, as BitTorrent protocol uses a congestion control strategy based on latency (LEDBAT).  These latency spikes cause download speed to plummet in easy, repeatable tests.

Note: This settings change DOES NOT require Administrator access.

Easy setup:

```
    Add the include header somewhere:

        #include "WLANOptimizer.h"

    Call the start function:

        StartWLANOptimizerThread();

```

And that's it.


#### Demo

To try it out quickly, run the `tests/stand_alone.exe` application.

To set up a more thorough demo between two computers on your LAN, run the `tests/tests_peer2peer_gui.exe` on two computers on your WiFi LAN.

The results on my WLAN:

![demo.png](https://raw.githubusercontent.com/catid/WLANOptimizer/master/demo.png)


#### Credits

Software by Christopher A. Taylor mrcatid@gmail.com

Please reach out if you need support or would like to collaborate on a project.
