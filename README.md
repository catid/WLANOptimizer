# WLANOptimizer
Single-header C library that fixes WiFi performance issues for online gaming and other low-latency real-time network traffic.

Presents the OptimizeWLAN() function to optimize WiFi settings for low-latency.

* This function takes about 1 second to execute.
* This setting change resets when the app closes.
* This can only change WiFi adapters with an active connection.

Note: This setting change DOES NOT require Administrator access.

This fixes a common issue on Windows laptops where the adapter scans for
networks while a connection is active, causing 100+ millisecond delays.
This makes a huge difference in performance for real-time multiplayer!

On my laptop it reduces the latency from 130 ms delay spikes to 10 ms or less - 10x improvement.

Example code:

```
    int result = OptimizeWLAN(1);

    if (result != OptimizeWLAN_Success)
    {
        if (result == OptimizeWLAN_NoConnections)
        {
            // No WiFi connections are available to modify - App should retry later.
        }
        return false;
    }
    return true;
```


#### Credits

Software by Christopher A. Taylor mrcatid@gmail.com

Please reach out if you need support or would like to collaborate on a project.
