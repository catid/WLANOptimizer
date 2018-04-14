#include "../WLANOptimizer.h"

#include <thread>
#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono_literals;


//#define OPT_BASIC_VERSION
#define OPT_ADVANCED_VERSION



#ifdef OPT_BASIC_VERSION

int main()
{
    StartWLANOptimizerThread();

    cout << "WLANOptimizer is running.  Close this application to deactivate it." << endl;
    for (;;) {
        std::this_thread::sleep_for(1000ms);
    }
    return 0;
}

#endif


#ifdef OPT_ADVANCED_VERSION

int main()
{
    cout << "WLANOptimizer is running.  Close this application to deactivate it." << endl;

    // Time between optimization attempts.  Retries because WiFi might reconnect.
    static const auto kOptimizeInterval = 11s; // chrono_literals

    for (;;)
    {
        int optimizeResult = OptimizeWLAN(1);

        // If result was a failure code,
        // and the code was not that no connections were available:
        if (optimizeResult != OptimizeWLAN_Success &&
            optimizeResult != OptimizeWLAN_NoConnections)
        {
            // Stop trying to optimize WLAN if we hit an unexpected failure
            cout << "*** WLANOptimizer failed with error code: " << optimizeResult << endl;
            break;
        }

        if (optimizeResult == OptimizeWLAN_Success) {
            cout << "*** Wireless LAN detected and settings updated to reduce latency." << endl;
        }
        else if (optimizeResult == OptimizeWLAN_NoConnections) {
            cout << "*** No Wireless LAN connections detected to optimize." << endl;
        }

        std::this_thread::sleep_for(kOptimizeInterval);
    }

    cout << "WLANOptimizer has stopped due to an error.  Close this application at any time." << endl;

    for (;;) {
        std::this_thread::sleep_for(1s);
    }

    return 0;
}

#endif
