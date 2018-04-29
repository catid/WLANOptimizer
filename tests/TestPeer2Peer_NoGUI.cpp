#include "Connection.h"

#include <iostream> // cin


static logger::Channel Logger("TestPeer2Peer", wopt::kMinLogLevel);

class NoGUIStatsReceiver : public wopt::IStatisticsReceiver
{
public:
    void OnStats(const wopt::Statistics& stats) override
    {
        Logger.Info("One-Way Delay Stats (msec): Min=", stats.Min, " Max=",
            stats.Max, " Avg=", stats.Average, " StdDev=", stats.StandardDeviation);

        Logger.Info("One-Way Delay Percentiles: 10%=", stats.Percentiles[1],
            " 20%=", stats.Percentiles[2],
            " 50%=", stats.Percentiles[5],
            " 80%=", stats.Percentiles[8],
            " 90%=", stats.Percentiles[9]);
    }
};


//------------------------------------------------------------------------------
// Entrypoint

int main()
{
    NoGUIStatsReceiver receiver;
    wopt::TestSocket sock(&receiver);

    if (!sock.Initialize(wopt::kServerPort))
    {
        Logger.Error("Socket initialization failed.  Only one copy of the tester can run on each computer");
        return -1;
    }

    Logger.Info("Listening on port ", wopt::kServerPort, ".  Enter a string and press ENTER to stop the app.");

    for (;;)
    {
        std::string str;
        std::cin >> str;
    }

    Logger.Info("Shutdown started..");

    sock.Shutdown();

    return 0;
}
