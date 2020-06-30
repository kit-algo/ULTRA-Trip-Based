/**********************************************************************************

 Copyright (c) 2020 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "RangeSearchUsingStations.h"

namespace TripBased {

template<bool ALLOW_REBOARDING_OF_TRIPS, bool DEBUG = false, bool REQUIRE_DIRECT_TRANSFER = false>
class StopEventGraphBuilderUsingULTRA {

public:
    inline static constexpr bool AllowReboardingOfTrips = ALLOW_REBOARDING_OF_TRIPS;
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool RequireDirectTransfer = REQUIRE_DIRECT_TRANSFER;
    using Type = StopEventGraphBuilderUsingULTRA<AllowReboardingOfTrips, Debug, RequireDirectTransfer>;

public:
    StopEventGraphBuilderUsingULTRA(const Data& data) :
        data(data) {
        stopEventGraph.addVertices(data.numberOfStopEvents());
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int maxInitialWalking = 15 * 60, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        std::vector<Shortcut> shortcuts;

        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            RangeSearchUsingStations<AllowReboardingOfTrips, Debug, RequireDirectTransfer> rangeSearch(data, maxInitialWalking);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                rangeSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                const std::vector<Shortcut>& localShortcuts = rangeSearch.getShortcuts();
                for (const Shortcut& shortcut : localShortcuts) {
                    shortcuts.emplace_back(shortcut);
                }
            }
        }

        std::sort(shortcuts.begin(), shortcuts.end(), [](const Shortcut& a, const Shortcut& b){
            return (a.origin < b.origin) || ((a.origin == b.origin) && (a.destination < b.destination));
        });
        stopEventGraph.addEdge(Vertex(shortcuts[0].origin), Vertex(shortcuts[0].destination));
        for (size_t i = 1; i < shortcuts.size(); i++) {
            if ((shortcuts[i].origin == shortcuts[i - 1].origin) && (shortcuts[i].destination == shortcuts[i - 1].destination)) continue;
            stopEventGraph.addEdge(Vertex(shortcuts[i].origin), Vertex(shortcuts[i].destination));
        }
        stopEventGraph.sortEdges(ToVertex);

        progress.finished();
    }

    inline const SimpleDynamicGraph& getStopEventGraph() const noexcept {
        return stopEventGraph;
    }

    inline SimpleDynamicGraph& getStopEventGraph() noexcept {
        return stopEventGraph;
    }

private:
    const Data& data;
    SimpleDynamicGraph stopEventGraph;

};

}
