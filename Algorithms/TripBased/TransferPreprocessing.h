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

#include <iostream>
#include <vector>
#include <string>

#include "../Dijkstra/Dijkstra.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"

namespace RAPTOR {

class TransferPreprocessing {

private:
    struct Source {
        Id routeId;
        Id tripIndex;
        Id stopIndex;
        Id stopEventIndex;
        Id stop;
        int arrivalTime;
    };

    struct Target {
        Id routeId;
        Id tripIndex;
        Id stopIndex;
        Id stopEventIndex;
    };

public:
    TransferPreprocessing(const Data& data) :
        data(data),
        dijkstra(data.transferGraph, data.transferGraph.getTravelTime()),
        transfersOfStopEventIndex(data.stopEvents.size()),
        direktTransfersTotal(0),
        direktTransfersNoTurns(0),
        direktTransfersFiltered(0),
        walkingTransfersTotal(0),
        walkingTransfersNoTurns(0),
        walkingTransfersFiltered(0) {
    }

    inline std::vector<std::vector<Transfer>> getTransfersOfStopEventIndex() noexcept {
        transfersOfStopEventIndex.clear();
        std::vector<int>(data.transferGraph.numVertices(), intMax).swap(arrivalTimeOfVertex);
        std::vector<int>(data.transferGraph.numVertices(), intMax).swap(departureTimeOfVertex);
        reachableVertices.clear();
        const Id numberOfRoutes = data.numberOfRoutes();
        for (source.routeId = 0; source.routeId < numberOfRoutes; source.routeId++) {
            const Id numberOfTrips = data.numberOfTripsInRoute(source.routeId);
            const Id numberOfStops = data.numberOfStopsInRoute(source.routeId);
            const Id* sourceStops = data.stopArrayOfRoute(source.routeId);
            for (source.tripIndex = 0; source.tripIndex < numberOfTrips; source.tripIndex++) {
                for (source.stopIndex = numberOfStops - 1; source.stopIndex > 0; source.stopIndex--) {
                    source.stopEventIndex = data.firstStopEventOfRoute[source.routeId] + (source.tripIndex * numberOfStops) + source.stopIndex;
                    source.arrivalTime = data.stopEvents[source.stopEventIndex].arrivalTime;
                    source.stop = sourceStops[source.stopIndex];

                    if (arrivalTimeOfVertex[source.stop] == intMax) reachableVertices.emplace_back(source.stop);
                    arrivalTimeOfVertex[source.stop] = std::min(arrivalTimeOfVertex[source.stop], source.arrivalTime);
                    departureTimeOfVertex[source.stop] = std::min(departureTimeOfVertex[source.stop], source.arrivalTime + data.stopData[source.stop].minTransferTime);
                    forAllFootpathsFrom(source.stop, [&](const ){})
                    // Run Dijkstra

                    collectTransfers(source.stop, source.arrivalTime + data.minTransferTime(source.stop));
                    // Run Dijkstra

                }
                for (const Vertex vertex : reachableVertices) {
                    arrivalTimeOfVertex[source.stop] = intMax;
                    departureTimeOfVertex[source.stop] = intMax;
                }
                reachableVertices.clear();
            }
        }
        return transfersOfStopEventIndex;
    }

private:
    template<typename CALL_BACK>
    inline void forAllFootpathsFrom(const Vertex stop, const CALL_BACK& callBack) noexcept {
        dijkstra.run(stop, noVertex, [&](const Vertex u){
            if (u == stop) return;
            callBack(u, dijkstra.getDistance(u));
        });
    }

    inline void collectTransfers(const Id stop, const int departureTime) noexcept {
        for (const RouteSegment& targetSegment : data.routesContainingStop(stop)) {
            const Id numberOfTrips = data.numberOfTripsInRoute(targetSegment.routeId);
            const Id numberOfStops = data.numberOfStopsInRoute(targetSegment.routeId);
            if (targetSegment.stopIndex + 1 == numberOfStops) continue;
            target.routeId = targetSegment.routeId;
            target.stopIndex = targetSegment.stopIndex;
            for (target.tripIndex = 0; target.tripIndex < numberOfTrips; target.tripIndex++) {
                target.stopEventIndex = data.firstStopEventOfRoute[target.routeId] + (target.tripIndex * numberOfStops) + target.stopIndex;
                if (data.stopEvents[target.stopEventIndex].departureTime >= departureTime) break;
            }
            if (target.stopEventIndex >= data.firstStopEventOfRoute[target.routeId + 1]) continue;
            if ((target.routeId == source.routeId) && (target.stopIndex > source.stopIndex) && (target.tripIndex >= source.tripIndex)) continue;
            if (transferIsTurn()) continue;

            if ((sourceStops[sourceStopIndex - 1] != stopArrayOfRoute(target.routeId)[target.stopIndex + 1]) || (stopEvents[targetStopEventIndex + 1].departureTime < stopEvents[sourceStopEventIndex - 1].arrivalTime + stopData[sourceStops[sourceStopIndex - 1]].minTransferTime)) {
                bool keep = false;

                // Run Dijkstra???
                transfersOfStopEventIndex[sourceStopEventIndex].emplace_back();
            }
            // Test for removal
        }
    }

    inline bool transferIsTurn() const noexcept {

    }

private:
    const Data& data;
    Dijkstra<RAPTOR::TransferGraph> dijkstra;

    std::vector<std::vector<Transfer>> transfersOfStopEventIndex;

    Source source;
    Target target;

    std::vector<int> arrivalTimeOfVertex;
    std::vector<int> departureTimeOfVertex;
    std::vector<Id> reachableVertices;

    int direktTransfersTotal;
    int direktTransfersNoTurns;
    int direktTransfersFiltered;
    int walkingTransfersTotal;
    int walkingTransfersNoTurns;
    int walkingTransfersFiltered;


};

}
