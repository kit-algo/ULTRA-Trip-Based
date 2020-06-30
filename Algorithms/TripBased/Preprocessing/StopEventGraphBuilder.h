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

#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class StopEventGraphBuilder {

private:
    class StopLabel {

    public:
        StopLabel() :
            arrivalTime(INFTY),
            timeStamp(0) {
        }

    public:
        inline void checkTimeStamp(const int newTimeStamp) noexcept {
            arrivalTime = (timeStamp != newTimeStamp) ? INFTY : arrivalTime;
            timeStamp = newTimeStamp;
        }

        inline void update(const int newTimeStamp, const int newArrivalTime) noexcept {
            checkTimeStamp(newTimeStamp);
            arrivalTime = std::min(arrivalTime, newArrivalTime);
        }

    public:
        int arrivalTime;
        int timeStamp;
    };

public:
    StopEventGraphBuilder(const Data& data) :
        data(data),
        edges(data.numberOfStopEvents()),
        labels(data.numberOfStops()),
        timeStamp(0) {
    }

public:
    inline void scanTrip(const TripId trip) noexcept {
        const StopId* stops = data.stopArrayOfTrip(trip);
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        for (StopIndex i = StopIndex(1); i < data.numberOfStopsInTrip(trip); i++) {
            const int arrivalTime = data.raptorData.stopEvents[firstEvent + i].arrivalTime;
            scanRoutes(trip, i, stops[i], arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                scanRoutes(trip, i, StopId(data.raptorData.transferGraph.get(ToVertex, edge)), arrivalTime + data.raptorData.transferGraph.get(TravelTime, edge));
            }
        }
    }

    inline void scanRoutes(const TripId trip, const StopIndex i, const StopId stop, const int arrivalTime) noexcept  {
        const RouteId originalRoute = data.routeOfTrip[trip];
        for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(stop)) {
            const TripId other = data.getEarliestTrip(route, arrivalTime);
            if (other == noTripId) continue;
            if ((route.routeId == originalRoute) && (other >= trip) && (route.stopIndex >= i)) continue;
            if (isUTransfer(trip, i, other, route.stopIndex)) continue;
            edges[data.firstStopEventOfTrip[trip] + i].emplace_back(Vertex(data.firstStopEventOfTrip[other] + route.stopIndex));
        }
    }

    inline bool isUTransfer(const TripId fromTrip, const StopIndex fromIndex, const TripId toTrip, const StopIndex toIndex) const noexcept {
        if (fromIndex < 2) return false;
        if (toIndex + 1 >= data.numberOfStopsInTrip(toTrip)) return false;
        if (data.getStop(fromTrip, StopIndex(fromIndex - 1)) != data.getStop(toTrip, StopIndex(toIndex + 1))) return false;
        if (data.getStopEvent(fromTrip, StopIndex(fromIndex - 1)).arrivalTime > data.getStopEvent(toTrip, StopIndex(toIndex + 1)).departureTime) return false;
        return true;
    }

    inline void reduceTransfers(const TripId trip) noexcept {
        timeStamp++;
        const StopId* stops = data.stopArrayOfTrip(trip);
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        for (StopIndex i = StopIndex(data.numberOfStopsInTrip(trip) - 1); i > 0; i--) {
            const int arrivalTime = data.raptorData.stopEvents[firstEvent + i].arrivalTime;
            labels[stops[i]].update(timeStamp, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                labels[data.raptorData.transferGraph.get(ToVertex, edge)].update(timeStamp, arrivalTime + data.raptorData.transferGraph.get(TravelTime, edge));
            }

            const Vertex stopEventVertex = Vertex(data.firstStopEventOfTrip[trip] + i);
            if (edges[stopEventVertex].empty()) continue;
            std::sort(edges[stopEventVertex].begin(), edges[stopEventVertex].end(), [&](const Vertex a, const Vertex b){
                return data.raptorData.stopEvents[a].arrivalTime < data.raptorData.stopEvents[b].arrivalTime;
            });
            std::vector<Vertex> transfers;
            transfers.emplace_back(edges[stopEventVertex][0]);
            for (size_t i = 0; i < edges[stopEventVertex].size(); i++) {
                if (transfers.back() != edges[stopEventVertex][i]) {
                    transfers.emplace_back(edges[stopEventVertex][i]);
                }
            }
            edges[stopEventVertex].clear();

            std::vector<Edge> deleteTransfers;
            for (const Vertex transferTarget : transfers) {
                bool keep = false;
                const StopIndex transferTargetIndex = data.indexOfStopEvent[transferTarget];
                const TripId transferTargetTrip = data.tripOfStopEvent[transferTarget];
                const StopId* transferTargetStops = data.stopArrayOfTrip(transferTargetTrip) + transferTargetIndex;
                for (size_t j = data.numberOfStopsInTrip(transferTargetTrip) - transferTargetIndex - 1; j > 0; j--) {
                    const StopId transferTargetStop = transferTargetStops[j];
                    const int transferTargetTime = data.raptorData.stopEvents[transferTarget + j].arrivalTime;
                    labels[transferTargetStop].checkTimeStamp(timeStamp);
                    if (labels[transferTargetStop].arrivalTime > transferTargetTime) {
                        labels[transferTargetStop].arrivalTime = transferTargetTime;
                        keep = true;
                    }
                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(transferTargetStop)) {
                        const StopId arrivalStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                        const int arrivalTime = transferTargetTime + data.raptorData.transferGraph.get(TravelTime, edge);
                        labels[arrivalStop].checkTimeStamp(timeStamp);
                        if (labels[arrivalStop].arrivalTime > arrivalTime) {
                            labels[arrivalStop].arrivalTime = arrivalTime;
                            keep = true;
                        }
                    }
                }
                if (keep) edges[stopEventVertex].emplace_back(transferTarget);
            }
            std::vector<Vertex> keptTransfers;
            keptTransfers.reserve(edges[stopEventVertex].size());
            for (const Vertex v : edges[stopEventVertex]) {
                keptTransfers.emplace_back(v);
            }
            edges[stopEventVertex].swap(keptTransfers);
        }
    }

public:
    const Data& data;

    std::vector<std::vector<Vertex>> edges;

    std::vector<StopLabel> labels;
    int timeStamp;

};

inline void ComputeStopEventGraph(Data& data) noexcept {
    Progress progress(data.numberOfTrips());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    StopEventGraphBuilder builder(data);
    for (const TripId trip : data.trips()) {
        builder.scanTrip(trip);
        builder.reduceTransfers(trip);
        progress++;
    }
    for (const Vertex from : stopEventGraph.vertices()) {
        for (const Vertex to : builder.edges[from]) {
            stopEventGraph.addEdge(from, to);
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraph(Data& data, const int numberOfThreads, const int pinMultiplier = 1) noexcept {
    Progress progress(data.numberOfTrips());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
    #pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        StopEventGraphBuilder builder(data);
        const size_t numberOfTrips = data.numberOfTrips();

        #pragma omp for schedule(dynamic,1)
        for (size_t i = 0; i < numberOfTrips; i++) {
            const TripId trip = TripId(i);
            builder.scanTrip(trip);
            builder.reduceTransfers(trip);
            progress++;
        }

        #pragma omp critical
        {
            for (const Vertex from : stopEventGraph.vertices()) {
                for (const Vertex to : builder.edges[from]) {
                    stopEventGraph.addEdge(from, to);
                }
            }
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

}
