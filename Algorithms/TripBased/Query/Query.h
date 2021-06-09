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

#include "ReachedIndexSmall.h"

#include "../../CH/Query/BucketQuery.h"

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

struct Journey {
    Journey(const int arrivalTime, const u_int32_t numberOfUsedVehicles) :
        arrivalTime(arrivalTime),
        numberOfUsedVehicles(numberOfUsedVehicles) {
    }
    inline friend std::ostream& operator<<(std::ostream& out, const Journey& j) noexcept {
        return out << "arrivalTime: " << j.arrivalTime << ", numberOfUsedVehicles: " << j.numberOfUsedVehicles;
    }
    int arrivalTime;
    u_int32_t numberOfUsedVehicles;
};

template<typename REACHED_INDEX, bool DEBUG = false>
class Query {

public:
    using ReachedIndex = REACHED_INDEX;
    static constexpr bool Debug = DEBUG;
    using Type = Query<ReachedIndex, Debug>;

private:
    struct TripLabel {
        TripLabel(const u_int32_t begin, const u_int32_t end) :
            begin(begin),
            end(end) {
        }
        u_int32_t begin;
        u_int32_t end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId, const StopEventId firstEvent = noStopEvent) :
            stopEvent(stopEvent),
            trip(trip),
            firstEvent(firstEvent) {
        }
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct RouteLabel {
        RouteLabel() :
            numberOfTrips(0) {
        }
        inline StopIndex end() const noexcept {
            return StopIndex(departureTimes.size() / numberOfTrips);
        }
        u_int32_t numberOfTrips;
        std::vector<int> departureTimes;
    };

public:
    Query(const Data& data, const CH::CH& chData) :
        data(data),
        bucketQuery(chData.forward, chData.backward, data.numberOfStops(), Weight),
        reachedIndex(data),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.numberOfRoutes()) {
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        if (Debug) totalTimer.restart();
        clear();
        computeInitialAndFinalTransfers(source, departureTime, target);
        evaluateInitialTransfers(departureTime);
        scanTrips();
        if (Debug) totalTime += totalTimer.elapsedMicroseconds();
    }

    inline int getEarliestArrivalTime() const noexcept {
        return minArrivalTimeByMaxNumberOfUsedVehicles.back();
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept {
        const int eat = minArrivalTimeByMaxNumberOfUsedVehicles.back();
        for (size_t i = 0; i < minArrivalTimeByMaxNumberOfUsedVehicles.size(); i++) {
            if (minArrivalTimeByMaxNumberOfUsedVehicles[i] == eat) return i;
        }
        return -1;
    }

    inline std::vector<Journey> getJourneys() const noexcept {
        std::vector<Journey> result;
        for (size_t i = 0; i < minArrivalTimeByMaxNumberOfUsedVehicles.size(); i++) {
            if (minArrivalTimeByMaxNumberOfUsedVehicles[i] >= INFTY) continue;
            if ((result.size() >= 1) && (result.back().arrivalTime == minArrivalTimeByMaxNumberOfUsedVehicles[i])) continue;
            result.emplace_back(minArrivalTimeByMaxNumberOfUsedVehicles[i], i);
        }
        return result;
    }

    inline void debug(const double f = 1.0) noexcept {
        std::cout << "Number of enqueued trips: " << String::prettyDouble(enqueueCount / f, 0) << std::endl;
        std::cout << "Number of scanned trips: " << String::prettyDouble(scannedTripsCount / f, 0) << std::endl;
        std::cout << "Number of scanned stops: " << String::prettyDouble(scannedStopsCount / f, 0) << std::endl;
        std::cout << "Number of scanned shortcuts: " << String::prettyDouble(scannedShortcutCount / f, 0) << std::endl;
        std::cout << "Number of rounds: " << String::prettyDouble(roundCount / f, 2) << std::endl;
        std::cout << "Number of found journeys: " << String::prettyDouble(addJourneyCount / f, 0) << std::endl;
        std::cout << "Number of initial transfers: " << String::prettyDouble(initialTransferCount / f, 0) << std::endl;
        std::cout << "Bucket-CH query time: " << String::musToString(chTime / f) << std::endl;
        std::cout << "Initial transfer evaluation time: " << String::musToString(initialTime / f) << std::endl;
        std::cout << "Trip scanning time: " << String::musToString(scanTime / f) << std::endl;
        std::cout << "total time: " << String::musToString(totalTime / f) << std::endl;
        addJourneyCount = 0;
        enqueueCount = 0;
        scannedTripsCount = 0;
        scannedStopsCount = 0;
        scannedShortcutCount = 0;
        roundCount = 0;
        initialTransferCount = 0;
        chTime = 0.0;
        initialTime = 0.0;
        scanTime = 0.0;
        totalTime = 0.0;
    }

private:
    inline void clear() noexcept {
        currentQueue.clear();
        nextQueue.clear();
        reachedIndex.clear();
        numberOfUsedVehicles = 0;
        minArrivalTime = INFTY;
        std::vector<int>(1, INFTY).swap(minArrivalTimeByMaxNumberOfUsedVehicles);
    }

    inline void computeInitialAndFinalTransfers(const Vertex source, const int departureTime, const Vertex target) noexcept {
        if (Debug) chTimer.restart();
        bucketQuery.run(source, target);
        if (bucketQuery.getDistance() != INFTY) {
            addJourney(departureTime + bucketQuery.getDistance());
        }
        if (Debug) chTime += chTimer.elapsedMicroseconds();
    }

    inline void evaluateInitialTransfers(const int departureTime) noexcept {
        if (Debug) initialTimer.restart();
        std::vector<bool> reachedRoutes(data.raptorData.numberOfRoutes(), false);
        for (const Vertex stop : bucketQuery.getForwardPOIs()) {
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(StopId(stop))) {
                reachedRoutes[route.routeId] = true;
            }
        }
        for (const RouteId route : data.raptorData.routes()) {
            if (!reachedRoutes[route]) continue;
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const int timeFromSource = bucketQuery.getForwardDistance(stops[stopIndex]);
                if (timeFromSource == INFTY) continue;
                const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), departureTime + timeFromSource, [&](const TripId trip, const int time) {
                        return label.departureTimes[labelIndex + trip] < time;
                    });
                    if (tripIndex >= label.numberOfTrips) continue;
                } else {
                    const int stopDepartureTime = departureTime + timeFromSource;
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime) continue;
                    tripIndex--;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        tripIndex--;
                    }
                }
                enqueue(firstTrip + tripIndex, stopIndex);
                if (tripIndex == 0) break;
            }
        }
        if (Debug) initialTime += initialTimer.elapsedMicroseconds();
    }

    inline void scanTrips() noexcept {
        if (Debug) scanTimer.restart();
        while (!nextQueue.empty()) {
            if constexpr (Debug) roundCount++;
            currentQueue.swap(nextQueue);
            numberOfUsedVehicles++;
            for (const TripLabel& label : currentQueue) { // Evaluate final transfers in order to check if the target is reachable
                if constexpr (Debug) scannedTripsCount++;
                for (StopEventId i(label.begin); i < label.end; i++) {
                    if constexpr (Debug) scannedStopsCount++;
                    if (data.arrivalEvents[i].arrivalTime >= minArrivalTime) break;
                    const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[i].stop);
                    if (timeToTarget != INFTY) addJourney(data.arrivalEvents[i].arrivalTime + timeToTarget);
                }
            }
            for (TripLabel& label : currentQueue) { // Find the range of transfers for each trip
                for (StopEventId i(label.begin); i < label.end; i++) {
                    if (data.arrivalEvents[i].arrivalTime >= minArrivalTime) label.end = i;
                }
                label.begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                label.end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            for (const TripLabel& label : currentQueue) { // Relax the transfers for each trip
                for (Edge edge(label.begin); edge < label.end; edge++) {
                    if constexpr (Debug) scannedShortcutCount++;
                    enqueue(edge);
                }
            }
            currentQueue.clear();
        }
        if (Debug) scanTime += scanTimer.elapsedMicroseconds();
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept {
        if constexpr (Debug) enqueueCount++;
        if (reachedIndex.alreadyReached(trip, index + 1)) return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        nextQueue.emplace_back(firstEvent + index + 1, firstEvent + reachedIndex(trip));
        reachedIndex.update(trip, index);
    }

    inline void enqueue(const Edge edge) noexcept {
        if constexpr (Debug) enqueueCount++;
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent)) return;
        nextQueue.emplace_back(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip)));
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

    inline void addJourney(const int newArrivalTime) noexcept {
        if constexpr (Debug) addJourneyCount++;
        if (numberOfUsedVehicles >= minArrivalTimeByMaxNumberOfUsedVehicles.size()) {
            minArrivalTimeByMaxNumberOfUsedVehicles.resize(numberOfUsedVehicles + 1, minArrivalTimeByMaxNumberOfUsedVehicles.back());
        }
        AssertMsg(numberOfUsedVehicles + 1 == minArrivalTimeByMaxNumberOfUsedVehicles.size(), "Wrong number of used vehicles!");
        minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles] = std::min(minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles], newArrivalTime);
        minArrivalTime = minArrivalTimeByMaxNumberOfUsedVehicles[numberOfUsedVehicles];
    }

private:
    const Data& data;

    CH::BucketQuery<CHGraph, true, false> bucketQuery;
    std::vector<TripLabel> currentQueue;
    std::vector<TripLabel> nextQueue;
    ReachedIndex reachedIndex;

    int minArrivalTime;
    u_int32_t numberOfUsedVehicles;
    std::vector<int> minArrivalTimeByMaxNumberOfUsedVehicles;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;

    size_t addJourneyCount{0};
    size_t enqueueCount{0};
    size_t scannedTripsCount{0};
    size_t scannedStopsCount{0};
    size_t scannedShortcutCount{0};
    size_t roundCount{0};
    size_t initialTransferCount{0};
    Timer chTimer;
    Timer initialTimer;
    Timer scanTimer;
    Timer totalTimer;
    double chTime;
    double initialTime;
    double scanTime;
    double totalTime;

};

}
