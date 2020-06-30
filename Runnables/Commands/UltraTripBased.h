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
#include <algorithm>
#include <random>
#include <vector>
#include <string>
#include <optional>
#include <cmath>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Geometry/Point.h"

#include "../../Algorithms/StronglyConnectedComponents.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/Debugger.h"
#include "../../Algorithms/CH/Preprocessing/WitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilderUsingULTRA.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../Algorithms/Dijkstra/Dijkstra.h"

using namespace Shell;

namespace ULTRA {

struct Query {
    Query(const int s = -1, const int t = -1, const int dt = never, const int geoRank = 0) : source(s), target(t), departureTime(dt), earliestArrivalTime(never), numberOfTrips(-1), queryTime(0), geoRank(geoRank) {}
    friend std::ostream& operator<<(std::ostream& out, const Query& q) {
        return out << q.geoRank << "," << q.source.value() << "," << q.target.value() << "," << q.departureTime << "," << q.earliestArrivalTime << "," << q.numberOfTrips << "," << q.queryTime;
    }
    Vertex source;
    Vertex target;
    int departureTime;
    int earliestArrivalTime;
    int numberOfTrips;
    double queryTime;
    int geoRank;
};

struct TargetDistance {
    TargetDistance(const int t = -1, const double distance = 0) : target(t), distance(distance) {}
    inline bool operator<(const TargetDistance& other) const noexcept {
        return distance < other.distance;
    }
    Vertex target;
    double distance;
};

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GenerateUltraQueries //////////////////////////////////////////////////////////////////////
class GenerateUltraQueries : public ParameterizedCommand {

public:
    GenerateUltraQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateUltraQueries", "Generates random ULTRA queries.") {
        addParameter("Raptor file");
        addParameter("Query file");
        addParameter("Number of queries");
        addParameter("Seed", "42");
        addParameter("Minimum departure time", "00:00:00");
        addParameter("Maximum departure time", "24:00:00");
        addParameter("Stop-based", "false", {"true", "false"});
    }

    virtual void execute() noexcept {
        const std::string raptorFile = getParameter("Raptor file");
        const std::string queryFile = getParameter("Query file");
        const size_t numberOfQueries = getParameter<int>("Number of queries");
        const int seed = getParameter<int>("Seed");
        const int minimumTime = String::parseSeconds(getParameter("Minimum departure time"));
        const int maximumTime = String::parseSeconds(getParameter("Maximum departure time"));
        const bool stopBased = getParameter<bool>("Stop-based");

        RAPTOR::Data data(raptorFile);

        std::mt19937 randomGenerator(seed);
        std::uniform_int_distribution<> vertexDistribution(0, stopBased ? (data.numberOfStops() - 1) : (data.transferGraph.numVertices() - 1));
        std::uniform_int_distribution<> timeDistribution(minimumTime, maximumTime);
        std::vector<ULTRA::Query> queries;
        while (queries.size() < numberOfQueries) {
            queries.emplace_back(vertexDistribution(randomGenerator), vertexDistribution(randomGenerator), timeDistribution(randomGenerator));
        }

        IO::serialize(queryFile, queries);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GenerateGeoRankQueries //////////////////////////////////////////////////////////////////////
class GenerateGeoRankQueries : public ParameterizedCommand {

public:
    GenerateGeoRankQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateGeoRankQueries", "Generates random ULTRA Geo-Rank queries.") {
        addParameter("Raptor file");
        addParameter("Query file");
        addParameter("Number of queries");
        addParameter("Seed", "42");
        addParameter("Minimum departure time", "00:00:00");
        addParameter("Maximum departure time", "24:00:00");
        addParameter("Stop-based", "false", {"true", "false"});
        addParameter("Max Rank", "100");
    }

    virtual void execute() noexcept {
        const std::string raptorFile = getParameter("Raptor file");
        const std::string queryFile = getParameter("Query file");
        const size_t numberOfQueries = getParameter<int>("Number of queries");
        const int seed = getParameter<int>("Seed");
        const int minimumTime = String::parseSeconds(getParameter("Minimum departure time"));
        const int maximumTime = String::parseSeconds(getParameter("Maximum departure time"));
        const bool stopBased = getParameter<bool>("Stop-based");
        const int maxRank = getParameter<int>("Max Rank");

        RAPTOR::Data data(raptorFile);

        std::mt19937 randomGenerator(seed);
        std::uniform_int_distribution<> vertexDistribution(0, stopBased ? (data.numberOfStops() - 1) : (data.transferGraph.numVertices() - 1));
        std::uniform_int_distribution<> timeDistribution(minimumTime, maximumTime);
        std::vector<ULTRA::Query> queries;
        std::vector<ULTRA::TargetDistance> targets;
        for (size_t i = 0; i < numberOfQueries; i++) {
            const Vertex source = Vertex(vertexDistribution(randomGenerator));
            const int departureTime = timeDistribution(randomGenerator);
            targets.clear();
            if (stopBased) {
                for (const StopId stop : data.stops()) {
                    targets.emplace_back(stop, Geometry::geoDistanceInCM(data.transferGraph.get(Coordinates, source), data.transferGraph.get(Coordinates, stop)));
                }
            } else {
                for (const Vertex vertex : data.transferGraph.vertices()) {
                    targets.emplace_back(vertex, Geometry::geoDistanceInCM(data.transferGraph.get(Coordinates, source), data.transferGraph.get(Coordinates, vertex)));
                }
            }
            std::sort(targets.begin(), targets.end());
            int r = 0;
            for (size_t rank = 1; rank < targets.size(); rank = rank * 2) {
                if (r > maxRank) break;
                queries.emplace_back(source, targets[rank].target, departureTime, r);
                r++;
            }
        }
        std::uniform_int_distribution<> queryDistribution(0, queries.size() - 1);
        for (size_t i = 0; i < queries.size(); i++) {
            const size_t j = queryDistribution(randomGenerator);
            if (i == j) continue;
            std::swap(queries[i], queries[j]);
        }

        IO::serialize(queryFile, queries);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RunUltraQueries //////////////////////////////////////////////////////////////////////
class RunUltraQueries : public ParameterizedCommand {

public:
    RunUltraQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUltraQueries", "Evaluates random ULTRA queries.") {
        addParameter("Network file");
        addParameter("CH file");
        addParameter("Query file");
        addParameter("Result file");
        addParameter("Query type", {"RAPTOR", "Trip-Based", "Trip-Based*"});
        addParameter("Debug", "true", {"true", "false"});
    }

    virtual void execute() noexcept {
        const std::string networkFile = getParameter("Network file");
        const std::string chFile = getParameter("CH file");
        const std::string queryFile = getParameter("Query file");
        const std::string resultFileName = getParameter("Result file");
        const std::string queryType = getParameter("Query type");
        const bool debug = getParameter<bool>("Debug");

        std::vector<ULTRA::Query> queries;
        IO::deserialize(queryFile, queries);

        if (queryType == "RAPTOR") {
            CH::CH ch(chFile);
            RAPTOR::Data data(networkFile);
            if (debug) {
                RAPTOR::ULTRARAPTOR<true, RAPTOR::BucketCHInitialTransfers, RAPTOR::SimpleDebugger> algorithm(data, ch);
                runQueries(algorithm, queries);
            } else {
                RAPTOR::ULTRARAPTOR<true, RAPTOR::BucketCHInitialTransfers, RAPTOR::NoDebugger> algorithm(data, ch);
                runQueries(algorithm, queries);
            }
        } else if (queryType == "Trip-Based") {
            CH::CH ch(chFile);
            TripBased::Data data(networkFile);
            if (debug) {
                TripBased::Query<TripBased::ReachedIndexSmall, true> algorithm(data, ch);
                runQueries(algorithm, queries);
            } else {
                TripBased::Query<TripBased::ReachedIndexSmall, false> algorithm(data, ch);
                runQueries(algorithm, queries);
            }
        } else if (queryType == "Trip-Based*") {
            TripBased::Data data(networkFile);
            data.printInfo();
            if (debug) {
                TripBased::TransitiveQuery<TripBased::ReachedIndexSmall, true> algorithm(data);
                runQueries(algorithm, queries);
            } else {
                TripBased::TransitiveQuery<TripBased::ReachedIndexSmall, false> algorithm(data);
                runQueries(algorithm, queries);
            }
        }

        std::ofstream resultFile(resultFileName);
        AssertMsg(resultFile.is_open(), "Could not open file " << resultFileName << "!");
        resultFile << "Rank,Source,Target,DepTime,ArrTime,Trips,QueryTime\n";
        for (ULTRA::Query& query : queries) {
            resultFile << query << "\n";
        }
    }

private:
    template<typename ALGORITHM>
    inline void runQueries(ALGORITHM& algorithm, std::vector<ULTRA::Query>& queries) {
        std::cout << "Evaluating " << String::prettyInt(queries.size()) << " queries..." << std::endl;
        Timer timer;
        Timer queryTimer;
        for (ULTRA::Query& query : queries) {
            queryTimer.restart();
            algorithm.run(query.source, query.departureTime, query.target);
            query.queryTime = queryTimer.elapsedMilliseconds();
            query.earliestArrivalTime = algorithm.getEarliestArrivalTime();
            query.numberOfTrips = algorithm.getEarliestArrivalNumerOfTrips();
        }
        const double time = timer.elapsedMilliseconds();
        std::cout << "Done in " << String::msToString(time) << " (" << String::prettyDouble(time / queries.size(), 1) << "ms per query)" << std::endl;
        algorithm.debug(queries.size());
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RaptorToTripBasedUsingULTRA //////////////////////////////////////////////////////////////////////
class RaptorToTripBasedUsingULTRA : public ParameterizedCommand {

public:
    RaptorToTripBasedUsingULTRA(BasicShell& shell) :
        ParameterizedCommand(shell, "raptorToTripBasedUsingULTRA", "Converts binary RAPTOR data to the Trip-Based transit format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Num threads", "0");
        addParameter("Thread offset", "1");
        addParameter("Walking limit");
        addParameter("Require direct transfer", "false");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numberOfthreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Thread offset");
        const int walkingLimit = getParameter<int>("Walking limit");
        const bool requireDirectTransfer = getParameter<bool>("Require direct transfer");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        if (requireDirectTransfer) {
            TripBased::StopEventGraphBuilderUsingULTRA<false, false, true> shortcutGraphBuilder(data);
            std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
            shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, pinMultiplier), walkingLimit);
            Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);
        } else {
            TripBased::StopEventGraphBuilderUsingULTRA<false, false, false> shortcutGraphBuilder(data);
            std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
            shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, pinMultiplier), walkingLimit);
            Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);
        }

        data.printInfo();
        data.serialize(outputFile);
        std::cout << "Finished ULTRA-Trip-Based preprocessing" << std::endl;
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Num threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Num threads");
        }
    }

};
