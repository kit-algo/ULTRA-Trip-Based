/**********************************************************************************

 Copyright (c) 2020-2021 Tobias Zündorf, Jonas Sauer

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
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/KeyFunction.h"
#include "../../Algorithms/CH/Preprocessing/Profiler.h"
#include "../../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../../Algorithms/CH/Preprocessing/WitnessSearch.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "../../Algorithms/CH/CH.h"

#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Timer.h"

using namespace Shell;

class BuildCH : public ParameterizedCommand {

public:
    BuildCH(BasicShell& shell) :
        ParameterizedCommand(shell, "buildCH", "Computes a CH for the input graph.") {
        addParameter("StaticGraph file");
        addParameter("CH file");
        addParameter("Number of test queries", "0");
        addParameter("Use full profiler?", "false");
        addParameter("Witness search type", "normal", {"normal", "bidirectional"});
        addParameter("Level weight", "1024");
        addParameter("Order file", "-");
        addParameter("Order type", "txt", {"txt", "bin32", "bin64"});
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use full profiler?")) {
            return chooseWitnessSearch<CH::FullProfiler>();
        } else {
            return chooseWitnessSearch<CH::TimeProfiler>();
        }
    }

private:
    template<typename PROFILER>
    inline void chooseWitnessSearch() noexcept {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            return chooseKeyFunction<PROFILER, CH::WitnessSearch<CHCoreGraph, PROFILER, 500>>();
        } else {
            return chooseKeyFunction<PROFILER, CH::BidirectionalWitnessSearch<CHCoreGraph, PROFILER, 200>>();
        }
    }

    template<typename PROFILER, typename WITNESS_SEARCH>
    inline void chooseKeyFunction() noexcept {
        const std::string staticGraphFile = getParameter("StaticGraph file");
        const std::string chFile = getParameter("CH file");
        const size_t numberOfTestQueries = getParameter<size_t>("Number of test queries");
        const std::string orderFile = getParameter("Order file");
        const std::string orderType = getParameter("Order type");

        TravelTimeGraph graph(staticGraphFile);
        Graph::printInfo(graph);
        graph.printAnalysis();

        CH::CH ch;
        using STOP_CRITERION = CH::NoStopCriterion;
        if (orderFile == "-") {
            using KEY_FUNCTION = CH::GreedyKey<WITNESS_SEARCH>;
            CH::Builder<PROFILER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), graph[TravelTime], KEY_FUNCTION(1024, getParameter<int>("Level weight"), 0));
            chBuilder.run();
            chBuilder.copyCoreToCH();
            std::cout << "Obtaining CH" << std::endl;
            ch = std::move(chBuilder);
        } else {
            using KEY_FUNCTION = CH::OrderKey<WITNESS_SEARCH>;
            Order order;
            if (orderType == "txt") {
                order = Order(Construct::FromTextFile, orderFile);
            } else if (orderType == "bin32") {
                std::vector<int32_t> data;
                IO::deserialize(orderFile, data);
                order = Order(data);
            } else if (orderType == "bin64") {
                std::vector<int64_t> data;
                IO::deserialize(orderFile, data);
                order = Order(data);
            }
            CH::Builder<PROFILER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), graph[TravelTime], KEY_FUNCTION(order));
            chBuilder.run();
            chBuilder.copyCoreToCH();
            std::cout << "Obtaining CH" << std::endl;
            ch = std::move(chBuilder);
        }

        ch.writeBinary(chFile);
        std::cout << std::endl;
        if (numberOfTestQueries > 0) {
            std::vector<Vertex> sources;
            std::vector<Vertex> targets;
            u_int64_t result = 0;
            for (size_t i = 0; i < numberOfTestQueries; i++) {
                sources.emplace_back(Vertex(rand() % ch.numVertices()));
                targets.emplace_back(Vertex(rand() % ch.numVertices()));
            }
            CH::Query<> query(ch);
            Timer timer;
            for (size_t i = 0; i < numberOfTestQueries; i++) {
                query.run(sources[i], targets[i]);
                result += query.getDistance();
            }
            double time = timer.elapsedMilliseconds();
            std::cout << "Executed " << numberOfTestQueries << " random queries in " << String::msToString(time) << " (checksum = " << result << ")" << std::endl;
        }
    }

};

class CoreCH : public ParameterizedCommand {

public:
    CoreCH(BasicShell& shell) :
        ParameterizedCommand(shell, "coreCH", "Computes a core-CH for a transfer network, where all stop vertices are kept uncontracted."),
        numberOfStops(0) {
        addParameter("Network input file");
        addParameter("CH output file");
        addParameter("Max core degree", "16");
        addParameter("Network type", "intermediate", {"intermediate", "raptor"});
        addParameter("Witness search type", "normal", {"normal", "bidirectional"});
        addParameter("Network output file", "-");
        addParameter("Use full profiler?", "false");
        addParameter("Level weight", "1024");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use full profiler?")) {
            return chooseWitnessSearch<CH::FullProfiler>();
        } else {
            return chooseWitnessSearch<CH::TimeProfiler>();
        }
    }

private:
    template<typename PROFILER>
    inline void chooseWitnessSearch() noexcept {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            return buildCH<PROFILER, CH::WitnessSearch<CHCoreGraph, PROFILER, 500>>();
        } else {
            return buildCH<PROFILER, CH::BidirectionalWitnessSearch<CHCoreGraph, PROFILER, 200>>();
        }
    }

    template<typename PROFILER, typename WITNESS_SEARCH>
    inline void buildCH() noexcept {
        const std::string networkInputFile = getParameter("Network input file");
        const std::string chOutputFile = getParameter("CH output file");
        const double maxCoreDegree = getParameter<double>("Max core degree");
        const std::string networkType = getParameter("Network type");
        const std::string networkOutputFile = getParameter("Network output file");

        CHCoreGraph graph;
        Intermediate::TransferGraph resultGraph;
        if (networkType == "raptor") {
            raptorData = RAPTOR::Data::FromBinary(networkInputFile);
            raptorData->printInfo();
            resultGraph.addVertices(raptorData->transferGraph.numVertices());
            resultGraph[Coordinates] = raptorData->transferGraph[Coordinates];
            numberOfStops = raptorData->numberOfStops();
            Graph::move(std::move(raptorData->transferGraph), graph, Weight << TravelTime);
        } else {
            inter = Intermediate::Data::FromBinary(networkInputFile);
            inter->printInfo();
            resultGraph.addVertices(inter->transferGraph.numVertices());
            resultGraph[Coordinates] = inter->transferGraph[Coordinates];
            numberOfStops = inter->numberOfStops();
            Graph::move(std::move(inter->transferGraph), graph, Weight << TravelTime);
        }
        Graph::printInfo(graph);
        std::vector<bool> isNormalVertex(numberOfStops, false);
        isNormalVertex.resize(graph.numVertices(), true);

        std::cout << "min Core Size: " << String::prettyInt(numberOfStops) << std::endl;
        std::cout << "max Core Degree: " << String::prettyInt(maxCoreDegree) << std::endl;
        using GREEDY_KEY_FUNCTION = CH::GreedyKey<WITNESS_SEARCH>;
        using KEY_FUNCTION = CH::PartialKey<WITNESS_SEARCH, GREEDY_KEY_FUNCTION>;
        using STOP_CRITERION = CH::CoreCriterion;
        KEY_FUNCTION keyFunction(isNormalVertex, graph.numVertices(), GREEDY_KEY_FUNCTION(1024, getParameter<int>("Level weight"), 0));
        CH::Builder<PROFILER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), keyFunction, STOP_CRITERION(numberOfStops, maxCoreDegree));
        chBuilder.run();
        chBuilder.copyCoreToCH();
        std::cout << "Obtaining CH" << std::endl;
        CH::CH ch(std::move(chBuilder));

        ch.writeBinary(chOutputFile);
        for (const Vertex vertex : resultGraph.vertices()) {
            if (ch.isCoreVertex(vertex)) {
                for (const Edge edge : ch.forward.edgesFrom(vertex)) {
                    resultGraph.addEdge(vertex, ch.forward.get(ToVertex, edge)).set(TravelTime, ch.forward.get(Weight, edge));
                }
            }
        }
        if (!networkOutputFile.empty()) {
            if (networkType == "raptor") {
                Graph::move(std::move(resultGraph), raptorData->transferGraph);
                raptorData->serialize(networkOutputFile);
            } else {
                inter->transferGraph = std::move(resultGraph);
                inter->serialize(networkOutputFile);
            }
        } else {
            resultGraph.writeBinary(chOutputFile + ".core");
        }
        std::cout << std::endl;
    }

private:
    std::optional<RAPTOR::Data> raptorData;
    std::optional<Intermediate::Data> inter;
    size_t numberOfStops;

};
