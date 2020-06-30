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

#include "../../Shell/Shell.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Algorithms/StronglyConnectedComponents.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/WitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/RAPTOR/TransferShortcuts/Preprocessing/Builder.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"


using namespace Shell;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RaptorToTripBased //////////////////////////////////////////////////////////////////////
class RaptorToTripBased : public ParameterizedCommand {

public:
    RaptorToTripBased(BasicShell& shell) :
        ParameterizedCommand(shell, "raptorToTripBased", "Converts binary RAPTOR data to the Trip-Based transit format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Num threads", "0");
        addParameter("Thread offset", "1");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numThreads = getParameter<int>("Num threads");
        const int pinMultiplier = getParameter<int>("Thread offset");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        if (numThreads == 0) {
            TripBased::ComputeStopEventGraph(data);
        } else {
            TripBased::ComputeStopEventGraph(data, numThreads, pinMultiplier);
        }

        data.printInfo();
        data.serialize(outputFile);
        std::cout << "Finished Trip-Based preprocessing" << std::endl;
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BuildCH //////////////////////////////////////////////////////////////////////
class BuildCH : public ParameterizedCommand {

public:
    BuildCH(BasicShell& shell) :
        ParameterizedCommand(shell, "buildCH", "Computes a CH for the input graph.") {
        addParameter("StaticGraph file");
        addParameter("CH file");
        addParameter("Number of test queries", "0");
        addParameter("Use full debugger", "false");
        addParameter("Witness search type", "normal", {"normal", "bidirectional"});
        addParameter("Level weight", "1024", {"32", "64", "128", "256", "512", "1024", "2048"});
        addParameter("Order file", "-");
        addParameter("Order type", "txt", {"txt", "bin32", "bin64"});
    }

    virtual void execute() noexcept {
        const bool useFullDebugger = getParameter<bool>("Use full debugger");
        if (useFullDebugger) {
            return chooseWitnessSearch<CH::FullDebugger>();
        } else {
            return chooseWitnessSearch<CH::TimeDebugger>();
        }
    }

private:
    template<typename DEBUGGER>
    inline void chooseWitnessSearch() noexcept {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            return chooseLevelWeight<DEBUGGER, CH::WitnessSearch<CHCoreGraph, DEBUGGER, 500>>();
        } else {
            return chooseLevelWeight<DEBUGGER, CH::BidirectionalWitnessSearch<CHCoreGraph, DEBUGGER, 200>>();
        }
    }

    template<typename DEBUGGER, typename WITNESS_SEARCH>
    inline void chooseLevelWeight() noexcept {
        const int levelWeight = getParameter<int>("Level weight");
        switch (levelWeight) {
            case 2048: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 2048>();
            }
            case 1024: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 1024>();
            }
            case 512: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 512>();
            }
            case 256: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 256>();
            }
            case 128: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 128>();
            }
            case 64: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 64>();
            }
            case 32: {
                return chooseKeyFunction<DEBUGGER, WITNESS_SEARCH, 32>();
            }
        }
    }

    template<typename DEBUGGER, typename WITNESS_SEARCH, int LEVEL_WEIGHT>
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
            using KEY_FUNCTION = CH::GreedyKey<WITNESS_SEARCH, 1024, LEVEL_WEIGHT, 0>;
            CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), graph[TravelTime]);
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
            CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), graph[TravelTime], KEY_FUNCTION(order));
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// CoreCH //////////////////////////////////////////////////////////////////////
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
        addParameter("Use full debugger", "false");
        addParameter("Level weight", "1024", {"32", "64", "128", "256", "512", "1024", "2048"});
    }

    virtual void execute() noexcept {
        const bool useFullDebugger = getParameter<bool>("Use full debugger");
        if (useFullDebugger) {
            return chooseWitnessSearch<CH::FullDebugger>();
        } else {
            return chooseWitnessSearch<CH::TimeDebugger>();
        }
    }

private:
    template<typename DEBUGGER>
    inline void chooseWitnessSearch() noexcept {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            return chooseLevelWeight<DEBUGGER, CH::WitnessSearch<CHCoreGraph, DEBUGGER, 500>>();
        } else {
            return chooseLevelWeight<DEBUGGER, CH::BidirectionalWitnessSearch<CHCoreGraph, DEBUGGER, 200>>();
        }
    }

    template<typename DEBUGGER, typename WITNESS_SEARCH>
    inline void chooseLevelWeight() noexcept {
        const int levelWeight = getParameter<int>("Level weight");
        switch (levelWeight) {
            case 2048: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 2048>();
            }
            case 1024: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 1024>();
            }
            case 512: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 512>();
            }
            case 256: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 256>();
            }
            case 128: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 128>();
            }
            case 64: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 64>();
            }
            case 32: {
                return buildCH<DEBUGGER, WITNESS_SEARCH, 32>();
            }
        }
    }

    template<typename DEBUGGER, typename WITNESS_SEARCH, int LEVEL_WEIGHT>
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
        using KEY_FUNCTION = CH::PartialKey<WITNESS_SEARCH, CH::GreedyKey<WITNESS_SEARCH, 1024, LEVEL_WEIGHT, 0>>;
        using STOP_CRITERION = CH::CoreCriterion;
        CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), KEY_FUNCTION(isNormalVertex, graph.numVertices()), STOP_CRITERION(numberOfStops, maxCoreDegree));
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// UltraPreprocessing //////////////////////////////////////////////////////////////////////
class UltraPreprocessing : public ParameterizedCommand {

public:
    UltraPreprocessing(BasicShell& shell) :
        ParameterizedCommand(shell, "ultraPreprocessing", "Computes one hop transfer shortcuts using the ULTRA approach.\n   <Number of threads> = 'max': use one thread per physical core.") {
        addParameter("RAPTOR file");
        addParameter("Walking limit");
        addParameter("Output file");
        addParameter("Number of threads", "1");
        addParameter("Pin multiplier", "1");
        addParameter("Allow reboarding of trips", "false");
        addParameter("Require direct transfer", "false");
    }

    virtual void execute() noexcept {
        const std::string raptorFileName = getParameter("RAPTOR file");
        const int walkingLimit = getParameter<int>("Walking limit");
        const std::string outputFileName = getParameter("Output file");
        const int numberOfthreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const bool allowReboardingOfTrips = getParameter<bool>("Allow reboarding of trips");
        const bool requireDirectTransfer = getParameter<bool>("Require direct transfer");
        RAPTOR::Data data = RAPTOR::Data::FromBinary(raptorFileName);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();
        if (allowReboardingOfTrips) {
            RAPTOR::TransferShortcuts::Preprocessing::Builder<true> shortcutGraphBuilder(data);
            std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
            shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, pinMultiplier), walkingLimit);
            Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
        } else {
            if (requireDirectTransfer) {
                RAPTOR::TransferShortcuts::Preprocessing::Builder<false, false, true, true> shortcutGraphBuilder(data);
                std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
                shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, pinMultiplier), walkingLimit);
                Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
            } else {
                RAPTOR::TransferShortcuts::Preprocessing::Builder<false, false, true, false> shortcutGraphBuilder(data);
                std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
                shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, pinMultiplier), walkingLimit);
                Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
            }
        }
        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFileName);
    }

    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

};
