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
#include <string>

#include "../../Algorithms/RAPTOR/ULTRA/Builder.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilder.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Helpers/MultiThreading.h"

#include "../../Shell/Shell.h"

using namespace Shell;

class ComputeStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeStopToStopShortcuts", "Computes stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Prune with existing shortcuts?", "true");
        addParameter("Require direct transfer?", "false");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const size_t witnessLimit = getParameter<size_t>("Witness limit");
        const size_t numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");
        const bool pruneWithExistingShortcuts = getParameter<bool>("Prune with existing shortcuts?");
        const bool requireDirectTransfer = getParameter<bool>("Require direct transfer?");

        RAPTOR::Data data = RAPTOR::Data::FromBinary(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();
        choosePrune(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer, pruneWithExistingShortcuts);
        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }

private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    inline void choosePrune(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit, const bool requireDirectTransfer, const bool pruneWithExistingShortcuts) const noexcept {
        if (pruneWithExistingShortcuts) {
            chooseRequireDirectTransfer<true>(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer);
        } else {
            chooseRequireDirectTransfer<false>(data, numberOfThreads, pinMultiplier, witnessLimit, requireDirectTransfer);
        }
    }

    template<bool PRUNE_WITH_EXISTING_SHORTCUTS>
    inline void chooseRequireDirectTransfer(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit, const bool requireDirectTransfer) const noexcept {
        if (requireDirectTransfer) {
            run<PRUNE_WITH_EXISTING_SHORTCUTS, true>(data, numberOfThreads, pinMultiplier, witnessLimit);
        } else {
            run<PRUNE_WITH_EXISTING_SHORTCUTS, false>(data, numberOfThreads, pinMultiplier, witnessLimit);
        }
    }

    template<bool PRUNE_WITH_EXISTING_SHORTCUTS, bool REQUIRE_DIRECT_TRANSFER>
    inline void run(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t witnessLimit) const noexcept {
        RAPTOR::ULTRA::Builder<false, PRUNE_WITH_EXISTING_SHORTCUTS, REQUIRE_DIRECT_TRANSFER> shortcutGraphBuilder(data);
        std::cout << "Computing stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
    }
};

class RAPTORToTripBased : public ParameterizedCommand {

public:
    RAPTORToTripBased(BasicShell& shell) :
        ParameterizedCommand(shell, "raptorToTripBased", "Converts stop-to-stop transfers to event-to-event transfers and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        if (numberOfThreads == 0) {
            TripBased::ComputeStopEventGraph(data);
        } else {
            TripBased::ComputeStopEventGraph(data, numberOfThreads, pinMultiplier);
        }

        data.printInfo();
        data.serialize(outputFile);
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

};

class ComputeEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeEventToEventShortcuts", "Computes event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int witnessLimit = getParameter<int>("Witness limit");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::ULTRABuilder shortcutGraphBuilder(data);
        std::cout << "Computing event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

};
