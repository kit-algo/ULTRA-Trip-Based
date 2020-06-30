[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Integrating ULTRA and Trip-Based Routing
This C++ framework contains a variant of the Trip-Based Routing algorithm that was combined with ULTRA in order to enable the efficient computation of shortest paths in multi-modal transportation networks.
Both, the preprocessing algorithms and the query algorithm, have a special focus on integrating unlimited path in the transfer graph and the Trip-Based query algorithm.
The framework was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).

## Usage

This framework contains code for generating Trip-Based transfer shortcuts using two alternative approaches. First, shortcuts computed by ULTRA can be used as input for the standard Trip-Based preprocessing algorithm. Secondly, an integrated variant of the ULTRA and Trip-based preprocessing steps can be used to directly compute the Trip-Based transfer shortcuts. Additionally, the framwork contains code for The ULTRA-Trip-Based query algorithm, which can compute Pareto-optimal journeys, given the transfer shortcuts found by either of the preprocessing approaches. Finally, the framework contains code of the standard transitive variant of the Trip-Based query algorithm and the ULTRA-RAPTOR algorithm for comparisson. All components of the framework can be compiled into a single console application, using the ``Makefile`` that is located in the ``Runnables`` folder. Within this application the following commands are available:

* ``RaptorToTripBased`` computes Trip-Based shortcuts using the sequential preprocessing
* ``RaptorToTripBasedUsingULTRA`` computes Trip-Based shortcuts using the integrated preprocessing
* ``UltraPreprocessing`` computes ULTRA shortcuts needed for the sequential preprocessing
* ``CoreCH`` computes a core-CH need for all pother reprocessing steps
* ``BuildCH`` computes a normal CH need for the ULTRA query algorithms
* ``GenerateUltraQueries`` generates random triples of source location, target location, and departure time
* ``GenerateGeoRankQueries`` generates random queries, grouped by their query distance (geo-rank)
* ``RunUltraQueries`` evaluates a query algorithm on queries generated with the commands above

All of the above commands use custom data formats for loading the public transit network and the transfer graph. As an example we provide the public transit network of Switzerland together with a transfer graph extracted from OpenStreetMap in the appropriate binary format at [https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/](https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/).

## Literature

The algorithms in this Framework are mainly based on two previously published algorithms: ULTRA and Trip-Based Routing.

* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*  
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf  
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019  
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)

* *Trip-Based Public Transit Routing*  
  Sascha Witt  
  In: Proceedings of the 23rd Annual European Symposium on Algorithms (ESA'15), Lecture Notes in Computer Science volume 9294, pages 1025--1036, 2015  
  [html](https://link.springer.com/chapter/10.1007/978-3-662-48350-3_85) [arXiv](https://arxiv.org/pdf/1504.07149)

