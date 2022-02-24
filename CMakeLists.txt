cmake_minimum_required(VERSION 3.21)
project(NI_PDP)

set(CMAKE_CXX_STANDARD 14)

add_executable(NI_PDP
        main.cpp src/models/WeightedGraph.cpp src/models/WeightedGraph.h src/utils/factories/WeightedGraphFactory.cpp src/utils/factories/WeightedGraphFactory.h src/models/Edge.cpp src/models/Edge.h src/algorithms/SequentialSolver.cpp src/algorithms/SequentialSolver.h src/enums/VertexColor.h)
