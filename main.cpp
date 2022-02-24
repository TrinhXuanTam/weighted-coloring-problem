//
// Created by David Trinh on 16/02/2022.
//

#include <iostream>
#include <bitset>
#include "src/utils/factories/WeightedGraphFactory.h"
#include "src/algorithms/SequentialSolver.h"

int main(int argc, char *argv[]) {
    std::string path = argv[1];
    WeightedGraph inputGraph = WeightedGraphFactory::fromFile(path);
    std::set<Edge> edges;

    auto solver = SequentialSolver(inputGraph);
    solver.solve();
    return 0;
}