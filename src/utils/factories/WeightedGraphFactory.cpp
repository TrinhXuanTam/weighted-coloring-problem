//
// Created by David Trinh on 23/02/2022.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include "WeightedGraphFactory.h"

WeightedGraph WeightedGraphFactory::fromFile(std::string &path) {
    std::ifstream infile(path);
    std::string line;
    std::getline(infile, line);
    int graphSize = std::stoi(line);
    std::set<Edge> edges;

    for (int i = 0; i < graphSize; ++i) {
        std::getline(infile, line);
        std::istringstream iss(line);
        for (int j = 0; j < graphSize; ++j) {
            int weight;
            iss >> weight;
            if (weight != 0) {
                edges.insert(Edge(i, j, weight));

            }
        }
    }

    std::vector<Edge> edgeVector = std::vector<Edge>(edges.begin(), edges.end());
    return WeightedGraph(graphSize, edgeVector);
}
