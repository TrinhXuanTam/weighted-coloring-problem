//
// Created by David Trinh on 23/02/2022.
//

#ifndef NI_PDP_WEIGHTEDGRAPH_H
#define NI_PDP_WEIGHTEDGRAPH_H

#include <vector>
#include <set>
#include "Edge.h"

class WeightedGraph {
public:
    explicit WeightedGraph(int size, std::vector<Edge> &edges);

    std::set<int> getNeighbors(int vertex) const;

    const std::set<Edge> &getIncidentEdges(int vertex) const;

    int getSize() const;

    const std::set<Edge> &getEdges() const;

private:
    int size;

    std::set<Edge> edges;

    std::vector<std::set<Edge>> adjacencyList;
};


#endif //NI_PDP_WEIGHTEDGRAPH_H
