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

    const std::set<int> &getNeighbors(int vertex) const;

    int getTotalEdges() const;

    const std::vector<Edge> &getEdges() const;

    void setEdges(const std::vector<Edge> &edges);

    int getSize() const;

private:
    int totalEdges;

    int size;

    std::vector<Edge> edges;

    std::vector<std::set<int>> adjacencyList;
};


#endif //NI_PDP_WEIGHTEDGRAPH_H
