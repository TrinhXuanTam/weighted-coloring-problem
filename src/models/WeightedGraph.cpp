//
// Created by David Trinh on 23/02/2022.
//

#include <iostream>
#include "WeightedGraph.h"

WeightedGraph::WeightedGraph(int size, std::vector<Edge> &edges) : size(size), edges(edges), totalEdges(edges.size()) {
    this->adjacencyList = std::vector<std::set<int>>(size, std::set<int>());

    for (auto &edge: edges) {
        adjacencyList[edge.getSrc()].insert(edge.getSrc());
        adjacencyList[edge.getDst()].insert(edge.getDst());
    }
}

const std::set<int> &WeightedGraph::getNeighbors(int vertex) const {
    return this->adjacencyList[vertex];
}

int WeightedGraph::getSize() const {
    return size;
}

const std::vector<Edge> &WeightedGraph::getEdges() const {
    return edges;
}

void WeightedGraph::setEdges(const std::vector<Edge> &edges) {
    WeightedGraph::edges = edges;
}

int WeightedGraph::getTotalEdges() const {
    return totalEdges;
}

