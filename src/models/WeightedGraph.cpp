//
// Created by David Trinh on 23/02/2022.
//

#include <iostream>
#include "WeightedGraph.h"

WeightedGraph::WeightedGraph(int size, std::vector<Edge> &edges) : size(size) {
    this->adjacencyList = std::vector<std::set<Edge>>(size, std::set<Edge>());

    for (auto &edge: edges) {
        this->adjacencyList[edge.getVertex1()].insert(edge);
        this->adjacencyList[edge.getVertex2()].insert(edge);
        this->edges.insert(edge);
    }

}

int WeightedGraph::getSize() const {
    return size;
}

std::set<int> WeightedGraph::getNeighbors(int vertex) const {
    std::set<int> neighbors;

    for (const Edge edge: this->adjacencyList[vertex]) {
        if (vertex == edge.getVertex2()) {
            neighbors.insert(edge.getVertex1());
        } else if (vertex == edge.getVertex1()) {
            neighbors.insert(edge.getVertex2());
        }
    }

    return neighbors;
}

const std::set<Edge> &WeightedGraph::getIncidentEdges(int vertex) const {
    return this->adjacencyList[vertex];
}

const std::set<Edge> &WeightedGraph::getEdges() const {
    return edges;
}

