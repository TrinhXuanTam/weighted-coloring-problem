//
// Created by David Trinh on 23/02/2022.
//

#include <stdexcept>
#include "Edge.h"

int Edge::getWeight() const {
    return weight;
}

bool Edge::operator<(const Edge &rhs) const {
    if (weight < rhs.weight)
        return false;
    if (rhs.weight < weight)
        return true;
    if (v1 < rhs.v1)
        return true;
    if (rhs.v1 < v1)
        return false;
    if (v2 < rhs.v2)
        return true;
    if (rhs.v2 < v2)
        return false;
    return false;
}

Edge::Edge(int v1, int v2, int weight) : weight(weight) {
    if (v1 < v2) {
        this->v1 = v1;
        this->v2 = v2;
    } else {
        this->v1 = v2;
        this->v2 = v1;
    }
}

int Edge::getVertex1() const {
    return this->v1;
}

int Edge::getVertex2() const {
    return this->v2;
}

int Edge::getNeighbor(int vertex) const {
    if (vertex != this->v1 && vertex != this->v2) { throw std::invalid_argument("Invalid vertex given."); }
    return vertex == this->v1 ? v2 : v1;
}
