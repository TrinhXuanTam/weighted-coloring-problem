//
// Created by David Trinh on 23/02/2022.
//

#include "Edge.h"

int Edge::getDst() const {
    return dst;
}

int Edge::getWeight() const {
    return weight;
}

bool Edge::operator<(const Edge &rhs) const {
    if (weight < rhs.weight)
        return false;
    if (rhs.weight < weight)
        return true;
    if (src < rhs.src)
        return true;
    if (rhs.src < src)
        return false;
    if (dst < rhs.dst)
        return true;
    if (rhs.dst < dst)
        return false;
    return false;
}

Edge::Edge(int src, int dst, int weight) : src(src), dst(dst), weight(weight) {}

int Edge::getSrc() const {
    return src;
}
