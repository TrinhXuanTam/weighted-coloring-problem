//
// Created by David Trinh on 23/02/2022.
//

#ifndef NI_PDP_EDGE_H
#define NI_PDP_EDGE_H


class Edge {
public:
    Edge(int v1, int v2, int weight);

    int getVertex1() const;

    int getVertex2() const;

    int getWeight() const;

    int getNeighbor(int vertex) const;

    bool operator<(const Edge &rhs) const;

private:
    int v1;

    int v2;

    int weight;
};


#endif //NI_PDP_EDGE_H
