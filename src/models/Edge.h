//
// Created by David Trinh on 23/02/2022.
//

#ifndef NI_PDP_EDGE_H
#define NI_PDP_EDGE_H


class Edge {
public:
    Edge(int src, int dst, int weight);

    int getDst() const;

    int getSrc() const;

    int getWeight() const;

    bool operator<(const Edge &rhs) const;

private:
    int src;
    int dst;
    int weight;
};


#endif //NI_PDP_EDGE_H
