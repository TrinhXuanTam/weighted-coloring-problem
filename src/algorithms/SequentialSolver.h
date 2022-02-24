//
// Created by David Trinh on 23/02/2022.
//

#ifndef NI_PDP_SEQUENTIALSOLVER_H
#define NI_PDP_SEQUENTIALSOLVER_H

#include "../models/WeightedGraph.h"
#include "../enums/VertexColor.h"

class SequentialSolver {
public:
    explicit SequentialSolver(const WeightedGraph &inputGraph);

    void solve();

private:
    void printSolution() const;

    void solveUtil(
            std::vector<VertexColor> coloring,
            std::set<Edge> openedEdges,
            std::set<Edge> addedEdges,
            int remainingValue,
            int currentValue
    );

    void colorVertices(
            const Edge &edge,
            VertexColor c1,
            VertexColor c2,
            const std::vector<VertexColor> &coloring,
            const std::set<Edge> &openedEdges,
            const std::set<Edge> &addedEdges,
            const int &remainingValue,
            const int &currentValue
    );

    int getEdgeWeight(std::set<Edge> edges) const;

    WeightedGraph inputGraph;

    bool isSolved = false;

    int recursionCnt = 0;

    int bestValue = 0;

    std::set<std::pair<std::vector<VertexColor>, std::set<Edge>>> solutions;
};


#endif //NI_PDP_SEQUENTIALSOLVER_H
