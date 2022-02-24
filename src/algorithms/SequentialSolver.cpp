//
// Created by David Trinh on 23/02/2022.
//

#include <queue>
#include <iostream>
#include "SequentialSolver.h"

SequentialSolver::SequentialSolver(const WeightedGraph &inputGraph) : inputGraph(inputGraph) {}

int SequentialSolver::getEdgeWeight(std::set<Edge> edges) const {
    int weight = 0;
    for (const Edge edge: edges) {
        weight += edge.getWeight();
    }
    return weight;
}

void SequentialSolver::printSolution() const {
    std::cout << "Max weight: " << bestValue << std::endl;
    std::cout << "Recursion count: " << this->recursionCnt << std::endl;
    std::cout << "Solutions: " << this->solutions.size() << std::endl;

    std::cout << std::endl;

    int solutionNo = 1;
    for (const auto &solution: this->solutions) {
        std::set<int> U;
        std::set<int> W;

        for (int j = 0; j < solution.first.size(); j++) {
            if (solution.first[j] == VertexColor::RED) {
                U.insert(j);
            }
            if (solution.first[j] == VertexColor::BLUE) {
                W.insert(j);
            }
        }

        std::cout << "Solution #" << solutionNo << std::endl;

        std::cout << "U: { ";
        for (const int &v: U) {
            std::cout << v << " ";
        }
        std::cout << "}" << std::endl;

        std::cout << "W: { ";
        for (const int &v: W) {
            std::cout << v << " ";
        }
        std::cout << "}" << std::endl;

        std::cout << "E: { ";
        for (const auto &e: solution.second) {
            std::cout << "(" << e.getVertex1() << ", " << e.getVertex2() << ") ";
        }
        std::cout << "}" << std::endl;

        std::cout << std::endl;
        solutionNo++;
    }
}


void SequentialSolver::solve() {
    if (this->isSolved) {
        printSolution();
        return;
    }

    std::vector<VertexColor> coloring(this->inputGraph.getSize(), VertexColor::UNASSIGNED);
    coloring[0] = VertexColor::BLUE;
    std::set<Edge> addedEdges;
    solveUtil(coloring, inputGraph.getEdges(), inputGraph.getIncidentEdges(0), addedEdges, 0);

    this->isSolved = true;
    printSolution();
}


void SequentialSolver::colorVertices(
        const Edge &edge,
        VertexColor c1,
        VertexColor c2,
        const std::vector<VertexColor> &coloring,
        const std::set<Edge> &remainingEdges,
        const std::set<Edge> &openedEdges,
        const std::set<Edge> &addedEdges,
        const int &currentValue
) {
    int v1 = edge.getVertex1();
    int v2 = edge.getVertex2();

    if ((coloring[v1] == VertexColor::UNASSIGNED || coloring[v1] == c1) &&
        (coloring[v2] == VertexColor::UNASSIGNED || coloring[v2] == c2)) {
        std::vector<VertexColor> newColoring(coloring);
        std::set<Edge> newRemainingEdges(openedEdges);
        std::set<Edge> newAddedEdges(addedEdges);
        newAddedEdges.insert(edge);
        newColoring[v1] = c1;
        newColoring[v2] = c2;

        if (coloring[v1] == VertexColor::UNASSIGNED) {
            for (Edge e: this->inputGraph.getIncidentEdges(v1)) {
                if (coloring[e.getNeighbor(v1)] == VertexColor::UNASSIGNED) {
                    newRemainingEdges.insert(e);
                }
            }
        }

        if (coloring[v2] == VertexColor::UNASSIGNED) {
            for (Edge e: this->inputGraph.getIncidentEdges(v2)) {
                if (coloring[e.getNeighbor(v2)] == VertexColor::UNASSIGNED) {
                    newRemainingEdges.insert(e);
                }
            }
        }

        solveUtil(
                newColoring,
                remainingEdges,
                newRemainingEdges,
                newAddedEdges,
                currentValue + edge.getWeight()
        );
    }
}

void SequentialSolver::solveUtil(
        std::vector<VertexColor> coloring,
        std::set<Edge> remainingEdges,
        std::set<Edge> openedEdges,
        std::set<Edge> addedEdges,
        int currentValue
) {
    this->recursionCnt++;

    // Better value was found.
    if (currentValue > bestValue) {
        bestValue = currentValue;
        this->solutions.clear();
    }

    // If current value is the best, mark added edges as solution.
    if (currentValue == bestValue) {
        this->solutions.insert(std::make_pair(coloring, addedEdges));
    }

    // Better solution can't be found.
    if (currentValue + getEdgeWeight(remainingEdges) < bestValue) { return; }

    // No opened edges are left.
    if (openedEdges.empty()) { return; }

    // Pop edge with the highest weight.
    VertexColor c1;
    VertexColor c2;
    Edge edge = *openedEdges.begin();
    openedEdges.erase(edge);
    remainingEdges.erase(edge);

    // First vertex is colored as red, second is blue
    c1 = VertexColor::RED;
    c2 = VertexColor::BLUE;
    colorVertices(edge, c1, c2, coloring, remainingEdges, openedEdges, addedEdges, currentValue);

    // First vertex is colored as blue, second is red
    c1 = VertexColor::BLUE;
    c2 = VertexColor::RED;
    colorVertices(edge, c1, c2, coloring, remainingEdges, openedEdges, addedEdges, currentValue);

    // Do not add edge to the solution.
    solveUtil(coloring, remainingEdges, openedEdges, addedEdges, currentValue);
}


