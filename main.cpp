//
// Created by David Trinh on 16/02/2022.
//

#include <iostream>
#include <vector>
#include <set>
#include <chrono>

enum VertexColor {
    UNASSIGNED,
    BLUE,
    RED
};

struct Edge {
    int v1;
    int v2;
    int weight;

    Edge(int v1, int v2, int weight) : weight(weight) {
        if (v1 < v2) {
            this->v1 = v1;
            this->v2 = v2;
        } else {
            this->v1 = v2;
            this->v2 = v1;
        }
    }

    bool operator<(const Edge &rhs) const {
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
};

struct Configuration {
    std::vector<VertexColor> coloring;
    std::set<Edge> addedEdges;

    Configuration(
            const std::vector<VertexColor> &coloring,
            const std::set<Edge> &addedEdges
    ) : coloring(coloring), addedEdges(addedEdges) {}

    bool operator<(const Configuration &rhs) const {
        if (coloring < rhs.coloring)
            return true;
        if (rhs.coloring < coloring)
            return false;
        return addedEdges < rhs.addedEdges;
    }
};

void printSolution(const int &bestValue, const int &recursionCnt, const std::set<Configuration> &results, const std::chrono::duration<double> time) {
    std::cout << "Total time: " << time.count() << "s" << std::endl;
    std::cout << "Max weight: " << bestValue << std::endl;
    std::cout << "Recursion count: " << recursionCnt << std::endl;
    std::cout << "Solutions: " << results.size() << std::endl;

    std::cout << std::endl;

    int solutionNo = 1;
    for (const auto &result: results) {
        std::set<int> U;
        std::set<int> W;

        for (size_t j = 0; j < result.coloring.size(); j++) {
            if (result.coloring[j] == VertexColor::RED) {
                U.insert(j);
            }
            if (result.coloring[j] == VertexColor::BLUE) {
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
        for (const auto &e: result.addedEdges) {
            std::cout << "(" << e.v1 << ", " << e.v2 << ") ";
        }
        std::cout << "}" << std::endl;

        std::cout << std::endl;
        solutionNo++;
    }
}

bool isColorAssignmentValid(const Edge &edge, VertexColor c1, VertexColor c2, const Configuration &configuration) {
    int v1 = edge.v1;
    int v2 = edge.v2;

    return (configuration.coloring[v1] == VertexColor::UNASSIGNED || configuration.coloring[v1] == c1) &&
           (configuration.coloring[v2] == VertexColor::UNASSIGNED || configuration.coloring[v2] == c2);
}

void solve(
        int &recursionCnt,
        int &bestValue,
        std::set<Configuration> &results,
        Configuration current,
        std::vector<std::set<Edge>> adjacencyMatrix,
        std::set<Edge> openedEdges,
        int remainingValue,
        int currentValue
) {
    recursionCnt++;

    // Better solution can't be found.
    if (currentValue + remainingValue < bestValue) { return; }

    // No opened edges are left.
    if (openedEdges.empty()) { return; }

    // Better value was found.
    if (currentValue > bestValue) {
        bestValue = currentValue;
        results.clear();
    }
            
    // If current value is the best, mark added edges as solution.
    if (currentValue == bestValue) {
        results.insert(current);
    }

    // Remove the heaviest edge from opened edges.
    Edge heaviestEdge = *openedEdges.begin();
    openedEdges.erase(openedEdges.begin());
    adjacencyMatrix[heaviestEdge.v1].erase(heaviestEdge);
    adjacencyMatrix[heaviestEdge.v2].erase(heaviestEdge);
    remainingValue -= heaviestEdge.weight;

    // Color vertices.
    VertexColor c1 = VertexColor::UNASSIGNED;
    VertexColor c2 = VertexColor::UNASSIGNED;
    if (isColorAssignmentValid(heaviestEdge, VertexColor::RED, VertexColor::BLUE, current)) {
        c1 = VertexColor::RED;
        c2 = VertexColor::BLUE;
    } else if (isColorAssignmentValid(heaviestEdge, VertexColor::BLUE, VertexColor::RED, current)) {
        c1 = VertexColor::BLUE;
        c2 = VertexColor::RED;
    }
    if (c1 != VertexColor::UNASSIGNED && c2 != VertexColor::UNASSIGNED) {
        Configuration newConfiguration = Configuration(current.coloring, current.addedEdges);
        newConfiguration.coloring[heaviestEdge.v1] = c1;
        newConfiguration.coloring[heaviestEdge.v2] = c2;
        newConfiguration.addedEdges.insert(heaviestEdge);
        std::vector<std::set<Edge>> newAdjacencyMatrix = adjacencyMatrix;
        std::set<Edge> newOpenedEdges = openedEdges;

        for (auto e: adjacencyMatrix[heaviestEdge.v1]) {
            newAdjacencyMatrix[e.v1].erase(e);
            newAdjacencyMatrix[e.v2].erase(e);
            newOpenedEdges.insert(e);
        }

        for (auto e: adjacencyMatrix[heaviestEdge.v2]) {
            newAdjacencyMatrix[e.v1].erase(e);
            newAdjacencyMatrix[e.v2].erase(e);
            newOpenedEdges.insert(e);
        }

        solve(recursionCnt, bestValue, results, newConfiguration, newAdjacencyMatrix, newOpenedEdges, remainingValue, currentValue + heaviestEdge.weight);
    }

    // Do not add edge to the solution.
    solve(recursionCnt, bestValue, results, current, adjacencyMatrix, openedEdges, remainingValue, currentValue);
}

int main(int argc, char *argv[]) {
    int vertexCnt = 0;
    int recursionCnt = 0;
    int bestValue = 0;
    int totalWeight = 0;
    std::set<Configuration> results;
    std::set<Edge> openedEdges;
    Edge heaviestEdge = Edge(-1, -1, -1);

    std::cin >> vertexCnt;
    std::vector<std::set<Edge>> adjacencyMatrix = std::vector<std::set<Edge>>(vertexCnt, std::set<Edge>());

    for (int i = 0; i < vertexCnt; ++i) {
        for (int j = 0; j < vertexCnt; ++j) {
            int weight;
            std::cin >> weight;
            if (weight != 0 && j >= i) {
                Edge edge(i, j, weight);
                adjacencyMatrix[i].insert(edge);
                adjacencyMatrix[j].insert(edge);
                if (weight > heaviestEdge.weight) {
                    heaviestEdge = edge;
                }
                totalWeight += weight;
            }
        }
    }

    std::vector<VertexColor> coloring = std::vector<VertexColor>(vertexCnt, VertexColor::UNASSIGNED);
    std::set<Edge> addedEdges;

    openedEdges = adjacencyMatrix[heaviestEdge.v1];
    coloring[heaviestEdge.v1] = VertexColor::RED;
    adjacencyMatrix[heaviestEdge.v1].erase(heaviestEdge);
    adjacencyMatrix[heaviestEdge.v2].erase(heaviestEdge);

    auto startTime = std::chrono::high_resolution_clock::now();

    solve(recursionCnt, bestValue, results, Configuration(coloring, addedEdges), adjacencyMatrix, openedEdges, totalWeight, 0);

    auto endTime = std::chrono::high_resolution_clock::now();

    printSolution(bestValue, recursionCnt, results, endTime - startTime);
}