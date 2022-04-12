//
// Created by David Trinh on 16/02/2022.
//

#include <iostream>
#include <set>
#include <vector>
#include <chrono>
#include <algorithm>
#include <optional>

enum VertexColor {
    UNASSIGNED,
    BLUE,
    RED
};

struct Edge {
    int v1;
    int v2;
    int weight;

    Edge(int v1, int v2, int weight) : v1(v1), v2(v2), weight(weight) {}

    static bool cmp(const Edge& lhs, const Edge& rhs) {
        if(lhs.v1 == rhs.v1) {
            return lhs.weight >= rhs.weight;
        }
        return lhs.v1 >= rhs.v1;
    }
};

struct Configuration {
    std::vector<VertexColor> coloring;
    std::vector<int> edges;
   
    Configuration(std::vector<VertexColor> coloring, std::vector<int> edges) : coloring(coloring), edges(edges) {}

    Configuration(std::vector<VertexColor> coloring) : coloring(coloring) {
        this->edges = std::vector<int>();
    }    
};

struct State {
    std::vector<Edge> edges;
    Configuration configuration;
    int remainingWeight;
    int currentWeight;
    int index;

    State(
        int vertexCnt,
        std::vector<Edge> edges
    ) : edges(edges), 
        configuration(Configuration(std::vector<VertexColor>(vertexCnt, VertexColor::UNASSIGNED))),
        remainingWeight(0),
        currentWeight(0),
        index(0) {
        for (const auto& edge : edges) {
            this->remainingWeight += edge.weight;
        }
    }

    State(
        std::vector<Edge> edges,
        Configuration configuration, 
        int remainingWeight, 
        int currentWeight, 
        int index
    ) : edges(edges),
        configuration(configuration), 
        remainingWeight(remainingWeight), 
        currentWeight(currentWeight), 
        index(index) {}

    std::optional<State> next(VertexColor c1, VertexColor c2) const {
        if ((size_t)(this->index) == this->edges.size()) {
            return std::nullopt;
        }

        State newState = *this;
        Edge current = edges[this->index];
        newState.remainingWeight -= current.weight;
        newState.index++;

        if (c1 == VertexColor::UNASSIGNED  && c2 == VertexColor::UNASSIGNED) {
            return newState;
        }

        if ((this->configuration.coloring[current.v1] == VertexColor::UNASSIGNED || this->configuration.coloring[current.v1] == c1)
        && (this->configuration.coloring[current.v2] == VertexColor::UNASSIGNED || this->configuration.coloring[current.v2] == c2)) {
            newState.currentWeight += current.weight;
            newState.configuration.coloring[current.v1] = c1;
            newState.configuration.coloring[current.v2] = c2;
            newState.configuration.edges.emplace_back(this->index);
            return newState;
        }

        return std::nullopt;
    }
};

void printSolution(
    const std::vector<Edge>& edges,
    const int &bestWeight, 
    const int &recursionCnt, 
    const std::vector<Configuration> &results, 
    const std::chrono::duration<double> time
) {
    std::cout << "Total time: " << time.count() << "s" << std::endl;
    std::cout << "Max weight: " << bestWeight << std::endl;
    std::cout << "Recursion count: " << recursionCnt << std::endl;

    std::cout << std::endl;
    
    std::set<std::set<int>> uniqueSolutions;
    int solutionNo = 1;
    for (const auto &result: results) {
        std::set<int> solutionEdges(result.edges.begin(), result.edges.end());
        if (uniqueSolutions.find(solutionEdges) == uniqueSolutions.end()) {
            uniqueSolutions.insert(solutionEdges);
        } else {
            continue;
        }

        std::vector<int> U;
        std::vector<int> W;

        for (size_t j = 0; j < result.coloring.size(); j++) {
            if (result.coloring[j] == VertexColor::RED) {
                U.emplace_back(j);
            }
            if (result.coloring[j] == VertexColor::BLUE) {
                W.emplace_back(j);
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
        for (const auto &e: result.edges) {
            std::cout << "(" << edges[e].v1 << ", " << edges[e].v2 << ") ";
        }
        std::cout << "}" << std::endl;

        std::cout << std::endl;
        solutionNo++;
    }
}

void solve(int &recursionCnt, int &bestWeight, std::vector<Configuration> &results, const State& state) {
    recursionCnt++;
    
    if (state.currentWeight >= bestWeight) {
        #pragma omp critical
        {
            // Better value was found.
            if (state.currentWeight > bestWeight) {
                bestWeight = state.currentWeight;
                results.clear();
            }
            
            // If current value is the best, mark added edges as solution.
            if (state.currentWeight == bestWeight) {
                results.push_back(state.configuration);
            }
        }
    }
    
    // Better solution can't be found.
    if (state.currentWeight + state.remainingWeight < bestWeight) { return; }

    // No opened edges are left.
    if ((size_t)(state.index) == state.edges.size()) { return; }

    // Add edge to the solution if valid coloring exists.
    std::optional<State> blueRedState = state.next(VertexColor::BLUE, VertexColor::RED);
    std::optional<State> redBlueState = state.next(VertexColor::RED, VertexColor::BLUE);
    if (blueRedState) {
        solve(recursionCnt, bestWeight, results, *blueRedState);
    } 
    if (redBlueState) {
        solve(recursionCnt, bestWeight, results, *redBlueState);
    }

    // Do not add edge to the solution.
    std::optional<State> edgeNotAddedState = state.next(VertexColor::UNASSIGNED, VertexColor::UNASSIGNED);
    if (edgeNotAddedState) {
        solve(recursionCnt, bestWeight, results, *edgeNotAddedState);
    }

}

int main(int argc, char *argv[]) {
    int vertexCnt = 0;
    int bestWeight = 0;
    int recursionCnt = 0;
    std::vector<Edge> edges;
    std::vector<Configuration> results;
    std::cin >> vertexCnt;
    for (int i = 0; i < vertexCnt; ++i) {
        for (int j = 0; j < vertexCnt; ++j) {
            int weight;
            std::cin >> weight;
            if (weight != 0 && j >= i) {
                edges.emplace_back(i, j, weight);
            }
        }
    }
    std::sort(edges.begin(), edges.end(), Edge::cmp);

    auto startTime = std::chrono::high_resolution_clock::now();
    solve(recursionCnt, bestWeight, results, State(vertexCnt, edges));
    auto endTime = std::chrono::high_resolution_clock::now();

    printSolution(edges, bestWeight, recursionCnt, results, endTime - startTime);

    return 0;
}
