//
// Created by David Trinh on 16/02/2022.
//

#include <omp.h>
#include <iostream>
#include <vector>
#include <set>
#include <chrono>
#include <optional>
#include <deque>

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

    bool operator==(const Edge &rhs) const {
        return this->v1 == rhs.v1 && this->v2 == rhs.v2 && this->weight == rhs.weight;
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

struct State {
    Configuration configuration;
    std::vector<std::set<Edge>> adjacencyMatrix;
    std::set<Edge> incidentEdges;
    int remainingValue;
    int currentValue;

public:
    State(
        const Configuration& configuration,
        const std::vector<std::set<Edge>>& adjacencyMatrix,
        const std::set<Edge>& incidentEdges,
        const int remainingValue,
        const int currentValue
    ) : configuration(configuration), 
        adjacencyMatrix(adjacencyMatrix), 
        incidentEdges(incidentEdges),
        remainingValue(remainingValue),
        currentValue(currentValue) {}

    const Edge& getHeaviestIncidentEdge() const { 
        return *this->incidentEdges.begin();
    }

    std::optional<State> next(bool isEdgeAdded) const {
        State newState = *this;
        Edge heaviestEdge = newState.getHeaviestIncidentEdge();
        newState.incidentEdges.erase(heaviestEdge);
        newState.adjacencyMatrix[heaviestEdge.v1].erase(heaviestEdge);
        newState.adjacencyMatrix[heaviestEdge.v2].erase(heaviestEdge);
        newState.remainingValue -= heaviestEdge.weight;

        if (!isEdgeAdded) return newState;

        VertexColor c1 = VertexColor::UNASSIGNED;
        VertexColor c2 = VertexColor::UNASSIGNED;
        if (this->isColorAssignmentValid(heaviestEdge, VertexColor::RED, VertexColor::BLUE, newState.configuration)) {
            c1 = VertexColor::RED;
            c2 = VertexColor::BLUE;
        }
        if (this->isColorAssignmentValid(heaviestEdge, VertexColor::BLUE, VertexColor::RED, newState.configuration)) {
            c1 = VertexColor::BLUE;
            c2 = VertexColor::RED;
        }
        if (c1 != VertexColor::UNASSIGNED && c2 != VertexColor::UNASSIGNED) {
            newState.configuration.coloring[heaviestEdge.v1] = c1;
            newState.configuration.coloring[heaviestEdge.v2] = c2;
            newState.configuration.addedEdges.insert(heaviestEdge);
            newState.currentValue += heaviestEdge.weight;

            for (auto e: this->adjacencyMatrix[heaviestEdge.v1]) {
                if(e == heaviestEdge) continue;
                newState.adjacencyMatrix[e.v1].erase(e);
                newState.adjacencyMatrix[e.v2].erase(e);
                newState.incidentEdges.insert(e);
            }

            for (auto e: this->adjacencyMatrix[heaviestEdge.v2]) {
                if(e == heaviestEdge) continue;
                newState.adjacencyMatrix[e.v1].erase(e);
                newState.adjacencyMatrix[e.v2].erase(e);
                newState.incidentEdges.insert(e);
            }

            return newState;
        }

        return std::nullopt;
    }

private:
    const bool isColorAssignmentValid(const Edge &edge, VertexColor c1, VertexColor c2, const Configuration &configuration) const {
        int v1 = edge.v1;
        int v2 = edge.v2;

        return (configuration.coloring[v1] == VertexColor::UNASSIGNED || configuration.coloring[v1] == c1) &&
               (configuration.coloring[v2] == VertexColor::UNASSIGNED || configuration.coloring[v2] == c2);
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

std::deque<State> generateStates(const State& initialState, const size_t maxStates ) {
    std::deque<State> states;
    states.push_back(initialState);

    while(states.size() < maxStates) {
        State front = states.front();
        states.pop_front();

        std::optional<State> addedState = front.next(true);
        std::optional<State> notAddedState = front.next(false);

        if (addedState) states.push_back(*addedState);
        if (notAddedState) states.push_back(*notAddedState);
    }

    return states;
}

void solve(int &recursionCnt, int &bestValue, std::set<Configuration> &results, const State& state) {
    #pragma omp atomic update
    recursionCnt++;

    if (state.currentValue >= bestValue) {
        #pragma omp critical
        {
            // Better value was found.
            if (state.currentValue > bestValue) {
                bestValue = state.currentValue;
                results.clear();
            }
            
            // If current value is the best, mark added edges as solution.
            if (state.currentValue == bestValue) {
                results.insert(state.configuration);
            }
        }
    }
    
    // Better solution can't be found.
    if (state.currentValue + state.remainingValue < bestValue) { return; }

    // No opened edges are left.
    if (state.incidentEdges.empty()) { return; }

    // Better solution can't be found.
    if (state.currentValue + state.remainingValue < bestValue) { return; }

    // No incident edges are left.
    if (state.incidentEdges.empty()) { return; }

    // Add edge to the solution if valid coloring exists.
    std::optional<State> edgeAddedState = state.next(true);
    if (edgeAddedState) {
        solve(recursionCnt, bestValue, results, *edgeAddedState);
    }

    // Do not add edge to the solution.
    std::optional<State> edgeNotAddedState = state.next(false);
    if (edgeNotAddedState) {
        solve(recursionCnt, bestValue, results, *edgeNotAddedState);
    }
}

int main(int argc, char *argv[]) {
    const int max_threads = omp_get_max_threads();
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

    State initialState(Configuration(coloring, addedEdges), adjacencyMatrix, openedEdges, totalWeight, 0);
    std::deque<State> states = generateStates(initialState, max_threads * 10);

    auto startTime = std::chrono::high_resolution_clock::now();

    #pragma omp parallel for shared(recursionCnt, bestValue, results) firstprivate(states) default(none)
    for (size_t i = 0; i < states.size(); i++) {
        solve(recursionCnt, bestValue, results, states[i]);
    }

    auto endTime = std::chrono::high_resolution_clock::now();

    printSolution(bestValue, recursionCnt, results, endTime - startTime);
}