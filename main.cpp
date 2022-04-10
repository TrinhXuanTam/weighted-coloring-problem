#include <iostream>
#include <set>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>
#include <optional>
#include <mpi.h>
#include <omp.h>
#include <fstream>

#define BUFFER_SIZE 512
#define MASTER_SOURCE 0
#define TAG_KILL 1
#define TAG_DO 2
#define TAG_DONE 3

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

    std::vector<int> serialize() {
        std::vector<int> buffer;

        buffer.push_back(index);
        buffer.push_back(currentWeight);
        buffer.push_back(remainingWeight);
        buffer.push_back(this->edges.size());
        buffer.push_back(this->configuration.coloring.size());

        for (const auto& edge : this->edges) {
            buffer.push_back(edge.v1);
            buffer.push_back(edge.v2);
            buffer.push_back(edge.weight);
        }

        for (const auto& VertexColor : this->configuration.coloring) {
            buffer.push_back(VertexColor);
        }

        for (const auto& edgeIndex : this->configuration.edges) {
            buffer.push_back(edgeIndex);
        }
            
        buffer.push_back(-1);
        return buffer;
    }

    static State deserialize(std::vector<int> buffer) {
        int index = buffer[0];
        int currentWeight = buffer[1];
        int remainingWeight = buffer[2];
        int edgeCnt = buffer[3];
        int vertexCnt = buffer[4];
        std::vector<Edge> edges;
        std::vector<VertexColor> coloring;
        std::vector<int> edgeIndexes;

        int i = 5;
        while (i < edgeCnt * 3 + 5) {
            edges.emplace_back(buffer[i], buffer[i + 1], buffer[i + 2]);
            i += 3;
        }

        int j = i;
        while (i < vertexCnt + j) {
            coloring.emplace_back(static_cast<VertexColor>(buffer[i]));
            i++;
        }

        while (buffer[i] != -1) {
            edgeIndexes.emplace_back(buffer[i]);
            i++;
        }

        return State(edges, Configuration(coloring, edgeIndexes), remainingWeight, currentWeight, index);
    }
};

struct Result {
    int weight;
    std::vector<Configuration> configurations;

    Result(int weight, std::vector<Configuration>& configurations) : weight(weight), configurations(configurations) {}

    std::vector<int> serialize() {
      return Result::serialize(this->weight, this->configurations);
    }

    static std::vector<int> serialize(const int weight, const std::vector<Configuration>& configurations) {
        std::vector<int> buffer;

        buffer.emplace_back(weight);

        for (const auto& configuration : configurations) {
            buffer.emplace_back(configuration.coloring.size());
            for (const auto& color : configuration.coloring) {
                buffer.emplace_back(color);
            }

            buffer.emplace_back(configuration.edges.size());
            for(const auto& edgeIndex : configuration.edges) {
                buffer.emplace_back(edgeIndex);
            }
        }
        buffer.emplace_back(-1);

        return buffer;
    };

    static Result deserialize(const std::vector<int>& buffer) {
        int weight = buffer[0];
        std::vector<Configuration> configurations;


        int i = 1;
        while (buffer[i] != -1) {
            std::vector<VertexColor> coloring;
            std::vector<int> edges;
            
            int coloringSize = buffer[i++];
            for (int j = i; i < j + coloringSize; i++) {
                coloring.emplace_back(static_cast<VertexColor>(buffer[i]));
            }

            int edgeSize = buffer[i++];
            for (int j = i; i < j + edgeSize; i++) {
                edges.emplace_back(buffer[i]);
            }

            configurations.emplace_back(coloring, edges);
        }

        return Result(weight, configurations);
    };
};

bool loadFromFile(const std::string& filename, int& vertexCnt, std::vector<Edge>& edges) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file >> vertexCnt;
    for (int i = 0; i < vertexCnt; ++i) {
        for (int j = 0; j < vertexCnt; ++j) {
            int weight;
            file >> weight;
            if (weight != 0 && j >= i) {
                edges.emplace_back(i, j, weight);
            }
        }
    }
    return true;
}

void printSolution(
    const std::vector<Edge>& edges,
    const int &bestWeight, 
    const std::vector<Configuration> &results, 
    const std::chrono::duration<double> time
) {
    std::cout << "Total time: " << time.count() << "s" << std::endl;
    std::cout << "Max weight: " << bestWeight << std::endl;
    
    std::set<std::set<int>> uniqueEdges;
    std::vector<Configuration> uniqueResults;

    for (const auto &result: results) {
        std::set<int> solutionEdges(result.edges.begin(), result.edges.end());
        if (uniqueEdges.find(solutionEdges) == uniqueEdges.end()) {
            uniqueEdges.insert(solutionEdges);
            uniqueResults.push_back(result);
        } else {
            continue;
        }
    }

    std::cout << "Solutions: " << uniqueResults.size() << std::endl;
    std::cout << std::endl;

    int solutionNo = 1;
    for (const auto &result: uniqueResults) {
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

std::deque<State> generateStates(const State& initialState, const size_t maxStates) {
    std::deque<State> states;
    states.push_back(initialState);

    while(states.size() < maxStates) {
        State front = states.front();
        states.pop_front();

        std::optional<State> blueRedState = front.next(VertexColor::BLUE, VertexColor::RED);
        std::optional<State> redBlueState = front.next(VertexColor::RED, VertexColor::BLUE);
        std::optional<State> notAddedState = front.next(VertexColor::UNASSIGNED, VertexColor::UNASSIGNED);

        if (blueRedState) states.push_back(*blueRedState);
        if (redBlueState) states.push_back(*redBlueState);
        if (notAddedState) states.push_back(*notAddedState);
        if (!blueRedState && !redBlueState && !notAddedState) {
            break;
        }
    }

    return states;
}


void solve(int &bestWeight, std::vector<Configuration> &results, const State& state) {
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
        solve(bestWeight, results, *blueRedState);
    } 
    if (redBlueState) {
        solve(bestWeight, results, *redBlueState);
    }

    // Do not add edge to the solution.
    std::optional<State> edgeNotAddedState = state.next(VertexColor::UNASSIGNED, VertexColor::UNASSIGNED);
    if (edgeNotAddedState) {
        solve(bestWeight, results, *edgeNotAddedState);
    }

}

void master(std::string filename, int procCnt) {
    int slavesCnt = procCnt - 1;
    int vertexCnt = 0;
    int bestWeight = 0;
    std::vector<Edge> edges;
    std::vector<Configuration> results;

    if (!loadFromFile(filename, vertexCnt, edges)) {
        std::cout << "Couldn't open the input file." << std::endl;
        return;
    }

    std::sort(edges.begin(), edges.end(), Edge::cmp);
    std::deque<State> states = generateStates(State(vertexCnt, edges), procCnt * 10);
    auto startTime = std::chrono::high_resolution_clock::now();

    for (int i = 1; i <= slavesCnt && !states.empty(); i++) {
        State state = states.front();
        std::vector<int> stateData = state.serialize();
        MPI_Send(stateData.data(), (int) stateData.size(), MPI_INT, i, TAG_DO, MPI_COMM_WORLD);
        states.pop_front();
    };

    while (slavesCnt > 0) {
        std::vector<int> buffer(BUFFER_SIZE);
        MPI_Status status;
        MPI_Recv(buffer.data(), BUFFER_SIZE, MPI_INT, MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
        
        if (status.MPI_TAG == TAG_DONE) {
            Result deserializedResult = Result::deserialize(buffer);
            if (bestWeight < deserializedResult.weight) {
                results.clear();
                bestWeight = deserializedResult.weight;
                results = deserializedResult.configurations;
            }

            if (bestWeight == deserializedResult.weight) {
                for (const auto& configuration: deserializedResult.configurations) {
                    results.push_back(configuration);
                }
            }
        }

        if (states.empty()) {
            int dummyData = 0;
            MPI_Send(&dummyData, 1, MPI_INT, status.MPI_SOURCE, TAG_KILL, MPI_COMM_WORLD);
            slavesCnt--;
        } else {
            State state = states.front();
            std::vector<int> stateData = state.serialize();
            MPI_Send(stateData.data(), (int) stateData.size(), MPI_INT, status.MPI_SOURCE, TAG_DO, MPI_COMM_WORLD);
            states.pop_front();
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();

    printSolution(edges, bestWeight, results, endTime - startTime);
}

void slave(int maxThreads) {
    std::vector<int> buffer(BUFFER_SIZE);
    MPI_Status status;

    while (true) {
        MPI_Recv(buffer.data(), BUFFER_SIZE, MPI_INT, MASTER_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);

        if (status.MPI_TAG == TAG_DO) {
            int bestWeight = 0;
            std::vector<Configuration> results;
            State parsedState = State::deserialize(buffer);
            std::deque<State> states = generateStates(parsedState, maxThreads * 10);

            #pragma omp parallel for shared(bestWeight, results) firstprivate(states) default(none)
            for (size_t i = 0; i < states.size(); i++) {
                solve(bestWeight, results, states[i]);
            }

            std::vector<int> serializedResult = Result::serialize(bestWeight, results);
            MPI_Send(serializedResult.data(), (int) serializedResult.size(), MPI_INT, MASTER_SOURCE, TAG_DONE, MPI_COMM_WORLD);
        }

        if (status.MPI_TAG == TAG_KILL) {
            break;
        }
    }
    
}

int main(int argc, char **argv){
    int rank, procCnt;

    if (argc < 2) {
        return 1;
    }

    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &procCnt);

    if(rank == 0) {
        master(argv[1], procCnt);
    } else {
        const int max_threads = omp_get_max_threads();
        slave(max_threads);
    }

    MPI_Finalize();
    return 0;
}
