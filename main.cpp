//
// Created by David Trinh on 16/02/2022.
//

#include <iostream>
#include "src/utils/factories/WeightedGraphFactory.h"

int main(int argc, char *argv[]) {
    std::string path = argv[1];
    WeightedGraph inputGraph = WeightedGraphFactory::fromFile(path);

    return 0;
}