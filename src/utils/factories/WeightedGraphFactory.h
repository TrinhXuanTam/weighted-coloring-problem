//
// Created by David Trinh on 23/02/2022.
//

#ifndef NI_PDP_WEIGHTEDGRAPHFACTORY_H
#define NI_PDP_WEIGHTEDGRAPHFACTORY_H

#include <string>
#include "../../models/WeightedGraph.h"

class WeightedGraphFactory {
public:
    static WeightedGraph fromFile(std::string &path);
};


#endif //NI_PDP_WEIGHTEDGRAPHFACTORY_H
