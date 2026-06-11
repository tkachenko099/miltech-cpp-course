#pragma once

#include "Types.hpp"

#include <string>
#include <vector>

class SimulationWriter
{
public:
    static void writeJson(
        const std::string& filename,
        const std::vector<SimStep>& steps
    );
};