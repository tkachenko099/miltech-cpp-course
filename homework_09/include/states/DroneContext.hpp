#pragma once

#include "Types.hpp"

struct DroneContext
{
    Coord position{};

    float speed{};
    float acceleration{};
    float direction{};

    float desiredDir{};
    float targetDir{};
    float turnRemaining{};

    const DroneConfig* cfg{};
};