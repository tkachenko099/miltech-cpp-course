#pragma once

#include "Types.hpp"

class IBallisticSolver
{
public:
    virtual DropPoint solve(
        Coord dronePos,
        float droneAltitude,
        Coord targetPos,
        float attackSpeed,
        float accelerationPath,
        const AmmoParams& ammo
    ) const = 0;

    virtual ~IBallisticSolver() = default;
};