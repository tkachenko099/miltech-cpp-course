#pragma once

#include "Types.hpp"
#include "interfaces/IBallisticSolver.hpp"

class AnalyticalSolver final
    : public IBallisticSolver
{
public:
    DropPoint solve(
        Coord dronePos,
        float droneAltitude,
        Coord targetPos,
        float attackSpeed,
        float accelerationPath,
        const AmmoParams& ammo
    ) const override;

private:
    static float computeHorizontalRange(
        float flightTime,
        float attackSpeed,
        const AmmoParams& ammo
    );
};