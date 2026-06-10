#pragma once

#include "Types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "solvers/BallisticTable.hpp"

#include <string>

class TableSolver final : public IBallisticSolver
{
public:
    explicit TableSolver(
        const std::string& tablePath
    );

    DropPoint solve(
        Coord dronePos,
        float droneAltitude,
        Coord targetPos,
        float attackSpeed,
        float accelerationPath,
        const AmmoParams& ammo
    ) const override;

private:
    BallisticTable table_;
};