#include "solvers/TableSolver.hpp"

#include <stdexcept>

TableSolver::TableSolver(
    const std::string& tablePath
)
{
    if (!table_.load(tablePath))
    {
        throw std::runtime_error(
            "cannot load ballistic table: " + tablePath
        );
    }
}

DropPoint TableSolver::solve(
    Coord dronePos,
    float droneAltitude,
    Coord targetPos,
    float attackSpeed,
    float accelerationPath,
    const AmmoParams& ammo
) const
{
    DropPoint result{};

    const BallisticResult tableResult =
        table_.lookup(
            droneAltitude,
            attackSpeed,
            ammo.mass,
            ammo.drag,
            ammo.lift
        );

    const float flightTime =
        tableResult.flightTime;

    const float horizontalRange =
        tableResult.horizontalRange;

    const Coord delta =
        targetPos - dronePos;

    const float distance =
        length(delta);

    if (distance < EPS)
    {
        return result;
    }

    const float ratio =
        (distance - horizontalRange)
        / distance;

    Coord firePoint{};

    if (horizontalRange + accelerationPath
        > distance)
    {
        const Coord maneuverPoint =
            targetPos
            - delta
              * (
                    (horizontalRange
                     + accelerationPath)
                    / distance
                );

        firePoint =
            maneuverPoint
            + (
                  targetPos
                  - maneuverPoint
              ) * ratio;
    }
    else
    {
        firePoint =
            dronePos
            + delta * ratio;
    }

    result.valid = true;
    result.dropPoint = firePoint;
    result.flightTime = flightTime;
    result.horizontalRange = horizontalRange;

    return result;
}