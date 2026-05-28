#include "solvers/AnalyticalSolver.hpp"

#include <algorithm>
#include <cmath>

float AnalyticalSolver::computeHorizontalRange(
    float flightTime,
    float attackSpeed,
    const AmmoParams& ammo
)
{
    return
          attackSpeed * flightTime
        - std::pow(flightTime, 2.0f)
          * ammo.drag
          * attackSpeed
          / (2.0f * ammo.mass);
}

DropPoint AnalyticalSolver::solve(
    Coord dronePos,
    float droneAltitude,
    Coord targetPos,
    float attackSpeed,
    float accelerationPath,
    const AmmoParams& ammo
) const
{
    DropPoint result{};

    const float a =
          ammo.drag
          * G
          * ammo.mass
        - 2.0f
          * ammo.drag
          * ammo.drag
          * ammo.lift
          * attackSpeed;

    const float b =
          -3.0f
          * G
          * ammo.mass
          * ammo.mass
        + 3.0f
          * ammo.drag
          * ammo.lift
          * ammo.mass
          * attackSpeed;

    const float c =
          6.0f
          * ammo.mass
          * ammo.mass
          * droneAltitude;

    if (std::fabs(a) < EPS)
    {
        return result;
    }

    const float p =
        -(b * b)
        / (3.0f * a * a);

    const float q =
          (2.0f * b * b * b)
          / (27.0f * a * a * a)
        + c / a;

    if (p >= 0.0f)
    {
        return result;
    }

    float acosArg =
          3.0f
          * q
          / (2.0f * p)
        * std::sqrt(
              -3.0f / p
          );

    acosArg = std::clamp(
        acosArg,
        -1.0f,
        1.0f
    );

    constexpr float pi =
        3.14159265358979323846f;

    const float phi =
        std::acos(acosArg);

    const float flightTime =
          2.0f
          * std::sqrt(-p / 3.0f)
          * std::cos(
                (phi + 4.0f * pi)
                / 3.0f
            )
        - b / (3.0f * a);

    if (flightTime <= 0.0f)
    {
        return result;
    }

    const float horizontalRange =
        computeHorizontalRange(
            flightTime,
            attackSpeed,
            ammo
        );

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

    result.dropPoint =
        firePoint;

    result.flightTime =
        flightTime;

    result.horizontalRange =
        horizontalRange;

    return result;
}