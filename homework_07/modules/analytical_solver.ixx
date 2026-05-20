module;

#include <algorithm>
#include <cmath>

export module analytical_solver;

import interfaces;
import mission_types;

export class AnalyticalSolver final : public IBallisticSolver
{
private:

    static float computeHorizontalRange(
        float t,
        float attackSpeed,
        const AmmoParams& ammo
    )
    {
        return
            attackSpeed * t
            - std::pow(t,2.0f)
            * ammo.drag
            * attackSpeed
            /(2.0f*ammo.mass);
    }

public:

    DropPoint solve(
        Coord dronePos,
        float droneAltitude,
        Coord targetPos,
        float attackSpeed,
        float accelerationPath,
        const AmmoParams& ammo
    ) const override
    {
        DropPoint result{};

        float t=1.0f;

        float h=
            computeHorizontalRange(
                t,
                attackSpeed,
                ammo
            );

        Coord delta=
            targetPos-dronePos;

        float D=
            length(delta);

        if(D<EPS)
        {
            return result;
        }

        float ratio=
            (D-h)/D;

        result.valid=true;
        result.flightTime=t;
        result.horizontalRange=h;
        result.dropPoint=
            dronePos+
            delta*ratio;

        return result;
    }
};