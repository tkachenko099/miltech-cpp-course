module;

#include "json.hpp"

#include <cmath>
#include <fstream>
#include <vector>

export module json_target_provider;

import interfaces;
import mission_types;

using json=nlohmann::json;

export class JsonTargetProvider final
    : public ITargetProvider
{
private:

    std::vector<std::vector<Coord>>
        targets_;

    int timeSteps_{};
    float arrayTimeStep_{1.0f};

    Coord interpolate(
        int targetIndex,
        float t
    ) const
    {
        int idx=
            static_cast<int>(
                std::floor(
                    t/arrayTimeStep_
                )
            )%timeSteps_;

        int next=
            (idx+1)%timeSteps_;

        Coord a=
            targets_[targetIndex][idx];

        Coord b=
            targets_[targetIndex][next];

        return a+(b-a)*0.5f;
    }

public:

    explicit JsonTargetProvider(
        const char* filename
    )
    {
    }

    int getTargetCount()
    const override
    {
        return targets_.size();
    }

    TargetState getTarget(
        int index,
        float time,
        float dt
    ) const override
    {
        Coord p1=
            interpolate(
                index,
                time
            );

        Coord p2=
            interpolate(
                index,
                time+dt
            );

        return {
            p1,
            (p2-p1)/dt
        };
    }
};