#include "providers/JsonTargetProvider.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include "json.hpp"

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(
    const std::string& filename
)
{
    load(filename);
}

int JsonTargetProvider::getTargetCount() const
{
    return static_cast<int>(
        targets_.size()
    );
}

TargetState JsonTargetProvider::getTarget(
    int index,
    float time,
    float dt
) const
{
    if (index < 0 ||
        index >= getTargetCount())
    {
        throw std::out_of_range(
            "target index out of range"
        );
    }

    if (dt <= 0.0f)
    {
        throw std::runtime_error(
            "invalid dt"
        );
    }

    const Coord p1 =
        interpolate(
            index,
            time
        );

    const Coord p2 =
        interpolate(
            index,
            time + dt
        );

    return {
        .pos = p1,
        .vel = (p2 - p1) / dt
    };
}

void JsonTargetProvider::load(
    const std::string& filename
)
{
    std::ifstream fin(filename);

    if (!fin.is_open())
    {
        throw std::runtime_error(
            "cannot open targets file"
        );
    }

    json j;
    fin >> j;

    const int targetCount =
        j["targetCount"];

    timeSteps_ =
        j["timeSteps"];

    if (targetCount <= 0 ||
        timeSteps_ <= 0)
    {
        throw std::runtime_error(
            "invalid targets file"
        );
    }

    if (j.contains("targetArrayTimeStep"))
    {
        arrayTimeStep_ =
            j["targetArrayTimeStep"];
    }

    targets_.assign(
        static_cast<std::size_t>(
            targetCount
        ),
        std::vector<Coord>(
            static_cast<std::size_t>(
                timeSteps_
            )
        )
    );

    for (int i = 0;
         i < targetCount;
         ++i)
    {
        for (int k = 0;
             k < timeSteps_;
             ++k)
        {
            targets_[i][k].x =
                j["targets"][i]
                 ["positions"][k]["x"];

            targets_[i][k].y =
                j["targets"][i]
                 ["positions"][k]["y"];
        }
    }
}

Coord JsonTargetProvider::interpolate(
    int targetIndex,
    float time
) const
{
    if (timeSteps_ <= 0)
    {
        throw std::runtime_error(
            "no target time steps"
        );
    }

    int idx =
        static_cast<int>(
            std::floor(
                time / arrayTimeStep_
            )
        ) % timeSteps_;

    if (idx < 0)
    {
        idx += timeSteps_;
    }

    const int next =
        (idx + 1) % timeSteps_;

    const float baseTime =
        std::floor(
            time / arrayTimeStep_
        ) * arrayTimeStep_;

    const float frac =
        (time - baseTime)
        / arrayTimeStep_;

    const Coord a =
        targets_[targetIndex][idx];

    const Coord b =
        targets_[targetIndex][next];

    return {
        a.x + (b.x - a.x) * frac,
        a.y + (b.y - a.y) * frac
    };
}