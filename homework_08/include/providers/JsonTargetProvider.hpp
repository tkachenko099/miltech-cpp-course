#pragma once

#include "Types.hpp"
#include "interfaces/ITargetProvider.hpp"

#include <string>
#include <vector>

class JsonTargetProvider final
    : public ITargetProvider
{
public:
    explicit JsonTargetProvider(
        const std::string& filename
    );

    int getTargetCount() const override;

    TargetState getTarget(
        int index,
        float time,
        float dt
    ) const override;

private:
    void load(
        const std::string& filename
    );

    Coord interpolate(
        int targetIndex,
        float time
    ) const;

private:
    std::vector<std::vector<Coord>>
        targets_{};

    int timeSteps_{};

    float arrayTimeStep_{1.0f};
};