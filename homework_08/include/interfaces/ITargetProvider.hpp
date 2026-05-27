#pragma once

#include "Types.hpp"

class ITargetProvider
{
public:
    virtual int getTargetCount() const = 0;

    virtual TargetState getTarget(
        int index,
        float time,
        float dt
    ) const = 0;

    virtual ~ITargetProvider() = default;
};