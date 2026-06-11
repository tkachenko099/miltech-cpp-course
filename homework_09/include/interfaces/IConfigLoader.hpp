#pragma once

#include "Types.hpp"

#include <string>

class IConfigLoader
{
public:
    virtual void load(
        const std::string& configPath,
        const std::string& ammoPath
    ) = 0;

    virtual const DroneConfig& getConfig() const = 0;

    virtual const AmmoParams& getAmmoParams() const = 0;

    virtual ~IConfigLoader() = default;
};