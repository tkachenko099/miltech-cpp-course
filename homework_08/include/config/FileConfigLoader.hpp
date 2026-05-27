#pragma once

#include "Types.hpp"
#include "interfaces/IConfigLoader.hpp"

#include <string>
#include <vector>

class FileConfigLoader final
    : public IConfigLoader
{
public:
    void load(
        const std::string& configPath,
        const std::string& ammoPath
    ) override;

    const DroneConfig& getConfig() const override;

    const AmmoParams& getAmmoParams() const override;

private:
    static DroneConfig readConfig(
        const std::string& filename
    );

    static std::vector<AmmoParams>
    readAmmoList(
        const std::string& filename
    );

    static AmmoParams findAmmo(
        const std::string& ammoName,
        const std::vector<AmmoParams>& ammoList
    );

    static void validateConfig(
        const DroneConfig& cfg
    );

private:
    DroneConfig cfg_{};

    AmmoParams ammo_{};
};