module;

#include "json.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

export module file_config_loader;

import interfaces;
import mission_types;

using json = nlohmann::json;

export class FileConfigLoader final
    : public IConfigLoader
{
private:

    DroneConfig cfg_{};
    AmmoParams ammo_{};

    static DroneConfig readConfig(
        const char* filename
    )
    {
        std::ifstream fin(filename);

        if(!fin.is_open())
        {
            throw std::runtime_error(
                "cannot open config"
            );
        }

        json j;
        fin>>j;

        DroneConfig cfg{};

        cfg.startPos.x=
            j["drone"]["position"]["x"];

        cfg.startPos.y=
            j["drone"]["position"]["y"];

        cfg.altitude=
            j["drone"]["altitude"];

        cfg.initialDir=
            j["drone"]["initialDirection"];

        cfg.attackSpeed=
            j["drone"]["attackSpeed"];

        cfg.accelPath=
            j["drone"]["accelerationPath"];

        cfg.angularSpeed=
            j["drone"]["angularSpeed"];

        cfg.turnThreshold=
            j["drone"]["turnThreshold"];

        cfg.ammoName=
            j["ammo"].get<std::string>();

        cfg.simTimeStep=
            j["simulation"]["timeStep"];

        cfg.hitRadius=
            j["simulation"]["hitRadius"];

        cfg.arrayTimeStep=
            j["targetArrayTimeStep"];

        return cfg;
    }

    static std::vector<AmmoParams>
    readAmmoList(
        const char* filename
    )
    {
        std::ifstream fin(filename);

        if(!fin.is_open())
        {
            throw std::runtime_error(
                "cannot open ammo"
            );
        }

        json j;
        fin>>j;

        std::vector<AmmoParams> ammoList;

        for(const auto& item : j)
        {
            AmmoParams ammo{};

            ammo.name=
                item["name"];

            ammo.mass=
                item["mass"];

            ammo.drag=
                item["drag"];

            ammo.lift=
                item["lift"];

            ammoList.push_back(
                ammo
            );
        }

        return ammoList;
    }

    static AmmoParams findAmmo(
        const std::string& name,
        const std::vector<AmmoParams>& list
    )
    {
        for(
            const auto& ammo
            :
            list
        )
        {
            if(ammo.name==name)
            {
                return ammo;
            }
        }

        throw std::runtime_error(
            "ammo not found"
        );
    }

    static void validateConfig(
        const DroneConfig& cfg
    )
    {
        if(
            cfg.simTimeStep<=0.0f ||
            cfg.attackSpeed<=0.0f ||
            cfg.accelPath<=0.0f
        )
        {
            throw std::runtime_error(
                "invalid config"
            );
        }
    }

public:

    void load(
        const char* configPath,
        const char* ammoPath
    ) override
    {
        cfg_=
            readConfig(
                configPath
            );

        auto ammoList=
            readAmmoList(
                ammoPath
            );

        ammo_=
            findAmmo(
                cfg_.ammoName,
                ammoList
            );

        validateConfig(
            cfg_
        );
    }

    const DroneConfig&
    getConfig() const override
    {
        return cfg_;
    }

    const AmmoParams&
    getAmmoParams() const override
    {
        return ammo_;
    }
};