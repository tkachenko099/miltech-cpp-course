#include "config/FileConfigLoader.hpp"

#include <fstream>
#include <stdexcept>
#include <unordered_map>

#include "json.hpp"

using json = nlohmann::json;

void FileConfigLoader::load(
    const std::string& configPath,
    const std::string& ammoPath
)
{
    cfg_ = readConfig(configPath);

    const auto ammoList =
        readAmmoList(ammoPath);

    ammo_ = findAmmo(
        cfg_.ammoName,
        ammoList
    );

    validateConfig(cfg_);
}

const DroneConfig& FileConfigLoader::getConfig() const
{
    return cfg_;
}

const AmmoParams& FileConfigLoader::getAmmoParams() const
{
    return ammo_;
}

DroneConfig FileConfigLoader::readConfig(
    const std::string& filename
)
{
    std::ifstream fin(filename);

    if (!fin.is_open())
    {
        throw std::runtime_error(
            "cannot open config file"
        );
    }

    json j;
    fin >> j;

    DroneConfig cfg{};

    cfg.startPos.x =
        j["drone"]["position"]["x"];

    cfg.startPos.y =
        j["drone"]["position"]["y"];

    cfg.altitude =
        j["drone"]["altitude"];

    cfg.initialDir =
        j["drone"]["initialDirection"];

    cfg.attackSpeed =
        j["drone"]["attackSpeed"];

    cfg.accelPath =
        j["drone"]["accelerationPath"];

    cfg.angularSpeed =
        j["drone"]["angularSpeed"];

    cfg.turnThreshold =
        j["drone"]["turnThreshold"];

    cfg.ammoName =
        j["ammo"].get<std::string>();

    cfg.simTimeStep =
        j["simulation"]["timeStep"];

    cfg.hitRadius =
        j["simulation"]["hitRadius"];

    cfg.arrayTimeStep =
        j["targetArrayTimeStep"];

    return cfg;
}

std::vector<AmmoParams> FileConfigLoader::readAmmoList(
    const std::string& filename
)
{
    std::ifstream fin(filename);

    if (!fin.is_open())
    {
        throw std::runtime_error(
            "cannot open ammo file"
        );
    }

    json j;
    fin >> j;

    std::vector<AmmoParams> ammoList;
    ammoList.reserve(j.size());

    for (const auto& item : j)
    {
        AmmoParams ammo{};

        ammo.name =
            item["name"].get<std::string>();

        ammo.mass =
            item["mass"];

        ammo.drag =
            item["drag"];

        ammo.lift =
            item["lift"];

        ammoList.push_back(ammo);
    }

    return ammoList;
}

AmmoParams FileConfigLoader::findAmmo(
    const std::string& ammoName,
    const std::vector<AmmoParams>& ammoList
)
{
    std::unordered_map<std::string, AmmoParams> ammoByName;

    for (const auto& ammo : ammoList)
    {
        ammoByName.emplace(
            ammo.name,
            ammo
        );
    }

    const auto it =
        ammoByName.find(ammoName);

    if (it == ammoByName.end())
    {
        throw std::runtime_error(
            "ammo not found: " + ammoName
        );
    }

    return it->second;
}

void FileConfigLoader::validateConfig(
    const DroneConfig& cfg
)
{
    if (cfg.simTimeStep <= 0.0f ||
        cfg.arrayTimeStep <= 0.0f ||
        cfg.attackSpeed <= 0.0f ||
        cfg.accelPath <= 0.0f ||
        cfg.angularSpeed <= 0.0f)
    {
        throw std::runtime_error(
            "invalid config values"
        );
    }
}