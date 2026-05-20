export module file_config_loader;
    DroneConfig cfg_{};
    AmmoParams ammo_{};

    static DroneConfig readConfig(const char* filename)
    {
        std::ifstream fin(filename);
        if (!fin.is_open())
        {
            throw std::runtime_error("cannot open config file");
        }

        json j;
        fin >> j;

        DroneConfig cfg{};
        cfg.startPos.x = j["drone"]["position"]["x"];
        cfg.startPos.y = j["drone"]["position"]["y"];
        cfg.altitude = j["drone"]["altitude"];
        cfg.initialDir = j["drone"]["initialDirection"];
        cfg.attackSpeed = j["drone"]["attackSpeed"];
        cfg.accelPath = j["drone"]["accelerationPath"];
        cfg.angularSpeed = j["drone"]["angularSpeed"];
        cfg.turnThreshold = j["drone"]["turnThreshold"];
        cfg.ammoName = j["ammo"].get<std::string>();
        cfg.simTimeStep = j["simulation"]["timeStep"];
        cfg.hitRadius = j["simulation"]["hitRadius"];
        cfg.arrayTimeStep = j["targetArrayTimeStep"];

        return cfg;
    }

    static std::vector<AmmoParams> readAmmoList(const char* filename)
    {
        std::ifstream fin(filename);
        if (!fin.is_open())
        {
            throw std::runtime_error("cannot open ammo file");
        }

        json j;
        fin >> j;

        std::vector<AmmoParams> ammoList;
        ammoList.reserve(j.size());

        for (const auto& item : j)
        {
            AmmoParams ammo{};
            ammo.name = item["name"].get<std::string>();
            ammo.mass = item["mass"];
            ammo.drag = item["drag"];
            ammo.lift = item["lift"];
            ammoList.push_back(ammo);
        }

        return ammoList;
    }

    static AmmoParams findAmmo(
        const std::string& ammoName,
        const std::vector<AmmoParams>& ammoList
    )
    {
        for (const auto& ammo : ammoList)
        {
            if (ammo.name == ammoName)
            {
                return ammo;
            }
        }

        throw std::runtime_error("ammo not found: " + ammoName);
    }

    static void validateConfig(const DroneConfig& cfg)
    {
        if (cfg.simTimeStep <= 0.0f ||
            cfg.arrayTimeStep <= 0.0f ||
            cfg.accelPath <= 0.0f ||
            cfg.attackSpeed <= 0.0f ||
            cfg.angularSpeed <= 0.0f)
        {
            throw std::runtime_error("invalid config values");
        }
    }
};