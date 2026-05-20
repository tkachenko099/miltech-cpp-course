export module json_target_provider;
        }

        const Coord p1 = interpolate(index, time);
        const Coord p2 = interpolate(index, time + dt);

        return TargetState{
            .pos = p1,
            .vel = (p2 - p1) / dt
        };
    }

private:
    std::vector<std::vector<Coord>> targets_{};
    int timeSteps_{};
    float arrayTimeStep_{1.0f};

    void load(const char* filename)
    {
        std::ifstream fin(filename);
        if (!fin.is_open())
        {
            throw std::runtime_error("cannot open targets file");
        }

        json j;
        fin >> j;

        const int targetCount = j["targetCount"];
        timeSteps_ = j["timeSteps"];

        if (targetCount <= 0 || timeSteps_ <= 0)
        {
            throw std::runtime_error("invalid target file");
        }

        if (j.contains("targetArrayTimeStep"))
        {
            arrayTimeStep_ = j["targetArrayTimeStep"];
        }

        targets_.assign(
            static_cast<std::size_t>(targetCount),
            std::vector<Coord>(static_cast<std::size_t>(timeSteps_))
        );

        for (int i = 0; i < targetCount; ++i)
        {
            for (int k = 0; k < timeSteps_; ++k)
            {
                targets_[i][k].x = j["targets"][i]["positions"][k]["x"];
                targets_[i][k].y = j["targets"][i]["positions"][k]["y"];
            }
        }
    }

    Coord interpolate(
        int targetIndex,
        float time
    ) const
    {
        if (timeSteps_ <= 0)
        {
            throw std::runtime_error("no target time steps");
        }

        int idx = static_cast<int>(std::floor(time / arrayTimeStep_)) % timeSteps_;
        if (idx < 0)
        {
            idx += timeSteps_;
        }

        const int next = (idx + 1) % timeSteps_;

        const float baseTime = std::floor(time / arrayTimeStep_) * arrayTimeStep_;
        const float frac = (time - baseTime) / arrayTimeStep_;

        const Coord a = targets_[targetIndex][idx];
        const Coord b = targets_[targetIndex][next];

        return {
            a.x + (b.x - a.x) * frac,
            a.y + (b.y - a.y) * frac
        };
    }
};