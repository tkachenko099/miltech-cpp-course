#pragma once

#include "Types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

#include <vector>

class MissionProcessor
{
public:
    MissionProcessor(
        ITargetProvider& targets,
        IBallisticSolver& solver,
        IConfigLoader& loader
    );

    void init(
        const std::string& configPath,
        const std::string& ammoPath
    );

    bool hasNext() const;

    DropPoint step();

    void reset();

    void changeSolver(
        IBallisticSolver& solver
    );

    const std::vector<SimStep>& getSteps() const;

private:
    static Drone initDrone(
        const DroneConfig& cfg
    );

    static float normalizeAngle(
        float angle
    );

    void updateDrone(
        const DropPoint& result
    );

    void saveStep(
        const DropPoint& result
    );

private:
    ITargetProvider& targets_;
    IBallisticSolver* solver_{};
    IConfigLoader& loader_;

    Drone drone_{};
    DroneConfig cfg_{};
    AmmoParams ammo_{};

    int currentIdx_{};
    float currentTime_{};

    std::vector<SimStep> steps_{};
};