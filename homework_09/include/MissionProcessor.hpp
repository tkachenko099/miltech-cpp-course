#pragma once

#include "Types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IDroneState.hpp"

#include <memory>
#include <string>
#include <vector>

class MissionProcessor
{
public:
    MissionProcessor(
        std::unique_ptr<ITargetProvider> targets,
        std::unique_ptr<IBallisticSolver> solver,
        std::unique_ptr<IConfigLoader> loader
    );

    void init(
        const std::string& configPath,
        const std::string& ammoPath
    );

    bool hasNext() const;

    DropPoint step();

    void reset();

    void changeSolver(
        std::unique_ptr<IBallisticSolver> solver
    );

    const std::vector<SimStep>& getSteps() const;

private:
    void saveStep(
        const DropPoint& result
    );

private:
    std::unique_ptr<ITargetProvider> targets_;
    std::unique_ptr<IBallisticSolver> solver_;
    std::unique_ptr<IConfigLoader> loader_;
    std::unique_ptr<IDroneState> droneState_;

    DroneContext droneCtx_{};

    DroneConfig cfg_{};
    AmmoParams ammo_{};

    int currentIdx_{};
    float currentTime_{};

    std::vector<SimStep> steps_{};
};