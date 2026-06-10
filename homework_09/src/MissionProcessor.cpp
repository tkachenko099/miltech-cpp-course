#include "MissionProcessor.hpp"

#include "states/DroneStates.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

MissionProcessor::MissionProcessor(
    std::unique_ptr<ITargetProvider> targets,
    std::unique_ptr<IBallisticSolver> solver,
    std::unique_ptr<IConfigLoader> loader
)
    : targets_{std::move(targets)},
      solver_{std::move(solver)},
      loader_{std::move(loader)},
      droneState_{std::make_unique<StateStopped>()}
{
    if (!targets_ || !solver_ || !loader_)
    {
        throw std::runtime_error(
            "MissionProcessor dependency is null"
        );
    }
}

void MissionProcessor::init(
    const std::string& configPath,
    const std::string& ammoPath
)
{
    loader_->load(
        configPath,
        ammoPath
    );

    cfg_ =
        loader_->getConfig();

    ammo_ =
        loader_->getAmmoParams();

    droneCtx_.position =
        cfg_.startPos;

    droneCtx_.speed =
        0.0f;

    droneCtx_.acceleration =
        (cfg_.attackSpeed * cfg_.attackSpeed)
        / (2.0f * cfg_.accelPath);

    droneCtx_.direction =
        cfg_.initialDir;

    droneCtx_.desiredDir =
        cfg_.initialDir;

    droneCtx_.targetDir =
        cfg_.initialDir;

    droneCtx_.turnRemaining =
        0.0f;

    droneCtx_.cfg =
        &cfg_;

    droneState_ =
        std::make_unique<StateStopped>();

    currentIdx_ =
        0;

    currentTime_ =
        0.0f;

    steps_.clear();
}

bool MissionProcessor::hasNext() const
{
    return currentIdx_
        < targets_->getTargetCount();
}

DropPoint MissionProcessor::step()
{
    if (!hasNext())
    {
        throw std::runtime_error(
            "no more targets"
        );
    }

    const TargetState target =
        targets_->getTarget(
            currentIdx_,
            currentTime_,
            cfg_.simTimeStep
        );

    DropPoint result =
        solver_->solve(
            droneCtx_.position,
            cfg_.altitude,
            target.pos,
            cfg_.attackSpeed,
            cfg_.accelPath,
            ammo_
        );

    result.targetIndex =
        currentIdx_;

    if (result.valid)
    {
        const Coord toDrop =
            result.dropPoint
            - droneCtx_.position;

        result.desiredDir =
            std::atan2(
                toDrop.y,
                toDrop.x
            );

        result.predictedTarget =
            target.pos
            + target.vel
              * result.flightTime;

        result.totalCost =
            distance2D(
                droneCtx_.position,
                result.dropPoint
            );

        droneCtx_.desiredDir =
            result.desiredDir;

        saveStep(result);

        auto nextState =
            droneState_->execute(
                droneCtx_
            );

        if (nextState)
        {
            droneState_ =
                std::move(nextState);
        }
    }

    ++currentIdx_;

    currentTime_ +=
        cfg_.simTimeStep;

    return result;
}

void MissionProcessor::reset()
{
    droneCtx_.position =
        cfg_.startPos;

    droneCtx_.speed =
        0.0f;

    droneCtx_.acceleration =
        (cfg_.attackSpeed * cfg_.attackSpeed)
        / (2.0f * cfg_.accelPath);

    droneCtx_.direction =
        cfg_.initialDir;

    droneCtx_.desiredDir =
        cfg_.initialDir;

    droneCtx_.targetDir =
        cfg_.initialDir;

    droneCtx_.turnRemaining =
        0.0f;

    droneCtx_.cfg =
        &cfg_;

    droneState_ =
        std::make_unique<StateStopped>();

    currentIdx_ =
        0;

    currentTime_ =
        0.0f;

    steps_.clear();
}

void MissionProcessor::changeSolver(
    std::unique_ptr<IBallisticSolver> solver
)
{
    if (!solver)
    {
        throw std::runtime_error(
            "new solver is null"
        );
    }

    solver_ =
        std::move(solver);
}

const std::vector<SimStep>&
MissionProcessor::getSteps() const
{
    return steps_;
}

void MissionProcessor::saveStep(
    const DropPoint& result
)
{
    const Coord dirVec{
        std::cos(droneCtx_.direction),
        std::sin(droneCtx_.direction)
    };

    SimStep step{};

    step.pos =
        droneCtx_.position;

    step.direction =
        droneCtx_.direction;

    step.stateName =
        droneState_->name();

    step.targetIdx =
        result.targetIndex;

    step.dropPoint =
        result.dropPoint;

    step.aimPoint =
        droneCtx_.position
        + dirVec
          * result.horizontalRange;

    step.predictedTarget =
        result.predictedTarget;

    step.flightTime =
        result.flightTime;

    step.horizontalRange =
        result.horizontalRange;

    steps_.push_back(
        step
    );
}