#include "MissionProcessor.hpp"

#include "states/DroneStates.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
float normalizeAngle(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0f * M_PI;
    }

    return angle;
}
}

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
    loader_->load(configPath, ammoPath);

    cfg_ = loader_->getConfig();
    ammo_ = loader_->getAmmoParams();

    droneCtx_.position = cfg_.startPos;
    droneCtx_.speed = 0.0f;

    droneCtx_.acceleration =
        (cfg_.attackSpeed * cfg_.attackSpeed)
        / (2.0f * cfg_.accelPath);

    droneCtx_.direction = cfg_.initialDir;
    droneCtx_.desiredDir = cfg_.initialDir;
    droneCtx_.targetDir = cfg_.initialDir;
    droneCtx_.turnRemaining = 0.0f;
    droneCtx_.cfg = &cfg_;

    droneState_ = std::make_unique<StateStopped>();

    currentIdx_ = -1;
    currentTime_ = 0.0f;

    processedTargets_.assign(
        targets_->getTargetCount(),
        false
    );

    steps_.clear();
}

bool MissionProcessor::hasNext() const
{
    for (bool processed : processedTargets_)
    {
        if (!processed)
        {
            return true;
        }
    }

    return false;
}

DropPoint MissionProcessor::step()
{
    if (!hasNext())
    {
        throw std::runtime_error(
            "no more targets"
        );
    }

    const int selectedTarget =
        selectTargetByMinArrivalTime();

    if (selectedTarget < 0)
    {
        throw std::runtime_error(
            "no reachable targets"
        );
    }

    currentIdx_ =
        selectedTarget;

    const TargetState target =
        targets_->getTarget(
            currentIdx_,
            currentTime_,
            cfg_.simTimeStep
        );

    DropPoint preliminary =
        solver_->solve(
            droneCtx_.position,
            cfg_.altitude,
            target.pos,
            cfg_.attackSpeed,
            cfg_.accelPath,
            ammo_
        );

    preliminary.targetIndex =
        currentIdx_;

    if (!preliminary.valid)
    {
        processedTargets_[currentIdx_] =
            true;

        currentTime_ +=
            cfg_.simTimeStep;

        return preliminary;
    }

    const Coord predictedTarget =
        target.pos
        + target.vel * preliminary.flightTime;

    DropPoint result =
        solver_->solve(
            droneCtx_.position,
            cfg_.altitude,
            predictedTarget,
            cfg_.attackSpeed,
            cfg_.accelPath,
            ammo_
        );

    result.targetIndex =
        currentIdx_;

    result.predictedTarget =
        predictedTarget;

    if (!result.valid)
    {
        processedTargets_[currentIdx_] =
            true;

        currentTime_ +=
            cfg_.simTimeStep;

        return result;
    }

    const Coord toDrop =
        result.dropPoint
        - droneCtx_.position;

    result.desiredDir =
        std::atan2(
            toDrop.y,
            toDrop.x
        );

    result.totalCost =
        distance2D(
            droneCtx_.position,
            result.dropPoint
        );

    droneCtx_.desiredDir =
        result.desiredDir;

    auto nextState =
        droneState_->execute(
            droneCtx_
        );

    if (nextState)
    {
        droneState_ =
            std::move(nextState);
    }

    saveStep(result);

    constexpr float dropRadius =
        1.0f;

    if (distance2D(
            droneCtx_.position,
            result.dropPoint
        ) <= dropRadius)
    {
        processedTargets_[currentIdx_] =
            true;
    }

    currentTime_ +=
        cfg_.simTimeStep;

    return result;
}

void MissionProcessor::reset()
{
    droneCtx_.position = cfg_.startPos;
    droneCtx_.speed = 0.0f;

    droneCtx_.acceleration =
        (cfg_.attackSpeed * cfg_.attackSpeed)
        / (2.0f * cfg_.accelPath);

    droneCtx_.direction = cfg_.initialDir;
    droneCtx_.desiredDir = cfg_.initialDir;
    droneCtx_.targetDir = cfg_.initialDir;
    droneCtx_.turnRemaining = 0.0f;
    droneCtx_.cfg = &cfg_;

    droneState_ =
        std::make_unique<StateStopped>();

    currentIdx_ =
        -1;

    currentTime_ =
        0.0f;

    processedTargets_.assign(
        targets_->getTargetCount(),
        false
    );

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

int MissionProcessor::selectTargetByMinArrivalTime()
{
    float bestTime =
        std::numeric_limits<float>::max();

    int bestIndex =
        -1;

    for (int i = 0;
         i < targets_->getTargetCount();
         ++i)
    {
        if (processedTargets_[i])
        {
            continue;
        }

        const TargetState target =
            targets_->getTarget(
                i,
                currentTime_,
                cfg_.simTimeStep
            );

        DropPoint preliminary =
            solver_->solve(
                droneCtx_.position,
                cfg_.altitude,
                target.pos,
                cfg_.attackSpeed,
                cfg_.accelPath,
                ammo_
            );

        if (!preliminary.valid)
        {
            continue;
        }

        const Coord predictedTarget =
            target.pos
            + target.vel
              * preliminary.flightTime;

        DropPoint result =
            solver_->solve(
                droneCtx_.position,
                cfg_.altitude,
                predictedTarget,
                cfg_.attackSpeed,
                cfg_.accelPath,
                ammo_
            );

        if (!result.valid)
        {
            continue;
        }

        const float arrivalTime =
            estimateArrivalTime(
                result.dropPoint
            );

        if (arrivalTime < bestTime)
        {
            bestTime =
                arrivalTime;

            bestIndex =
                i;
        }
    }

    return bestIndex;
}

float MissionProcessor::estimateArrivalTime(
    const Coord& dropPoint
) const
{
    const Coord toDrop =
        dropPoint
        - droneCtx_.position;

    const float distance =
        length(toDrop);

    const float desiredDir =
        std::atan2(
            toDrop.y,
            toDrop.x
        );

    const float angleDelta =
        normalizeAngle(
            desiredDir
            - droneCtx_.direction
        );

    const float turnTime =
        std::fabs(angleDelta)
        / cfg_.angularSpeed;

    const float moveTime =
        distance
        / cfg_.attackSpeed;

    return turnTime + moveTime;
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

    step.state =
        droneState_->code();

    step.targetIdx =
        result.targetIndex;

    step.dropPoint =
        result.dropPoint;

    step.aimPoint =
        droneCtx_.position
        + dirVec * result.horizontalRange;

    step.predictedTarget =
        result.predictedTarget;

    steps_.push_back(step);
}