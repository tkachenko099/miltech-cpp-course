#include "MissionProcessor.hpp"

#include <cmath>
#include <stdexcept>

MissionProcessor::MissionProcessor(
    ITargetProvider& targets,
    IBallisticSolver& solver,
    IConfigLoader& loader
)
    : targets_{targets},
      solver_{&solver},
      loader_{loader}
{
}

void MissionProcessor::init(
    const std::string& configPath,
    const std::string& ammoPath
)
{
    loader_.load(
        configPath,
        ammoPath
    );

    cfg_ =
        loader_.getConfig();

    ammo_ =
        loader_.getAmmoParams();

    drone_ =
        initDrone(cfg_);

    currentIdx_ = 0;
    currentTime_ = 0.0f;

    steps_.clear();
}

bool MissionProcessor::hasNext() const
{
    return currentIdx_
        < targets_.getTargetCount();
}

DropPoint MissionProcessor::step()
{
    if (!hasNext())
    {
        throw std::runtime_error(
            "no more targets"
        );
    }

    TargetState target =
        targets_.getTarget(
            currentIdx_,
            currentTime_,
            cfg_.simTimeStep
        );

    DropPoint result =
        solver_->solve(
            drone_.pos,
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
            result.dropPoint - drone_.pos;

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
                drone_.pos,
                result.dropPoint
            );

        saveStep(result);
        updateDrone(result);

        drone_.currentTargetIndex =
            currentIdx_;
    }

    ++currentIdx_;
    currentTime_ += cfg_.simTimeStep;

    return result;
}

void MissionProcessor::reset()
{
    drone_ =
        initDrone(cfg_);

    currentIdx_ = 0;
    currentTime_ = 0.0f;

    steps_.clear();
}

void MissionProcessor::changeSolver(
    IBallisticSolver& solver
)
{
    solver_ = &solver;
}

const std::vector<SimStep>&
MissionProcessor::getSteps() const
{
    return steps_;
}

Drone MissionProcessor::initDrone(
    const DroneConfig& cfg
)
{
    Drone drone{};

    drone.pos =
        cfg.startPos;

    drone.z =
        cfg.altitude;

    drone.dir =
        cfg.initialDir;

    drone.speed =
        0.0f;

    drone.attackSpeed =
        cfg.attackSpeed;

    drone.acceleration =
        (cfg.attackSpeed * cfg.attackSpeed)
        / (2.0f * cfg.accelPath);

    drone.state =
        DroneState::Stopped;

    drone.currentTargetIndex =
        -1;

    return drone;
}

float MissionProcessor::normalizeAngle(
    float angle
)
{
    constexpr float pi =
        3.14159265358979323846f;

    while (angle > pi)
    {
        angle -= 2.0f * pi;
    }

    while (angle < -pi)
    {
        angle += 2.0f * pi;
    }

    return angle;
}

void MissionProcessor::updateDrone(
    const DropPoint& result
)
{
    const float angleDiff =
        normalizeAngle(
            result.desiredDir - drone_.dir
        );

    const float dt =
        cfg_.simTimeStep;

    if (std::fabs(angleDiff)
        > cfg_.turnThreshold)
    {
        if (drone_.speed > 0.0f)
        {
            drone_.state =
                DroneState::Decelerating;

            drone_.speed -=
                drone_.acceleration * dt;

            if (drone_.speed < 0.0f)
            {
                drone_.speed = 0.0f;
            }
        }
        else
        {
            drone_.state =
                DroneState::Turning;

            const float maxTurn =
                cfg_.angularSpeed * dt;

            if (std::fabs(angleDiff)
                <= maxTurn)
            {
                drone_.dir =
                    result.desiredDir;

                drone_.state =
                    DroneState::Accelerating;
            }
            else
            {
                drone_.dir +=
                    angleDiff > 0.0f
                        ? maxTurn
                        : -maxTurn;

                drone_.dir =
                    normalizeAngle(
                        drone_.dir
                    );
            }
        }
    }
    else
    {
        drone_.dir =
            result.desiredDir;

        if (drone_.speed
            < drone_.attackSpeed)
        {
            drone_.state =
                DroneState::Accelerating;

            drone_.speed +=
                drone_.acceleration * dt;

            if (drone_.speed
                > drone_.attackSpeed)
            {
                drone_.speed =
                    drone_.attackSpeed;
            }
        }
        else
        {
            drone_.state =
                DroneState::Moving;
        }
    }

    const Coord dirVec{
        std::cos(drone_.dir),
        std::sin(drone_.dir)
    };

    drone_.pos +=
        dirVec
        * (drone_.speed * dt);
}

void MissionProcessor::saveStep(
    const DropPoint& result
)
{
    const Coord dirVec{
        std::cos(drone_.dir),
        std::sin(drone_.dir)
    };

    SimStep step{};

    step.pos =
        drone_.pos;

    step.direction =
        drone_.dir;

    step.state =
        drone_.state;

    step.targetIdx =
        result.targetIndex;

    step.dropPoint =
        result.dropPoint;

    step.aimPoint =
        drone_.pos
        + dirVec
          * result.horizontalRange;

    step.predictedTarget =
        result.predictedTarget;

    steps_.push_back(step);
}