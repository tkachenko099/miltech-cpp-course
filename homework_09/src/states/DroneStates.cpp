#define _USE_MATH_DEFINES

#include "states/DroneStates.hpp"

#include <cmath>
#include <memory>

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

void moveForward(DroneContext& ctx)
{
    const float dt = ctx.cfg->simTimeStep;

    const Coord dirVec{
        std::cos(ctx.direction),
        std::sin(ctx.direction)
    };

    ctx.position += dirVec * (ctx.speed * dt);
}

void steerToDesiredDirection(DroneContext& ctx)
{
    const float delta =
        normalizeAngle(
            ctx.desiredDir - ctx.direction
        );

    const float maxTurn =
        ctx.cfg->angularSpeed
        * ctx.cfg->simTimeStep;

    if (std::fabs(delta) <= maxTurn)
    {
        ctx.direction = ctx.desiredDir;
    }
    else
    {
        ctx.direction +=
            delta > 0.0f
                ? maxTurn
                : -maxTurn;

        ctx.direction =
            normalizeAngle(ctx.direction);
    }
}
}

std::unique_ptr<IDroneState>
StateStopped::execute(DroneContext& ctx)
{
    const float delta =
        normalizeAngle(
            ctx.desiredDir - ctx.direction
        );

    if (std::fabs(delta) > ctx.cfg->turnThreshold)
    {
        ctx.targetDir = ctx.desiredDir;

        return std::make_unique<StateTurning>();
    }

    ctx.direction = ctx.desiredDir;

    return std::make_unique<StateAccelerating>();
}

const char* StateStopped::name() const
{
    return "Stopped";
}

std::unique_ptr<IDroneState>
StateAccelerating::execute(DroneContext& ctx)
{
    const float dt = ctx.cfg->simTimeStep;

    steerToDesiredDirection(ctx);

    ctx.speed += ctx.acceleration * dt;

    if (ctx.speed > ctx.cfg->attackSpeed)
    {
        ctx.speed = ctx.cfg->attackSpeed;
    }

    moveForward(ctx);

    if (ctx.speed >= ctx.cfg->attackSpeed)
    {
        return std::make_unique<StateMoving>();
    }

    return nullptr;
}

const char* StateAccelerating::name() const
{
    return "Accelerating";
}

std::unique_ptr<IDroneState>
StateMoving::execute(DroneContext& ctx)
{
    steerToDesiredDirection(ctx);

    moveForward(ctx);

    return nullptr;
}

const char* StateMoving::name() const
{
    return "Moving";
}

std::unique_ptr<IDroneState>
StateDecelerating::execute(DroneContext& ctx)
{
    const float dt = ctx.cfg->simTimeStep;

    steerToDesiredDirection(ctx);

    ctx.speed -= ctx.acceleration * dt;

    if (ctx.speed < 0.0f)
    {
        ctx.speed = 0.0f;
    }

    moveForward(ctx);

    if (ctx.speed <= 0.0f)
    {
        return std::make_unique<StateStopped>();
    }

    return nullptr;
}

const char* StateDecelerating::name() const
{
    return "Decelerating";
}

std::unique_ptr<IDroneState>
StateTurning::execute(DroneContext& ctx)
{
    const float delta =
        normalizeAngle(
            ctx.targetDir - ctx.direction
        );

    const float maxTurn =
        ctx.cfg->angularSpeed
        * ctx.cfg->simTimeStep;

    if (std::fabs(delta) <= maxTurn)
    {
        ctx.direction = ctx.targetDir;

        return std::make_unique<StateAccelerating>();
    }

    ctx.direction +=
        delta > 0.0f
            ? maxTurn
            : -maxTurn;

    ctx.direction =
        normalizeAngle(ctx.direction);

    return nullptr;
}

const char* StateTurning::name() const
{
    return "Turning";
}

int StateStopped::code() const
{
    return 0;
}

int StateAccelerating::code() const
{
    return 1;
}

int StateDecelerating::code() const
{
    return 2;
}

int StateTurning::code() const
{
    return 3;
}

int StateMoving::code() const
{
    return 4;
}