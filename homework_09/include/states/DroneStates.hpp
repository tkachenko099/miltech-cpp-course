#pragma once

#include "state/IDroneState.hpp"

class StateStopped final : public IDroneState
{
public:
    std::unique_ptr<IDroneState>
    execute(DroneContext& ctx) override;

    const char* name() const override;
};

class StateAccelerating final : public IDroneState
{
public:
    std::unique_ptr<IDroneState>
    execute(DroneContext& ctx) override;

    const char* name() const override;
};

class StateDecelerating final : public IDroneState
{
public:
    std::unique_ptr<IDroneState>
    execute(DroneContext& ctx) override;

    const char* name() const override;
};

class StateTurning final : public IDroneState
{
public:
    std::unique_ptr<IDroneState>
    execute(DroneContext& ctx) override;

    const char* name() const override;
};

class StateMoving final : public IDroneState
{
public:
    std::unique_ptr<IDroneState>
    execute(DroneContext& ctx) override;

    const char* name() const override;
};