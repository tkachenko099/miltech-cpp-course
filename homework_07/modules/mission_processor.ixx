export module mission_processor;

import interfaces;
import mission_types;

import <cmath>;
import <stdexcept>;
import <vector>;

export class MissionProcessor
{
public:
    MissionProcessor(
        ITargetProvider* targets,
        IBallisticSolver* solver,
        IConfigLoader* loader
    )
        : targets_{targets},
          solver_{solver},
          loader_{loader}
    {
        if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr)
        {
            throw std::runtime_error("MissionProcessor dependency is null");
        }
    }

    void init(
        const char* configPath,
        const char* ammoPath
    )
    {
        loader_->load(configPath, ammoPath);
        cfg_ = loader_->getConfig();
        ammo_ = loader_->getAmmoParams();
        drone_ = initDrone(cfg_);
        currentIdx_ = 0;
        currentTime_ = 0.0f;
        steps_.clear();
    }

    bool hasNext() const
    {
        return currentIdx_ < targets_->getTargetCount();
    }

    DropPoint step()
    {
        if (!hasNext())
        {
            throw std::runtime_error("no more targets");
        }

        TargetState target = targets_->getTarget(
            currentIdx_,
            currentTime_,
            cfg_.simTimeStep
        );

        DropPoint result = solver_->solve(
            drone_.pos,
            drone_.z,
            target.pos,
            drone_.attackSpeed,
            cfg_.accelPath,
            ammo_
        );

        result.targetIndex = currentIdx_;

        if (!result.valid)
        {
};