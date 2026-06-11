module;

#include <cmath>
#include <vector>

export module mission_processor;

import interfaces;
import mission_types;

export class MissionProcessor
{
private:

    ITargetProvider*
        targets_{};

    IBallisticSolver*
        solver_{};

    IConfigLoader*
        loader_{};

    Drone drone_{};

    DroneConfig cfg_{};

    AmmoParams ammo_{};

    int currentIdx_{};

    float currentTime_{};

    std::vector<SimStep>
        steps_;

public:

    MissionProcessor(
        ITargetProvider* targets,
        IBallisticSolver* solver,
        IConfigLoader* loader
    )
        :
        targets_(targets),
        solver_(solver),
        loader_(loader)
    {
    }

    void init(
        const char* configPath,
        const char* ammoPath
    )
    {
        loader_->load(
            configPath,
            ammoPath
        );

        cfg_=
            loader_->getConfig();

        ammo_=
            loader_->getAmmoParams();

        currentIdx_=0;
        currentTime_=0;
        steps_.clear();
    }

    bool hasNext() const
    {
        return
            currentIdx_
            <
            targets_->getTargetCount();
    }

    DropPoint step()
    {
        TargetState target=
            targets_->getTarget(
                currentIdx_,
                currentTime_,
                cfg_.simTimeStep
            );

        DropPoint result=
            solver_->solve(
                drone_.pos,
                cfg_.altitude,
                target.pos,
                cfg_.attackSpeed,
                cfg_.accelPath,
                ammo_
            );

        ++currentIdx_;

        return result;
    }

    void reset()
    {
        currentIdx_=0;
        currentTime_=0;
    }

    void changeSolver(
        IBallisticSolver* solver
    )
    {
        solver_=solver;
    }
};