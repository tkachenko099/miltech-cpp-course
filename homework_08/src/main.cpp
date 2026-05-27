#include "MissionProcessor.hpp"

#include "Types.hpp"

#include "config/ComponentFactory.hpp"

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

#include <exception>
#include <iostream>

int main()
{
    IConfigLoader* loader = nullptr;
    ITargetProvider* provider = nullptr;
    IBallisticSolver* solver = nullptr;

    try
    {
        loader = createLoader(
            LoaderType::File
        );

        provider = createProvider(
            ProviderType::Json,
            "data/targets.json"
        );

        solver = createSolver(
            SolverType::Analytical
        );

        MissionProcessor mission{
            provider,
            solver,
            loader
        };

        mission.init(
            "data/config.json",
            "data/ammo.json"
        );

        while (mission.hasNext())
        {
            const DropPoint result =
                mission.step();

            if (!result.valid)
            {
                std::cout
                    << "target "
                    << result.targetIndex
                    << ": no valid solution\n";

                continue;
            }

            std::cout
                << "target "
                << result.targetIndex
                << ": drop=("
                << result.dropPoint.x
                << ", "
                << result.dropPoint.y
                << ") "
                << "flight_time="
                << result.flightTime
                << " "
                << "range="
                << result.horizontalRange
                << '\n';
        }

        delete solver;
        delete provider;
        delete loader;

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr
            << "error: "
            << ex.what()
            << '\n';

        delete solver;
        delete provider;
        delete loader;

        return 1;
    }
}