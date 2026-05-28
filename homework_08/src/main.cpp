#include "MissionProcessor.hpp"
#include "Types.hpp"
#include "config/ComponentFactory.hpp"

#include <exception>
#include <iostream>

int main()
{
    try
    {
        auto loader =
            createLoader(
                LoaderType::File
            );

        auto provider =
            createProvider(
                ProviderType::Json,
                "data/targets.json"
            );

        auto solver =
            createSolver(
                SolverType::Analytical
            );

        MissionProcessor mission{
            *provider,
            *solver,
            *loader
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

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr
            << "error: "
            << ex.what()
            << '\n';

        return 1;
    }
}