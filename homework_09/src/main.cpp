#include "MissionProcessor.hpp"
#include "SimulationWriter.hpp"
#include "config/ComponentFactory.hpp"

#include <exception>
#include <iostream>
#include <memory>

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
                SolverType::Table,
                "data/ballistic_table.txt"
            );

        MissionProcessor mission{
            std::move(provider),
            std::move(solver),
            std::move(loader)
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

            if (!mission.getSteps().empty())
            {
                const SimStep& lastStep =
                    mission.getSteps().back();

                std::cout
                    << "target "
                    << result.targetIndex
                    << ": drone=("
                    << lastStep.pos.x
                    << ", "
                    << lastStep.pos.y
                    << ") "
                    << "state="
                    << lastStep.state
                    << " "
                    << "drop=("
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
        }

        SimulationWriter::writeJson(
            "simulation.json",
            mission.getSteps()
        );

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