import factory;
import mission_processor;
import simulation_writer;

import <exception>;
import <iostream>;

int main()
{
    IConfigLoader* loader = nullptr;
    ITargetProvider* provider = nullptr;
    IBallisticSolver* solver = nullptr;

    try
    {
        loader = createLoader(LoaderType::File);
        provider = createProvider(ProviderType::Json, "targets.json");
        solver = createSolver(SolverType::Analytical);

        MissionProcessor mission{
            provider,
            solver,
            loader
        };

        mission.init(
            "config.json",
            "ammo.json"
        );

        while (mission.hasNext())
        {
            const DropPoint result = mission.step();

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
                << ")\n";
        }

        writeSimulationJson(
            "simulation.json",
            mission.getSteps()
        );

        delete solver;
        delete provider;
        delete loader;

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << '\n';

        delete solver;
        delete provider;
        delete loader;

        return 1;
    }
}