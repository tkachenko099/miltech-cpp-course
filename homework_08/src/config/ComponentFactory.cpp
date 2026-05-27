#include "config/ComponentFactory.hpp"

#include "config/FileConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "solvers/AnalyticalSolver.hpp"

#include <stdexcept>

IBallisticSolver* createSolver(
    SolverType type
)
{
    switch (type)
    {
        case SolverType::Analytical:
            return new AnalyticalSolver{};
    }

    throw std::runtime_error(
        "unknown solver type"
    );
}

ITargetProvider* createProvider(
    ProviderType type,
    const std::string& param
)
{
    switch (type)
    {
        case ProviderType::Json:
            return new JsonTargetProvider{
                param
            };
    }

    throw std::runtime_error(
        "unknown provider type"
    );
}

IConfigLoader* createLoader(
    LoaderType type
)
{
    switch (type)
    {
        case LoaderType::File:
            return new FileConfigLoader{};
    }

    throw std::runtime_error(
        "unknown loader type"
    );
}