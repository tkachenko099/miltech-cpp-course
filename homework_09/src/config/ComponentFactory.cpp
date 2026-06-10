#include "config/ComponentFactory.hpp"

#include "config/FileConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "solvers/TableSolver.hpp"

#include <memory>
#include <stdexcept>

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type,
    const std::string& param
)
{
    switch (type)
    {
        case SolverType::Analytical:
            return std::make_unique<AnalyticalSolver>();

        case SolverType::Table:
            return std::make_unique<TableSolver>(param);
    }

    throw std::runtime_error("unknown solver type");
}

std::unique_ptr<ITargetProvider> createProvider(
    ProviderType type,
    const std::string& param
)
{
    switch (type)
    {
        case ProviderType::Json:
            return std::make_unique<JsonTargetProvider>(param);
    }

    throw std::runtime_error("unknown provider type");
}

std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type
)
{
    switch (type)
    {
        case LoaderType::File:
            return std::make_unique<FileConfigLoader>();
    }

    throw std::runtime_error("unknown loader type");
}