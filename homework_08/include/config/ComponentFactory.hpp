#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

#include <memory>
#include <string>

enum class SolverType
{
    Analytical
};

enum class ProviderType
{
    Json
};

enum class LoaderType
{
    File
};

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type
);

std::unique_ptr<ITargetProvider> createProvider(
    ProviderType type,
    const std::string& param
);

std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type
);