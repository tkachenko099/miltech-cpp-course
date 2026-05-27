#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"

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

IBallisticSolver* createSolver(
    SolverType type
);

ITargetProvider* createProvider(
    ProviderType type,
    const std::string& param
);

IConfigLoader* createLoader(
    LoaderType type
);