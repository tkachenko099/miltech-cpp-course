export module factory;

import interfaces;
import json_target_provider;
import file_config_loader;
import analytical_solver;

export enum class SolverType   { Analytical };
export enum class ProviderType { Json };
export enum class LoaderType   { File };

export IBallisticSolver* createSolver(SolverType type) {
    switch (type) {
        case SolverType::Analytical:
            return new AnalyticalSolver{};
    }
    return nullptr;
}

export ITargetProvider* createProvider(ProviderType type, const char* path) {
    switch (type) {
        case ProviderType::Json:
            return new JsonTargetProvider{path};
    }
    return nullptr;
}

export IConfigLoader* createLoader(LoaderType type) {
    switch (type) {
        case LoaderType::File:
            return new FileConfigLoader{};
    }
    return nullptr;
}