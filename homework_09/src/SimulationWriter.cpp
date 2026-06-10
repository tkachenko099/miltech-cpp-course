#include "SimulationWriter.hpp"

#include "json.hpp"

#include <fstream>
#include <stdexcept>

namespace
{
nlohmann::json coordToJson(
    const Coord& coord
)
{
    return {
        {"x", coord.x},
        {"y", coord.y}
    };
}
}

void SimulationWriter::writeJson(
    const std::string& filename,
    const std::vector<SimStep>& steps
)
{
    nlohmann::json out;

    out["totalSteps"] =
        steps.size();

    out["steps"] =
        nlohmann::json::array();

    for (const auto& step : steps)
    {
        nlohmann::json item;

        item["position"] =
            coordToJson(step.pos);

        item["direction"] =
            step.direction;

        item["state"] =
            step.state;

        item["targetIndex"] =
            step.targetIdx;

        item["dropPoint"] =
            coordToJson(step.dropPoint);

        item["aimPoint"] =
            coordToJson(step.aimPoint);

        item["predictedTarget"] =
            coordToJson(step.predictedTarget);

        out["steps"].push_back(item);
    }

    std::ofstream fout(filename);

    if (!fout.is_open())
    {
        throw std::runtime_error(
            "cannot open output file: " + filename
        );
    }

    fout << out.dump(2);
}