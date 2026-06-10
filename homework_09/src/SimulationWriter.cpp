#include "SimulationWriter.hpp"

#include "json.hpp"

#include <fstream>
#include <stdexcept>

void SimulationWriter::writeJson(
    const std::string& filename,
    const std::vector<SimStep>& steps
)
{
    nlohmann::json out;

    out["total_steps"] = steps.size();
    out["steps"] = nlohmann::json::array();

    for (const auto& step : steps)
    {
        nlohmann::json item;

        item["target"] =
            step.targetIdx;

        item["drop"] = {
            {"x", step.dropPoint.x},
            {"y", step.dropPoint.y}
        };

        item["flight_time"] =
            step.flightTime;

        item["range"] =
            step.horizontalRange;

        item["state"] =
            step.stateName;

        item["drone"] = {
            {"x", step.pos.x},
            {"y", step.pos.y},
            {"direction", step.direction}
        };

        item["aim_point"] = {
            {"x", step.aimPoint.x},
            {"y", step.aimPoint.y}
        };

        item["predicted_target"] = {
            {"x", step.predictedTarget.x},
            {"y", step.predictedTarget.y}
        };

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