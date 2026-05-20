export module simulation_writer;

import mission_types;

import <fstream>;
import <vector>;

#include "json.hpp"

using json = nlohmann::json;

export void writeSimulationJson(
    const char* filename,
    const std::vector<SimStep>& steps
)
{
    json out;
    out["totalSteps"] = steps.size();
    out["steps"] = json::array();

    for (const auto& step : steps)
    {
        json s;
        s["position"] = {
            {"x", step.pos.x},
            {"y", step.pos.y}
        };
        s["direction"] = step.direction;
        s["state"] = static_cast<int>(step.state);
        s["targetIndex"] = step.targetIdx;
        s["dropPoint"] = {
            {"x", step.dropPoint.x},
            {"y", step.dropPoint.y}
        };
        s["aimPoint"] = {
            {"x", step.aimPoint.x},
            {"y", step.aimPoint.y}
        };
        s["predictedTarget"] = {
            {"x", step.predictedTarget.x},
            {"y", step.predictedTarget.y}
        };

        out["steps"].push_back(s);
    }

    std::ofstream fout(filename);
    fout << out.dump(2);
}