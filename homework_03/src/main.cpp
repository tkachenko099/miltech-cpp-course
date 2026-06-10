#define _USE_MATH_DEFINES
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

#define ENABLE_LOG   1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

constexpr int   MAX_STEPS = 10000;
constexpr float G         = 9.81f;
constexpr float EPS       = 1e-6f;

enum DroneState
{
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4
};

struct Coord
{
    float x{};
    float y{};

    Coord operator+(const Coord& other) const
    {
        return {x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const
    {
        return {x - other.x, y - other.y};
    }

    Coord operator*(float s) const
    {
        return {x * s, y * s};
    }

    Coord operator/(float s) const
    {
        return {x / s, y / s};
    }

    bool operator==(const Coord& other) const
    {
        return std::fabs(x - other.x) < EPS &&
               std::fabs(y - other.y) < EPS;
    }

    Coord& operator+=(const Coord& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Coord& operator-=(const Coord& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Coord& operator*=(float s)
    {
        x *= s;
        y *= s;
        return *this;
    }

    Coord& operator/=(float s)
    {
        x /= s;
        y /= s;
        return *this;
    }
};

inline Coord operator*(float s, const Coord& c)
{
    return {s * c.x, s * c.y};
}

inline float length(Coord c)
{
    return std::hypot(c.x, c.y);
}

inline Coord normalize(Coord c)
{
    float len = length(c);
    if (len < EPS)
    {
        return {0.0f, 0.0f};
    }
    return c / len;
}

inline float distance2D(Coord a, Coord b)
{
    return length(a - b);
}

struct AmmoParams
{
    char name[32]{};
    float mass{};
    float drag{};
    float lift{};
};

struct DroneConfig
{
    Coord startPos{};
    float altitude{};
    float initialDir{};
    float attackSpeed{};
    float accelPath{};
    char  ammoName[32]{};
    float arrayTimeStep{};
    float simTimeStep{};
    float hitRadius{};
    float angularSpeed{};
    float turnThreshold{};
};

struct SimStep
{
    Coord pos{};
    float direction{};
    int   state{};
    int   targetIdx{};
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
};

struct Drone
{
    Coord pos{};
    float z{};
    float dir{};
    float speed{};
    float attackSpeed{};
    float acceleration{};
    DroneState state{STOPPED};
    int currentTargetIndex{-1};
};

struct TargetState
{
    Coord pos{};
    Coord vel{};
};

struct Solution
{
    bool  valid{false};
    int   targetIndex{-1};
    float totalCost{};
    Coord dropPoint{};
    Coord predictedTarget{};
    float desiredDir{};
    float flightTime{};
    float horizontalRange{};
};

float normalizeAngle(float a)
{
    while (a > M_PI)  a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

bool readConfigJson(const char* filename, DroneConfig& cfg)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        return false;
    }

    json j;
    fin >> j;

    cfg.startPos.x    = j["drone"]["position"]["x"];
    cfg.startPos.y    = j["drone"]["position"]["y"];
    cfg.altitude      = j["drone"]["altitude"];
    cfg.initialDir    = j["drone"]["initialDirection"];
    cfg.attackSpeed   = j["drone"]["attackSpeed"];
    cfg.accelPath     = j["drone"]["accelerationPath"];
    cfg.angularSpeed  = j["drone"]["angularSpeed"];
    cfg.turnThreshold = j["drone"]["turnThreshold"];

    std::string ammoName = j["ammo"].get<std::string>();
    std::strncpy(cfg.ammoName, ammoName.c_str(), 31);
    cfg.ammoName[31] = '\0';

    cfg.simTimeStep   = j["simulation"]["timeStep"];
    cfg.hitRadius     = j["simulation"]["hitRadius"];
    cfg.arrayTimeStep = j["targetArrayTimeStep"];

    return true;
}

bool readAmmoJson(const char* filename, AmmoParams*& ammo, int& ammoCount)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        return false;
    }

    json j;
    fin >> j;

    ammoCount = static_cast<int>(j.size());
    ammo = new AmmoParams[ammoCount];

    for (int i = 0; i < ammoCount; ++i)
    {
        std::string name = j[i]["name"].get<std::string>();
        std::strncpy(ammo[i].name, name.c_str(), 31);
        ammo[i].name[31] = '\0';

        ammo[i].mass = j[i]["mass"];
        ammo[i].drag = j[i]["drag"];
        ammo[i].lift = j[i]["lift"];
    }

    return true;
}

bool findAmmo(const char* ammoName, AmmoParams* ammo, int ammoCount, AmmoParams& result)
{
    for (int i = 0; i < ammoCount; ++i)
    {
        if (std::strcmp(ammoName, ammo[i].name) == 0)
        {
            result = ammo[i];
            return true;
        }
    }
    return false;
}

bool readTargetsJson(const char* filename, Coord**& targets, int& targetCount, int& timeSteps)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        return false;
    }

    json j;
    fin >> j;

    targetCount = j["targetCount"];
    timeSteps   = j["timeSteps"];

    targets = new Coord*[targetCount];
    for (int i = 0; i < targetCount; ++i)
    {
        targets[i] = new Coord[timeSteps];
        for (int k = 0; k < timeSteps; ++k)
        {
            targets[i][k].x = j["targets"][i]["positions"][k]["x"];
            targets[i][k].y = j["targets"][i]["positions"][k]["y"];
        }
    }

    return true;
}

void freeTargets(Coord**& targets, int targetCount)
{
    if (!targets)
    {
        return;
    }

    for (int i = 0; i < targetCount; ++i)
    {
        delete[] targets[i];
        targets[i] = nullptr;
    }

    delete[] targets;
    targets = nullptr;
}

Drone initDrone(const DroneConfig& cfg)
{
    Drone drone{};
    drone.pos          = cfg.startPos;
    drone.z            = cfg.altitude;
    drone.dir          = cfg.initialDir;
    drone.speed        = 0.0f;
    drone.attackSpeed  = cfg.attackSpeed;
    drone.acceleration = (cfg.attackSpeed * cfg.attackSpeed) / (2.0f * cfg.accelPath);
    drone.state        = STOPPED;
    drone.currentTargetIndex = -1;
    return drone;
}

Coord interpolateTarget(Coord** targets,
                        int targetIndex,
                        float t,
                        float arrayTimeStep,
                        int timeSteps)
{
    int idx = static_cast<int>(std::floor(t / arrayTimeStep)) % timeSteps;
    if (idx < 0)
    {
        idx += timeSteps;
    }

    int next = (idx + 1) % timeSteps;

    float baseTime = std::floor(t / arrayTimeStep) * arrayTimeStep;
    float frac = (t - baseTime) / arrayTimeStep;

    Coord a = targets[targetIndex][idx];
    Coord b = targets[targetIndex][next];

    return {
        a.x + (b.x - a.x) * frac,
        a.y + (b.y - a.y) * frac
    };
}

TargetState getTargetState(Coord** targets,
                           int targetIndex,
                           float t,
                           float dt,
                           float arrayTimeStep,
                           int timeSteps)
{
    TargetState s{};

    Coord p1 = interpolateTarget(targets, targetIndex, t,      arrayTimeStep, timeSteps);
    Coord p2 = interpolateTarget(targets, targetIndex, t + dt, arrayTimeStep, timeSteps);

    s.pos = p1;
    s.vel = (p2 - p1) / dt;
    return s;
}

bool computeDropPoint(float droneX,
                      float droneY,
                      float droneZ,
                      float targetX,
                      float targetY,
                      float attackSpeed,
                      float accelerationPath,
                      const AmmoParams& ammo,
                      float& dropX,
                      float& dropY,
                      float& flightTime,
                      float& horizontalRange)
{
    const float a = ammo.drag * G * ammo.mass
                  - 2.0f * ammo.drag * ammo.drag * ammo.lift * attackSpeed;

    const float b = -3.0f * G * ammo.mass * ammo.mass
                  + 3.0f * ammo.drag * ammo.lift * ammo.mass * attackSpeed;

    const float c = 6.0f * ammo.mass * ammo.mass * droneZ;

    if (std::fabs(a) < EPS)
    {
        return false;
    }

    const float p = -(b * b) / (3.0f * a * a);
    const float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;

    if (p >= 0.0f)
    {
        return false;
    }

    float acosArg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
    acosArg = std::clamp(acosArg, -1.0f, 1.0f);

    const float phi = std::acos(acosArg);
    const float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * M_PI) / 3.0f)
                  - b / (3.0f * a);

    const float h =
          attackSpeed * t
        - std::pow(t, 2.0f) * ammo.drag * attackSpeed / (2.0f * ammo.mass)
        + std::pow(t, 3.0f) *
          (
              6.0f * ammo.drag * G * ammo.lift * ammo.mass
            - 6.0f * std::pow(ammo.drag, 2.0f) * (std::pow(ammo.lift, 2.0f) - 1.0f) * attackSpeed
          ) / (36.0f * std::pow(ammo.mass, 2.0f))
        + std::pow(t, 4.0f) *
          (
              -6.0f * std::pow(ammo.drag, 2.0f) * G * ammo.lift *
              (1.0f + std::pow(ammo.lift, 2.0f) + std::pow(ammo.lift, 4.0f)) * ammo.mass
            + 3.0f * std::pow(ammo.drag, 3.0f) * std::pow(ammo.lift, 2.0f) *
              (1.0f + std::pow(ammo.lift, 2.0f)) * attackSpeed
            + 6.0f * std::pow(ammo.drag, 3.0f) * std::pow(ammo.lift, 4.0f) *
              (1.0f + std::pow(ammo.lift, 2.0f) * attackSpeed)
          ) / (36.0f * std::pow(1.0f + std::pow(ammo.lift, 2.0f), 2.0f) * std::pow(ammo.mass, 3.0f))
        + std::pow(t, 5.0f) *
          (
              3.0f * std::pow(ammo.drag, 3.0f) * G * std::pow(ammo.lift, 3.0f) * ammo.mass
            - 3.0f * std::pow(ammo.drag, 4.0f) * std::pow(ammo.lift, 2.0f) *
              (1.0f + std::pow(ammo.lift, 2.0f)) * attackSpeed
          ) / (36.0f * (1.0f + std::pow(ammo.lift, 2.0f)) * std::pow(ammo.mass, 4.0f));

    Coord dronePos{droneX, droneY};
    Coord targetPos{targetX, targetY};
    Coord delta = targetPos - dronePos;

    float D = length(delta);
    if (D < EPS)
    {
        return false;
    }

    float ratio = (D - h) / D;

    if (h + accelerationPath > D)
    {
        Coord maneuverPoint = targetPos - delta * ((h + accelerationPath) / D);
        Coord firePoint = maneuverPoint + (targetPos - maneuverPoint) * ratio;
        dropX = firePoint.x;
        dropY = firePoint.y;
    }
    else
    {
        Coord firePoint = dronePos + delta * ratio;
        dropX = firePoint.x;
        dropY = firePoint.y;
    }

    flightTime = t;
    horizontalRange = h;
    return true;
}

Solution estimateIntercept(const Drone& drone,
                           const AmmoParams& ammo,
                           Coord** targets,
                           int timeSteps,
                           int targetIndex,
                           float currentTime,
                           const DroneConfig& cfg)
{
    Solution sol{};
    sol.targetIndex = targetIndex;

    constexpr float w1 = 0.7f;
    constexpr float w2 = 0.3f;
    constexpr float switchPenalty = 1.5f;

    TargetState ts = getTargetState(targets,
                                    targetIndex,
                                    currentTime,
                                    cfg.simTimeStep,
                                    cfg.arrayTimeStep,
                                    timeSteps);

    float totalCost = 0.0f;
    Coord bestPredicted{};
    Coord bestDrop{};
    float bestDesiredDir = 0.0f;
    float bestFlightTime = 0.0f;
    float bestHorizontalRange = 0.0f;
    bool firstValid = false;

    for (int horizon = 1; horizon <= 2; ++horizon)
    {
        float lookAheadTime = horizon * cfg.simTimeStep;
        Coord predicted = ts.pos + ts.vel * lookAheadTime;

        float dropX = 0.0f;
        float dropY = 0.0f;
        float flightTime = 0.0f;
        float horizontalRange = 0.0f;

        if (!computeDropPoint(drone.pos.x, drone.pos.y, drone.z,
                              predicted.x, predicted.y,
                              drone.attackSpeed,
                              cfg.accelPath,
                              ammo,
                              dropX, dropY,
                              flightTime, horizontalRange))
        {
            return sol;
        }

        Coord dropPoint{dropX, dropY};
        Coord toDrop = dropPoint - drone.pos;

        float desiredDir = std::atan2(toDrop.y, toDrop.x);
        float angularDiff = normalizeAngle(desiredDir - drone.dir);

        float pathTime = length(toDrop) / std::max(drone.attackSpeed, EPS);

        float maneuverTime = 0.0f;
        if (std::fabs(angularDiff) > cfg.turnThreshold)
        {
            float stopTime = drone.speed / std::max(drone.acceleration, EPS);
            float turnTime = std::fabs(angularDiff) / std::max(cfg.angularSpeed, EPS);
            float accelTime = drone.attackSpeed / std::max(drone.acceleration, EPS);
            maneuverTime = stopTime + turnTime + accelTime;
        }

        float localCost = pathTime + maneuverTime;

        if (horizon == 1)
        {
            totalCost += w1 * localCost;
            bestPredicted = predicted;
            bestDrop = dropPoint;
            bestDesiredDir = desiredDir;
            bestFlightTime = flightTime;
            bestHorizontalRange = horizontalRange;
            firstValid = true;
        }
        else
        {
            totalCost += w2 * localCost;
        }
    }

    if (!firstValid)
    {
        return sol;
    }

    if (drone.currentTargetIndex != -1 &&
        drone.currentTargetIndex != targetIndex)
    {
        totalCost += switchPenalty;
    }

    sol.valid = true;
    sol.totalCost = totalCost;
    sol.dropPoint = bestDrop;
    sol.predictedTarget = bestPredicted;
    sol.desiredDir = bestDesiredDir;
    sol.flightTime = bestFlightTime;
    sol.horizontalRange = bestHorizontalRange;
    return sol;
}

Solution chooseBestTarget(const Drone& drone,
                          const AmmoParams& ammo,
                          Coord** targets,
                          int targetCount,
                          int timeSteps,
                          float currentTime,
                          const DroneConfig& cfg)
{
    Solution best{};
    best.valid = false;
    best.totalCost = 1e30f;

    for (int i = 0; i < targetCount; ++i)
    {
        Solution cur = estimateIntercept(drone, ammo, targets, timeSteps, i, currentTime, cfg);
        if (cur.valid && cur.totalCost < best.totalCost)
        {
            best = cur;
        }
    }

    return best;
}

void updateDrone(Drone& drone, const Solution& sol, const DroneConfig& cfg)
{
    float angleDiff = normalizeAngle(sol.desiredDir - drone.dir);
    float dt = cfg.simTimeStep;

    if (std::fabs(angleDiff) > cfg.turnThreshold)
    {
        if (drone.speed > 0.0f)
        {
            drone.state = DECELERATING;
            drone.speed -= drone.acceleration * dt;
            if (drone.speed < 0.0f)
            {
                drone.speed = 0.0f;
            }
        }
        else
        {
            drone.state = TURNING;
            float maxTurn = cfg.angularSpeed * dt;

            if (std::fabs(angleDiff) <= maxTurn)
            {
                drone.dir = sol.desiredDir;
                drone.state = ACCELERATING;
            }
            else
            {
                drone.dir += (angleDiff > 0.0f ? maxTurn : -maxTurn);
                drone.dir = normalizeAngle(drone.dir);
            }
        }
    }
    else
    {
        drone.dir = sol.desiredDir;

        if (drone.speed < drone.attackSpeed)
        {
            drone.state = ACCELERATING;
            drone.speed += drone.acceleration * dt;
            if (drone.speed > drone.attackSpeed)
            {
                drone.speed = drone.attackSpeed;
            }
        }
        else
        {
            drone.state = MOVING;
        }
    }

    Coord dirVec{std::cos(drone.dir), std::sin(drone.dir)};
    drone.pos += dirVec * (drone.speed * dt);
}

bool reachedDropPoint(const Drone& drone, const Solution& sol, float hitRadius)
{
    return distance2D(drone.pos, sol.dropPoint) <= hitRadius;
}

void writeSimulationJson(const char* filename, SimStep* steps, int totalSteps)
{
    json out;
    out["totalSteps"] = totalSteps;
    out["steps"] = json::array();

    for (int i = 0; i < totalSteps; ++i)
    {
        json s;
        s["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
        s["direction"] = steps[i].direction;
        s["state"] = steps[i].state;
        s["targetIndex"] = steps[i].targetIdx;
        s["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
        s["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
        s["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
        out["steps"].push_back(s);
    }

    std::ofstream fout(filename);
    fout << out.dump(2);
}

int main()
{
    DroneConfig cfg{};
    if (!readConfigJson("config.json", cfg))
    {
        std::cerr << "Error: cannot read config.json\n";
        return 1;
    }

    if (cfg.simTimeStep <= 0.0f ||
        cfg.arrayTimeStep <= 0.0f ||
        cfg.accelPath <= 0.0f ||
        cfg.attackSpeed <= 0.0f ||
        cfg.angularSpeed <= 0.0f)
    {
        std::cerr << "Error: invalid config values\n";
        return 1;
    }

    LOG("Config loaded");
    DEBUG("attackSpeed=" << cfg.attackSpeed << ", dt=" << cfg.simTimeStep);

    AmmoParams* ammo = nullptr;
    int ammoCount = 0;
    if (!readAmmoJson("ammo.json", ammo, ammoCount))
    {
        std::cerr << "Error: cannot read ammo.json\n";
        return 1;
    }

    AmmoParams selectedAmmo{};
    if (!findAmmo(cfg.ammoName, ammo, ammoCount, selectedAmmo))
    {
        std::cerr << "Error: ammo not found: " << cfg.ammoName << "\n";
        delete[] ammo;
        ammo = nullptr;
        return 1;
    }
    LOG("Ammo found: " << selectedAmmo.name);

    Coord** targets = nullptr;
    int targetCount = 0;
    int timeSteps = 0;
    if (!readTargetsJson("targets.json", targets, targetCount, timeSteps))
    {
        std::cerr << "Error: cannot read targets.json\n";
        delete[] ammo;
        ammo = nullptr;
        return 1;
    }
    LOG("Targets loaded: targetCount=" << targetCount << ", timeSteps=" << timeSteps);

    Drone drone = initDrone(cfg);

    SimStep* steps = new SimStep[MAX_STEPS];

    float currentTime = 0.0f;
    int step = 0;
    bool dropped = false;

    while (step < MAX_STEPS)
    {
        Solution best = chooseBestTarget(drone, selectedAmmo, targets, targetCount, timeSteps, currentTime, cfg);
        if (!best.valid)
        {
            std::cerr << "Error: no valid target solution\n";
            break;
        }

        Coord dirVec{std::cos(drone.dir), std::sin(drone.dir)};

        steps[step].pos             = drone.pos;
        steps[step].direction       = drone.dir;
        steps[step].state           = static_cast<int>(drone.state);
        steps[step].targetIdx       = best.targetIndex;
        steps[step].dropPoint       = best.dropPoint;
        steps[step].aimPoint        = drone.pos + dirVec * best.horizontalRange;
        steps[step].predictedTarget = interpolateTarget(
                                        targets,
                                        best.targetIndex,
                                        currentTime + best.flightTime,
                                        cfg.arrayTimeStep,
                                        timeSteps);

        DEBUG("step=" << step
              << " pos=(" << drone.pos.x << "," << drone.pos.y << ")"
              << " target=" << best.targetIndex
              << " state=" << static_cast<int>(drone.state));

        drone.currentTargetIndex = best.targetIndex;

        if (reachedDropPoint(drone, best, cfg.hitRadius))
        {
            dropped = true;
            ++step;
            break;
        }

        updateDrone(drone, best, cfg);
        currentTime += cfg.simTimeStep;
        ++step;
    }

    writeSimulationJson("simulation.json", steps, step);
    LOG("Simulation complete. Steps: " << step);

    delete[] steps;
    steps = nullptr;

    freeTargets(targets, targetCount);

    delete[] ammo;
    ammo = nullptr;

    return 0;
}