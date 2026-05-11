#define _USE_MATH_DEFINES 
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <algorithm>

/**
 * @brief Here we declare consts which are used for calculations
*/
constexpr int TARGET_COUNT = 5;
constexpr int TIME_SAMPLES = 60;
constexpr int MAX_STEPS = 10000;
constexpr int NAME_SIZE = 32;
constexpr float PI = M_PI;
constexpr float G = 9.81f;
constexpr int AMMO_COUNT = 5;

/**
 * @brief Array with bomb names
*/
const char bombNames[AMMO_COUNT][15] =
{
    "VOG-17",
    "M67",
    "RKG-3",
    "GLIDING-VOG",
    "GLIDING-RKG"
};

/**
 * @brief Array with bomb masses 
*/
const float bombM[AMMO_COUNT] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};

/**
 * @brief Array with bomb drag params 
*/
const float bombD[AMMO_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};

/**
 * @brief Array with bomb lift params 
*/
const float bombL[AMMO_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

/**
 * @brief Enumeration with different drone states 
*/
enum DroneState
{
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

/**
 * @brief Structure that contain input data 
*/
struct InputData
{
    float xd, yd, zd;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    char ammo_name[NAME_SIZE];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

/**
 * @brief Structure with chosen bomb name and parameters 
*/
struct Ammo
{
    char name[15];
    float m;
    float d;
    float l;
};

/**
 * @brief This struct is useful to contain both X and Y coordinates at the same place 
*/
struct Vec2
{
    float x;
    float y;
};

/**
 * @brief Structure that contains current drone parameters 
*/
struct Drone
{
    float x;
    float y;
    float z;
    float dir;
    float speed;
    float attackSpeed;
    float acceleration;
    DroneState state;
};

/**
 * @brief Structure that contain terget state and parameters at some point of time
*/
struct TargetState
{
    float x;
    float y;
    float vx;
    float vy;
};

/**
 * @brief This struct contains resolved params for drop coordinates, predicted (lineary interpolated) target coords and the direction that drone must look at
*/
struct Solution
{
    bool valid;
    int targetIndex;
    float totalTime;
    float dropX;
    float dropY;
    float predictedTargetX;
    float predictedTargetY;
    float desiredDir;
};

/**
 * @brief This function compares ammo name from input with ammo names from bombNames array and sets ammo params in Ammo structure 
*/
bool chooseAmmo(const char* ammoName, Ammo& ammo);

/**
 * @brief This function reads the input data from input.txt 
*/
bool readInput(const char* filename, InputData& in);

/**
 * @brief This function reads targets.txt file with data about targets X and Y coordinates 
*/
bool readTargets(const char* filename,
                 float targetX[TARGET_COUNT][TIME_SAMPLES],
                 float targetY[TARGET_COUNT][TIME_SAMPLES]);

/**
 * @brief This function implements linear interpolation of one of the target coordinate that we need to calculate drop point coords and then to choose the closest one 
*/
float interpolate1D(const float arr[TIME_SAMPLES], float t, float arrayTimeStep);

/**
 * @brief This function returns interpolated coordinates of target 
*/
Vec2 interpolateTarget(const float targetX[TARGET_COUNT][TIME_SAMPLES],
                       const float targetY[TARGET_COUNT][TIME_SAMPLES],
                       int targetIndex,
                       float t,
                       float arrayTimeStep);

TargetState getTargetState(const float targetX[TARGET_COUNT][TIME_SAMPLES],
                           const float targetY[TARGET_COUNT][TIME_SAMPLES],
                           int targetIndex,
                           float t,
                           float dt,
                           float arrayTimeStep);

float normalizeAngle(float a);

bool computeDropPoint(
    float droneX,
    float droneY,
    float droneZ,
    float targetX,
    float targetY,
    float attackSpeed,
    float accelerationPath,
    const Ammo& ammo,
    float& dropX,
    float& dropY,
    float& flightTime,
    float& horizontalRange);

float distance2D(float x1, float y1, float x2, float y2);

Solution estimateIntercept(const Drone& drone,
                           const Ammo& ammo,
                           const float targetX[TARGET_COUNT][TIME_SAMPLES],
                           const float targetY[TARGET_COUNT][TIME_SAMPLES],
                           int targetIndex,
                           float currentTime,
                           const InputData& in);

/**
 * @brief Here we choose the closest drop point in terms of the time of arrival  
*/
Solution chooseBestTarget(const Drone& drone,
                          const Ammo& ammo,
                          const float targetX[TARGET_COUNT][TIME_SAMPLES],
                          const float targetY[TARGET_COUNT][TIME_SAMPLES],
                          float currentTime,
                          const InputData& in);

/**
 * @brief This function updates current drone state and target 
*/
void updateDrone(Drone& drone, const Solution& sol, const InputData& in);

/**
 * @brief This function is needed to stop the simulation when the bomb has reached the target 
*/
bool reachedDropPoint(const Drone& drone, const Solution& sol, float hitRadius);

/**
 * @brief This function writes params into output 
*/
void writeSimulation(const char* filename,
                     int steps,
                     const float outX[MAX_STEPS],
                     const float outY[MAX_STEPS],
                     const float outDir[MAX_STEPS],
                     const int outState[MAX_STEPS],
                     const int outTarget[MAX_STEPS]);

int main()
{
    InputData input{};
    if (!readInput("input.txt", input))
    {
        std::cerr << "Error: cannot read input.txt\n";
        return 1;
    }

    if (input.simTimeStep <= 0.0f)
    {
        std::cerr << "Error: simTimeStep must be > 0\n";
        return 1;
    }

    if (input.arrayTimeStep <= 0.0f)
    {
        std::cerr << "Error: arrayTimeStep must be > 0\n";
        return 1;
    }

    if (input.accelerationPath <= 0.0f)
    {
        std::cerr << "Error: accelerationPath must be > 0\n";
        return 1;
    }

    float targetX[TARGET_COUNT][TIME_SAMPLES];
    float targetY[TARGET_COUNT][TIME_SAMPLES];

    if (!readTargets("targets.txt", targetX, targetY))
    {
        std::cerr << "Error: cannot read targets.txt\n";
        return 1;
    }

    Ammo ammo{};
    if (!chooseAmmo(input.ammo_name, ammo))
    {
        std::cerr << "Error: unknown ammo name\n";
        return 1;
    }

    Drone drone{};
    drone.x = input.xd;
    drone.y = input.yd;
    drone.z = input.zd;
    drone.dir = input.initialDir;
    drone.speed = 0.0f;
    drone.attackSpeed = input.attackSpeed;
    drone.acceleration = (input.attackSpeed * input.attackSpeed) / (2.0f * input.accelerationPath);
    drone.state = STOPPED;

    float currentTime = 0.0f;
    int step = 0;
    bool dropped = false;

    float outX[MAX_STEPS];
    float outY[MAX_STEPS];
    float outDir[MAX_STEPS];
    int outState[MAX_STEPS];
    int outTarget[MAX_STEPS];

    while (step < MAX_STEPS)
    {
        Solution best = chooseBestTarget(drone, ammo, targetX, targetY, currentTime, input);
        if (!best.valid)
        {
            std::cerr << "Error: no valid target solution\n";
            return 1;
        }

        outX[step] = drone.x;
        outY[step] = drone.y;
        outDir[step] = drone.dir;
        outState[step] = static_cast<int>(drone.state);
        outTarget[step] = best.targetIndex;

        if (reachedDropPoint(drone, best, input.hitRadius))
        {
            dropped = true;
            ++step;
            break;
        }

        updateDrone(drone, best, input);

        currentTime += input.simTimeStep;
        ++step;
    }

    writeSimulation("simulation.txt", step, outX, outY, outDir, outState, outTarget);

    if (!dropped)
    {
        std::cerr << "Drop point was not reached within MAX_STEPS iterations of simulation\n";
    }

    return 0;
}


bool chooseAmmo(const char* ammoName, Ammo& ammo)
{
    for (int i = 0; i < AMMO_COUNT; ++i)
    {
        if (std::strcmp(ammoName, bombNames[i]) == 0)
        {
            std::strcpy(ammo.name, bombNames[i]);
            ammo.m = bombM[i];
            ammo.d = bombD[i];
            ammo.l = bombL[i];
            return true;
        }
    }
    return false;
}

bool readInput(const char* filename, InputData& in)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        return false;
    }

    if (!(file >> in.xd >> in.yd >> in.zd
              >> in.initialDir
              >> in.attackSpeed
              >> in.accelerationPath
              >> in.ammo_name
              >> in.arrayTimeStep
              >> in.simTimeStep
              >> in.hitRadius
              >> in.angularSpeed
              >> in.turnThreshold))
    {
        return false;
    }

    return true;
}

bool readTargets(const char* filename,
                 float targetX[TARGET_COUNT][TIME_SAMPLES],
                 float targetY[TARGET_COUNT][TIME_SAMPLES])
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        return false;
    }

    for (int i = 0; i < TARGET_COUNT; ++i)
    {
        for (int j = 0; j < TIME_SAMPLES; ++j)
        {
            if (!(file >> targetX[i][j]))
            {
                return false;
            }
        }
    }

    for (int i = 0; i < TARGET_COUNT; ++i)
    {
        for (int j = 0; j < TIME_SAMPLES; ++j)
        {
            if (!(file >> targetY[i][j]))
            {
                return false;
            }
        }
    }

    return true;
}

float interpolate1D(const float arr[TIME_SAMPLES], float t, float arrayTimeStep)
{
    int idx = static_cast<int>(std::floor(t / arrayTimeStep)) % TIME_SAMPLES;
    if (idx < 0) idx += TIME_SAMPLES;

    int next = (idx + 1) % TIME_SAMPLES;
    float localT = t - std::floor(t / arrayTimeStep) * arrayTimeStep;
    float frac = localT / arrayTimeStep;

    return arr[idx] + (arr[next] - arr[idx]) * frac;
}

Vec2 interpolateTarget(const float targetX[TARGET_COUNT][TIME_SAMPLES],
                       const float targetY[TARGET_COUNT][TIME_SAMPLES],
                       int targetIndex,
                       float t,
                       float arrayTimeStep)
{
    Vec2 p;
    p.x = interpolate1D(targetX[targetIndex], t, arrayTimeStep);
    p.y = interpolate1D(targetY[targetIndex], t, arrayTimeStep);
    return p;
}

TargetState getTargetState(const float targetX[TARGET_COUNT][TIME_SAMPLES],
                           const float targetY[TARGET_COUNT][TIME_SAMPLES],
                           int targetIndex,
                           float t,
                           float dt,
                           float arrayTimeStep)
{
    TargetState s{};

    if (dt <= 0.0f)
    {
        std::cerr << "Error: dt must be > 0\n";
        return s;
    }

    Vec2 p1 = interpolateTarget(targetX, targetY, targetIndex, t, arrayTimeStep);
    Vec2 p2 = interpolateTarget(targetX, targetY, targetIndex, t + dt, arrayTimeStep);

    s.x = p1.x;
    s.y = p1.y;
    s.vx = (p2.x - p1.x) / dt;
    s.vy = (p2.y - p1.y) / dt;

    return s;
}

float normalizeAngle(float a)
{
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

bool computeDropPoint(
    float droneX,
    float droneY,
    float droneZ,
    float targetX,
    float targetY,
    float attackSpeed,
    float accelerationPath,
    const Ammo& ammo,
    float& dropX,
    float& dropY,
    float& flightTime,
    float& horizontalRange)
{
    if (droneZ > 100.0f)
    {
        // не ошибка, просто ограничение модели
    }

    const float a = ammo.d * G * ammo.m
                  - 2.0f * ammo.d * ammo.d * ammo.l * attackSpeed;

    const float b = -3.0f * G * ammo.m * ammo.m
                  + 3.0f * ammo.d * ammo.l * ammo.m * attackSpeed;

    const float c = 6.0f * ammo.m * ammo.m * droneZ;

    if (a == 0.0f)
    {
        return false;
    }

    const float p = -(std::pow(b, 2.0f) / (3.0f * std::pow(a, 2.0f)));
    const float q = (2.0f * std::pow(b, 3.0f)) / (27.0f * std::pow(a, 3.0f)) + c / a;

    if (p >= 0.0f)
    {
        return false;
    }

    float acosArg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
    if (acosArg < -1.0f) acosArg = -1.0f;
    if (acosArg >  1.0f) acosArg =  1.0f;

    const float phi = std::acos(acosArg);

    const float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * PI) / 3.0f)
                  - b / (3.0f * a);

    const float h =
          attackSpeed * t
        - std::pow(t, 2.0f) * ammo.d * attackSpeed / (2.0f * ammo.m)
        + std::pow(t, 3.0f) *
          (
              6.0f * ammo.d * G * ammo.l * ammo.m
            - 6.0f * std::pow(ammo.d, 2.0f) * (std::pow(ammo.l, 2.0f) - 1.0f) * attackSpeed
          ) / (36.0f * std::pow(ammo.m, 2.0f))
        + std::pow(t, 4.0f) *
          (
              -6.0f * std::pow(ammo.d, 2.0f) * G * ammo.l *
              (1.0f + std::pow(ammo.l, 2.0f) + std::pow(ammo.l, 4.0f)) * ammo.m
            + 3.0f * std::pow(ammo.d, 3.0f) * std::pow(ammo.l, 2.0f) *
              (1.0f + std::pow(ammo.l, 2.0f)) * attackSpeed
            + 6.0f * std::pow(ammo.d, 3.0f) * std::pow(ammo.l, 4.0f) *
              (1.0f + std::pow(ammo.l, 2.0f) * attackSpeed)
          ) / (36.0f * std::pow(1.0f + std::pow(ammo.l, 2.0f), 2.0f) * std::pow(ammo.m, 3.0f))
        + std::pow(t, 5.0f) *
          (
              3.0f * std::pow(ammo.d, 3.0f) * G * std::pow(ammo.l, 3.0f) * ammo.m
            - 3.0f * std::pow(ammo.d, 4.0f) * std::pow(ammo.l, 2.0f) *
              (1.0f + std::pow(ammo.l, 2.0f)) * attackSpeed
          ) / (36.0f * (1.0f + std::pow(ammo.l, 2.0f)) * std::pow(ammo.m, 4.0f));

    const float dx = targetX - droneX;
    const float dy = targetY - droneY;
    const float D = std::sqrt(dx * dx + dy * dy);

    if (D == 0.0f)
    {
        return false;
    }

    const float ratio = (D - h) / D;

    if (h + accelerationPath > D)
    {
        const float xdm = targetX - dx * (h + accelerationPath) / D;
        const float ydm = targetY - dy * (h + accelerationPath) / D;

        dropX = xdm + (targetX - xdm) * ratio;
        dropY = ydm + (targetY - ydm) * ratio;
    }
    else
    {
        dropX = droneX + dx * ratio;
        dropY = droneY + dy * ratio;
    }

    flightTime = t;
    horizontalRange = h;

    return true;
}

float distance2D(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

Solution estimateIntercept(const Drone& drone,
                           const Ammo& ammo,
                           const float targetX[TARGET_COUNT][TIME_SAMPLES],
                           const float targetY[TARGET_COUNT][TIME_SAMPLES],
                           int targetIndex,
                           float currentTime,
                           const InputData& in)
{
    Solution sol{};
    sol.valid = false;
    sol.targetIndex = targetIndex;

    TargetState ts = getTargetState(targetX, targetY, targetIndex,
                                    currentTime, in.simTimeStep, in.arrayTimeStep);

    // here we get first rough estimation of the distance and arrival time of the drone to the drop point
    float roughDistance = distance2D(drone.x, drone.y, ts.x, ts.y);
    float roughTime = roughDistance / std::max(drone.attackSpeed, 0.001f);

    float predictedX = ts.x + ts.vx * roughTime;
    float predictedY = ts.y + ts.vy * roughTime;

    float dropX = 0.0f;
    float dropY = 0.0f;
    float flightTime = 0.0f;
    float horizontalRange = 0.0f;

    if (!computeDropPoint(drone.x, drone.y, drone.z,
                        predictedX, predictedY,
                        drone.attackSpeed,
                        in.accelerationPath,
                        ammo,
                        dropX, dropY,
                        flightTime, horizontalRange))
    {
        return sol;
    }

    float dx = dropX - drone.x;
    float dy = dropY - drone.y;
    float desiredDir = std::atan2(dy, dx);
    float angularDiff = normalizeAngle(desiredDir - drone.dir);

    float pathTime = distance2D(drone.x, drone.y, dropX, dropY) /
                     std::max(drone.attackSpeed, 0.001f);

    float extraTurnTime = 0.0f;
    if (std::fabs(angularDiff) > in.turnThreshold)
    {
        float stopTime = drone.speed / std::max(drone.acceleration, 0.001f);
        float turnTime = std::fabs(angularDiff) / in.angularSpeed;
        float accelTime = drone.attackSpeed / drone.acceleration;
        extraTurnTime = stopTime + turnTime + accelTime;
    }

    sol.valid = true;
    sol.totalTime = pathTime + extraTurnTime;
    sol.dropX = dropX;
    sol.dropY = dropY;
    sol.predictedTargetX = predictedX;
    sol.predictedTargetY = predictedY;
    sol.desiredDir = desiredDir;

    return sol;
}

Solution chooseBestTarget(const Drone& drone,
                          const Ammo& ammo,
                          const float targetX[TARGET_COUNT][TIME_SAMPLES],
                          const float targetY[TARGET_COUNT][TIME_SAMPLES],
                          float currentTime,
                          const InputData& in)
{
    Solution best{};
    best.valid = false;
    best.totalTime = 1e30f;

    for (int i = 0; i < TARGET_COUNT; ++i)
    {
        Solution cur = estimateIntercept(drone, ammo, targetX, targetY, i, currentTime, in);
        if (cur.valid && cur.totalTime < best.totalTime)
        {
            best = cur;
        }
    }

    return best;
}

void updateDrone(Drone& drone, const Solution& sol, const InputData& in)
{
    float desiredDir = sol.desiredDir;
    float angleDiff = normalizeAngle(desiredDir - drone.dir);
    float dt = in.simTimeStep;

    if (std::fabs(angleDiff) > in.turnThreshold)
    {
        if (drone.speed > 0.0f)
        {
            drone.state = DECELERATING;
            drone.speed -= drone.acceleration * dt;
            if (drone.speed < 0.0f) drone.speed = 0.0f;
        }
        else
        {
            drone.state = TURNING;
            float maxTurn = in.angularSpeed * dt;

            if (std::fabs(angleDiff) <= maxTurn)
            {
                drone.dir = desiredDir;
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
        drone.dir = desiredDir;

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

    drone.x += drone.speed * std::cos(drone.dir) * dt;
    drone.y += drone.speed * std::sin(drone.dir) * dt;
}

bool reachedDropPoint(const Drone& drone, const Solution& sol, float hitRadius)
{
    return distance2D(drone.x, drone.y, sol.dropX, sol.dropY) <= hitRadius;
}

void writeSimulation(const char* filename,
                     int steps,
                     const float outX[MAX_STEPS],
                     const float outY[MAX_STEPS],
                     const float outDir[MAX_STEPS],
                     const int outState[MAX_STEPS],
                     const int outTarget[MAX_STEPS])
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        return;
    }

    file << steps << '\n';

    for (int i = 0; i < steps; ++i)
    {
        file << outX[i] << ' ' << outY[i];
        if (i + 1 < steps) file << ' ';
    }
    file << '\n';

    for (int i = 0; i < steps; ++i)
    {
        file << outDir[i];
        if (i + 1 < steps) file << ' ';
    }
    file << '\n';

    for (int i = 0; i < steps; ++i)
    {
        file << outState[i];
        if (i + 1 < steps) file << ' ';
    }
    file << '\n';

    for (int i = 0; i < steps; ++i)
    {
        file << outTarget[i];
        if (i + 1 < steps) file << ' ';
    }
    file << '\n';
}