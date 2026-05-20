module;

#include <cmath>
#include <string>

export module mission_types;

export constexpr float G = 9.81f;
export constexpr float EPS = 1e-6f;

export enum class DroneState {
    Stopped = 0,
    Accelerating = 1,
    Decelerating = 2,
    Turning = 3,
    Moving = 4
};

export struct Coord {
    float x{};
    float y{};

    Coord operator+(const Coord& other) const { return {x + other.x, y + other.y}; }
    Coord operator-(const Coord& other) const { return {x - other.x, y - other.y}; }
    Coord operator*(float s) const { return {x * s, y * s}; }
    Coord operator/(float s) const { return {x / s, y / s}; }

    Coord& operator+=(const Coord& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
};

export inline float length(Coord c) {
    return std::hypot(c.x, c.y);
}

export inline float distance2D(Coord a, Coord b) {
    return length(a - b);
}

export struct AmmoParams {
    std::string name{};
    float mass{};
    float drag{};
    float lift{};
};

export struct DroneConfig {
    Coord startPos{};
    float altitude{};
    float initialDir{};
    float attackSpeed{};
    float accelPath{};
    std::string ammoName{};
    float arrayTimeStep{};
    float simTimeStep{};
    float hitRadius{};
    float angularSpeed{};
    float turnThreshold{};
};

export struct TargetState {
    Coord pos{};
    Coord vel{};
};

export struct Drone {
    Coord pos{};
    float z{};
    float dir{};
    float speed{};
    float attackSpeed{};
    float acceleration{};
    DroneState state{DroneState::Stopped};
    int currentTargetIndex{-1};
};

export struct DropPoint {
    bool valid{false};
    int targetIndex{-1};
    Coord dropPoint{};
    Coord predictedTarget{};
    float desiredDir{};
    float flightTime{};
    float horizontalRange{};
    float totalCost{};
};

export struct SimStep {
    Coord pos{};
    float direction{};
    DroneState state{};
    int targetIdx{};
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
};