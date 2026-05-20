export module mission_types;

import <cmath>;

export constexpr float G = 9.81f;
export constexpr float EPS = 1e-6f;

export struct Coord {
    float x{};
    float y{};

    Coord operator+(const Coord&) const;
    Coord operator-(const Coord&) const;
    Coord operator*(float) const;
};

export struct AmmoParams {
    std::string name;
    float mass{};
    float drag{};
    float lift{};
};

export struct DroneConfig {
    Coord startPos{};
    float altitude{};
    float attackSpeed{};
    float accelPath{};
    float angularSpeed{};
    float simTimeStep{};
    float hitRadius{};
    std::string ammoName;
};

export struct TargetState {
    Coord pos{};
    Coord vel{};
};

export struct DropPoint {
    bool valid{};
    Coord position{};
    float flightTime{};
    float horizontalRange{};
};

export struct Drone {
    Coord pos{};
    float dir{};
    float speed{};
};