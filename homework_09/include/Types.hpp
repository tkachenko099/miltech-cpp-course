#pragma once

#include <cmath>
#include <cstddef>
#include <string>

constexpr float G = 9.81f;
constexpr float EPS = 1e-6f;

enum class DroneState
{
    Stopped = 0,
    Accelerating = 1,
    Decelerating = 2,
    Turning = 3,
    Moving = 4
};

struct BallisticResult
{
    float flightTime{};
    float horizontalRange{};
};

struct Coord
{
    float x{};
    float y{};

    constexpr Coord operator+(const Coord& other) const
    {
        return {
            x + other.x,
            y + other.y
        };
    }

    constexpr Coord operator-(const Coord& other) const
    {
        return {
            x - other.x,
            y - other.y
        };
    }

    constexpr Coord operator*(float scalar) const
    {
        return {
            x * scalar,
            y * scalar
        };
    }

    constexpr Coord operator/(float scalar) const
    {
        return {
            x / scalar,
            y / scalar
        };
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

    Coord& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Coord& operator/=(float scalar)
    {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

inline Coord operator*(
    float scalar,
    const Coord& coord
)
{
    return {
        scalar * coord.x,
        scalar * coord.y
    };
}

inline float length(const Coord& coord)
{
    return std::hypot(
        coord.x,
        coord.y
    );
}

inline float distance2D(
    const Coord& a,
    const Coord& b
)
{
    return length(b - a);
}

inline Coord normalize(
    const Coord& coord
)
{
    const float len = length(coord);

    if (len < EPS)
    {
        return {0.0f, 0.0f};
    }

    return coord / len;
}

struct AmmoParams
{
    std::string name{};

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

    float arrayTimeStep{};
    float simTimeStep{};

    float hitRadius{};

    float angularSpeed{};
    float turnThreshold{};

    std::string ammoName{};
};

struct TargetState
{
    Coord pos{};
    Coord vel{};
};

struct Drone
{
    Coord pos{};

    float z{};
    float dir{};

    float speed{};
    float attackSpeed{};
    float acceleration{};

    DroneState state{
        DroneState::Stopped
    };

    int currentTargetIndex{-1};
};

struct DropPoint
{
    bool valid{false};

    int targetIndex{-1};

    Coord dropPoint{};
    Coord predictedTarget{};

    float desiredDir{};

    float flightTime{};
    float horizontalRange{};
    float totalCost{};
};

struct SimStep
{
    Coord pos{};

    float direction{};

    DroneState state{
        DroneState::Stopped
    };

    int targetIdx{-1};

    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
};