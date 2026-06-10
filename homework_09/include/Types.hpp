#pragma once

#include <cmath>
#include <string>

constexpr float G = 9.81f;
constexpr float EPS = 1e-6f;

struct Coord
{
    float x{};
    float y{};

    Coord operator+(const Coord& rhs) const
    {
        return {
            x + rhs.x,
            y + rhs.y
        };
    }

    Coord operator-(const Coord& rhs) const
    {
        return {
            x - rhs.x,
            y - rhs.y
        };
    }

    Coord operator*(float scalar) const
    {
        return {
            x * scalar,
            y * scalar
        };
    }

    Coord operator/(float scalar) const
    {
        return {
            x / scalar,
            y / scalar
        };
    }

    Coord& operator+=(const Coord& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
};

inline float length(const Coord& v)
{
    return std::sqrt(
        v.x * v.x +
        v.y * v.y
    );
}

inline float distance2D(
    const Coord& a,
    const Coord& b
)
{
    return length(b - a);
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

    float angularSpeed{};

    float turnThreshold{};

    float simTimeStep{};

    float hitRadius{};

    float arrayTimeStep{};

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

    float speed{};

    float dir{};

    float attackSpeed{};

    float acceleration{};

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

struct BallisticResult
{
    float flightTime{};
    float horizontalRange{};
};

struct SimStep
{
    Coord pos{};

    float direction{};

    std::string stateName{};

    int targetIdx{-1};

    Coord dropPoint{};

    Coord aimPoint{};

    Coord predictedTarget{};
};