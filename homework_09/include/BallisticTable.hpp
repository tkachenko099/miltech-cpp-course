#pragma once

#include "Types.hpp"

#include <string>
#include <vector>

class BallisticTable
{
public:
    struct Interp
    {
        int lo{};
        float frac{};
    };

    bool load(const std::string& path);

    BallisticResult lookup(
        float altitude,
        float velocity,
        float mass,
        float drag,
        float lift
    ) const;

private:
    std::size_t index(
        int iz,
        int iv,
        int im,
        int id,
        int il
    ) const;

    const BallisticResult& at(
        int iz,
        int iv,
        int im,
        int id,
        int il
    ) const;

    static Interp findInterp(
        float value,
        const std::vector<float>& axis
    );

    static BallisticResult lerp(
        const BallisticResult& a,
        const BallisticResult& b,
        float t
    );

private:
    std::vector<float> axisZ0_;
    std::vector<float> axisV0_;
    std::vector<float> axisM_;
    std::vector<float> axisD_;
    std::vector<float> axisL_;

    std::vector<BallisticResult> data_;
};