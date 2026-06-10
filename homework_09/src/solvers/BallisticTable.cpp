#include "solvers/BallisticTable.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

bool BallisticTable::load(
    const std::string& path
)
{
    std::ifstream fin(path);

    if (!fin.is_open())
    {
        return false;
    }

    int nZ{};
    int nV{};
    int nM{};
    int nD{};
    int nL{};

    fin >> nZ >> nV >> nM >> nD >> nL;

    axisZ0_.resize(nZ);
    axisV0_.resize(nV);
    axisM_.resize(nM);
    axisD_.resize(nD);
    axisL_.resize(nL);

    for (auto& v : axisZ0_)
    {
        fin >> v;
    }

    for (auto& v : axisV0_)
    {
        fin >> v;
    }

    for (auto& v : axisM_)
    {
        fin >> v;
    }

    for (auto& v : axisD_)
    {
        fin >> v;
    }

    for (auto& v : axisL_)
    {
        fin >> v;
    }

    const std::size_t totalSize =
          static_cast<std::size_t>(nZ)
        * static_cast<std::size_t>(nV)
        * static_cast<std::size_t>(nM)
        * static_cast<std::size_t>(nD)
        * static_cast<std::size_t>(nL);

    data_.resize(totalSize);

    for (auto& item : data_)
    {
        fin >> item.flightTime
            >> item.horizontalRange;
    }

    return true;
}

std::size_t BallisticTable::index(
    int iz,
    int iv,
    int im,
    int id,
    int il
) const
{
    return
        ((((static_cast<std::size_t>(iz)
            * axisV0_.size())
            + iv)
            * axisM_.size()
            + im)
            * axisD_.size()
            + id)
            * axisL_.size()
            + il;
}

const BallisticResult&
BallisticTable::at(
    int iz,
    int iv,
    int im,
    int id,
    int il
) const
{
    return data_.at(
        index(
            iz,
            iv,
            im,
            id,
            il
        )
    );
}

BallisticTable::Interp
BallisticTable::findInterp(
    float value,
    const std::vector<float>& axis
)
{
    if (axis.size() < 2)
    {
        throw std::runtime_error(
            "axis too small"
        );
    }

    if (value <= axis.front())
    {
        return {0, 0.0f};
    }

    if (value >= axis.back())
    {
        return {
            static_cast<int>(
                axis.size() - 2
            ),
            1.0f
        };
    }

    for (std::size_t i = 0;
         i < axis.size() - 1;
         ++i)
    {
        if (value >= axis[i]
            && value <= axis[i + 1])
        {
            const float frac =
                (value - axis[i])
                /
                (axis[i + 1]
                 - axis[i]);

            return {
                static_cast<int>(i),
                frac
            };
        }
    }

    return {
        static_cast<int>(
            axis.size() - 2
        ),
        1.0f
    };
}

BallisticResult
BallisticTable::lerp(
    const BallisticResult& a,
    const BallisticResult& b,
    float t
)
{
    return {
        .flightTime =
            a.flightTime
            + (b.flightTime
               - a.flightTime)
                  * t,

        .horizontalRange =
            a.horizontalRange
            + (b.horizontalRange
               - a.horizontalRange)
                  * t
    };
}

BallisticResult
BallisticTable::lookup(
    float altitude,
    float velocity,
    float mass,
    float drag,
    float lift
) const
{
    const auto z =
        findInterp(
            altitude,
            axisZ0_
        );

    const auto v =
        findInterp(
            velocity,
            axisV0_
        );

    const auto m =
        findInterp(
            mass,
            axisM_
        );

    const auto d =
        findInterp(
            drag,
            axisD_
        );

    const auto l =
        findInterp(
            lift,
            axisL_
        );

    BallisticResult corners[32];

    int idx = 0;

    for (int bz = 0; bz < 2; ++bz)
    {
        for (int bv = 0; bv < 2; ++bv)
        {
            for (int bm = 0; bm < 2; ++bm)
            {
                for (int bd = 0; bd < 2; ++bd)
                {
                    for (int bl = 0; bl < 2; ++bl)
                    {
                        corners[idx++] =
                            at(
                                z.lo + bz,
                                v.lo + bv,
                                m.lo + bm,
                                d.lo + bd,
                                l.lo + bl
                            );
                    }
                }
            }
        }
    }

    int count = 32;

    float weights[5] =
    {
        z.frac,
        v.frac,
        m.frac,
        d.frac,
        l.frac
    };

    for (int axis = 4;
         axis >= 0;
         --axis)
    {
        int out = 0;

        for (int i = 0;
             i < count;
             i += 2)
        {
            corners[out++] =
                lerp(
                    corners[i],
                    corners[i + 1],
                    weights[axis]
                );
        }

        count /= 2;
    }

    return corners[0];
}