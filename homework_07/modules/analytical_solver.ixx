export module analytical_solver;
        {
            return result;
        }

        float acosArg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
        acosArg = std::clamp(acosArg, -1.0f, 1.0f);

        constexpr float pi = 3.14159265358979323846f;

        const float phi = std::acos(acosArg);
        const float t = 2.0f * std::sqrt(-p / 3.0f) *
                        std::cos((phi + 4.0f * pi) / 3.0f)
                      - b / (3.0f * a);

        if (t <= 0.0f)
        {
            return result;
        }

        const float h = computeHorizontalRange(t, attackSpeed, ammo);

        const Coord delta = targetPos - dronePos;
        const float D = length(delta);

        if (D < EPS)
        {
            return result;
        }

        const float ratio = (D - h) / D;

        Coord firePoint{};

        if (h + accelerationPath > D)
        {
            const Coord maneuverPoint = targetPos - delta * ((h + accelerationPath) / D);
            firePoint = maneuverPoint + (targetPos - maneuverPoint) * ratio;
        }
        else
        {
            firePoint = dronePos + delta * ratio;
        }

        result.valid = true;
        result.dropPoint = firePoint;
        result.flightTime = t;
        result.horizontalRange = h;
        return result;
    }

private:
    static float computeHorizontalRange(
        float t,
        float attackSpeed,
        const AmmoParams& ammo
    )
    {
        return
              attackSpeed * t
            - std::pow(t, 2.0f) * ammo.drag * attackSpeed / (2.0f * ammo.mass)
            + std::pow(t, 3.0f) *
              (
                  6.0f * ammo.drag * G * ammo.lift * ammo.mass
                - 6.0f * std::pow(ammo.drag, 2.0f) *
                  (std::pow(ammo.lift, 2.0f) - 1.0f) * attackSpeed
              ) / (36.0f * std::pow(ammo.mass, 2.0f))
            + std::pow(t, 4.0f) *
              (
                  -6.0f * std::pow(ammo.drag, 2.0f) * G * ammo.lift *
                  (1.0f + std::pow(ammo.lift, 2.0f) + std::pow(ammo.lift, 4.0f)) * ammo.mass
                + 3.0f * std::pow(ammo.drag, 3.0f) * std::pow(ammo.lift, 2.0f) *
                  (1.0f + std::pow(ammo.lift, 2.0f)) * attackSpeed
                + 6.0f * std::pow(ammo.drag, 3.0f) * std::pow(ammo.lift, 4.0f) *
                  (1.0f + std::pow(ammo.lift, 2.0f) * attackSpeed)
              ) / (36.0f * std::pow(1.0f + std::pow(ammo.lift, 2.0f), 2.0f) *
                   std::pow(ammo.mass, 3.0f))
            + std::pow(t, 5.0f) *
              (
                  3.0f * std::pow(ammo.drag, 3.0f) * G *
                  std::pow(ammo.lift, 3.0f) * ammo.mass
                - 3.0f * std::pow(ammo.drag, 4.0f) * std::pow(ammo.lift, 2.0f) *
                  (1.0f + std::pow(ammo.lift, 2.0f)) * attackSpeed
              ) / (36.0f * (1.0f + std::pow(ammo.lift, 2.0f)) *
                   std::pow(ammo.mass, 4.0f));
    }
};