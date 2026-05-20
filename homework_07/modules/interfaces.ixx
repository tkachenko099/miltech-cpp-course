export module interfaces;

import mission_types;

export class ITargetProvider {
public:
    virtual int getTargetCount() const = 0;
    virtual TargetState getTarget(int index, float t, float dt) const = 0;
    virtual ~ITargetProvider() = default;
};

export class IBallisticSolver {
public:
    virtual DropPoint solve(
        Coord dronePos,
        float altitude,
        Coord targetPos,
        float attackSpeed,
        float accelerationPath,
        const AmmoParams& ammo
    ) const = 0;

    virtual ~IBallisticSolver() = default;
};

export class IConfigLoader {
public:
    virtual void load(const char* configPath, const char* ammoPath) = 0;
    virtual const DroneConfig& getConfig() const = 0;
    virtual const AmmoParams& getAmmoParams() const = 0;
    virtual ~IConfigLoader() = default;
};