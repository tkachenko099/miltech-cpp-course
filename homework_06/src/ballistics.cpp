#include "ballistics.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

AmmoParams chooseAmmoParams(const std::string &typeStr) {
  if (typeStr == "VOG-17") {
    return {0.35F, 0.07F, 0.0F};
  }

  if (typeStr == "M67") {
    return {0.60F, 0.10F, 0.0F};
  }

  if (typeStr == "RKG-3") {
    return {1.20F, 0.10F, 0.0F};
  }

  if (typeStr == "GLIDING-VOG") {
    return {0.45F, 0.10F, 1.0F};
  }

  if (typeStr == "GLIDING-RKG") {
    return {1.40F, 0.10F, 1.0F};
  }

  throw std::runtime_error("Unknown ammo type: " + typeStr);
}

InputData readInputFile(const std::string &filename) {
  std::ifstream file(filename);

  if (!file) {
    throw std::runtime_error("cannot open input file: " + filename);
  }

  InputData data{};

  if (!(file >> data.xd >> data.yd >> data.zd >> data.targetX >> data.targetY >>
        data.attackSpeed >> data.accelerationPath)) {
    throw std::runtime_error("not enough numeric data in input file");
  }

  std::getline(file >> std::ws, data.ammoType);

  if (data.ammoType.empty()) {
    throw std::runtime_error("missing ammo type string");
  }

  const AmmoParams ammo = chooseAmmoParams(data.ammoType);

  data.ammoM = ammo.m;
  data.ammoD = ammo.d;
  data.ammoL = ammo.l;

  return data;
}

float calculate_h(const InputData &data, const float t) {
  const float t2 = std::pow(t, 2.0F);
  const float t3 = std::pow(t, 3.0F);
  const float t4 = std::pow(t, 4.0F);
  const float t5 = std::pow(t, 5.0F);

  const float ammoD2 = std::pow(data.ammoD, 2.0F);
  const float ammoD3 = std::pow(data.ammoD, 3.0F);
  const float ammoD4 = std::pow(data.ammoD, 4.0F);

  const float ammoM2 = std::pow(data.ammoM, 2.0F);
  const float ammoM3 = std::pow(data.ammoM, 3.0F);
  const float ammoM4 = std::pow(data.ammoM, 4.0F);

  const float ammoL2 = std::pow(data.ammoL, 2.0F);
  const float ammoL3 = std::pow(data.ammoL, 3.0F);
  const float ammoL4 = std::pow(data.ammoL, 4.0F);

  const float onePlusL2 = 1.0F + ammoL2;

  const float firstTerm = data.attackSpeed * t;

  const float secondTerm =
      t2 * data.ammoD * data.attackSpeed / (2.0F * data.ammoM);

  const float thirdTerm =
      t3 *
      ((6.0F * data.ammoD * G * data.ammoL * data.ammoM) -
       (6.0F * ammoD2 * (ammoL2 - 1.0F) * data.attackSpeed)) /
      (36.0F * ammoM2);

  const float fourthTerm =
      t4 *
      ((-6.0F * ammoD2 * G * data.ammoL * (1.0F + ammoL2 + ammoL4) *
        data.ammoM) +
       (3.0F * ammoD3 * ammoL2 * onePlusL2 * data.attackSpeed) +
       (6.0F * ammoD3 * ammoL4 * (1.0F + ammoL2 * data.attackSpeed))) /
      (36.0F * std::pow(onePlusL2, 2.0F) * ammoM3);

  const float fifthTerm =
      t5 *
      ((3.0F * ammoD3 * G * ammoL3 * data.ammoM) -
       (3.0F * ammoD4 * ammoL2 * onePlusL2 * data.attackSpeed)) /
      (36.0F * onePlusL2 * ammoM4);

  return firstTerm - secondTerm + thirdTerm + fourthTerm + fifthTerm;
}

DropSolution computeDropSolution(const InputData &data) {
  if (data.zd <= 0.0F) {
    throw std::runtime_error("drone altitude must be positive");
  }

  if (data.attackSpeed <= 0.0F) {
    throw std::runtime_error("attack speed must be positive");
  }

  if (data.ammoM <= 0.0F) {
    throw std::runtime_error("ammo mass must be positive");
  }

  const float a =
      (data.ammoD * G * data.ammoM) -
      (2.0F * data.ammoD * data.ammoD * data.ammoL * data.attackSpeed);

  const float b =
      (-3.0F * G * data.ammoM * data.ammoM) +
      (3.0F * data.ammoD * data.ammoL * data.ammoM * data.attackSpeed);

  const float c = 6.0F * data.ammoM * data.ammoM * data.zd;

  if (a == 0.0F) {
    throw std::runtime_error("coefficient a is zero");
  }

  const float p = -(std::pow(b, 2.0F) / (3.0F * std::pow(a, 2.0F)));

  const float q =
      (2.0F * std::pow(b, 3.0F)) / (27.0F * std::pow(a, 3.0F)) + (c / a);

  if (p >= 0.0F) {
    throw std::runtime_error("p must be negative for the current formula");
  }

  float acosArg = 3.0F * q / (2.0F * p) * std::sqrt(-3.0F / p);
  acosArg = std::clamp(acosArg, -1.0F, 1.0F);

  const float phi = std::acos(acosArg);

  const float t =
      (2.0F * std::sqrt(-p / 3.0F) * std::cos((phi + 4.0F * M_PI) / 3.0F)) -
      (b / (3.0F * a));

  if (t <= 0.0F) {
    throw std::runtime_error("computed flight time is not positive");
  }

  const float h = calculate_h(data, t);

  const float dx = data.targetX - data.xd;
  const float dy = data.targetY - data.yd;
  const float horizontalDistance = std::sqrt((dx * dx) + (dy * dy));

  if (horizontalDistance == 0.0F) {
    throw std::runtime_error(
        "drone and target coordinates are identical, distance is zero");
  }

  float fireX = 0.0F;
  float fireY = 0.0F;

  if (h + data.accelerationPath > horizontalDistance) {
    const float maneuverX =
        data.targetX - dx * (h + data.accelerationPath) / horizontalDistance;

    const float maneuverY =
        data.targetY - dy * (h + data.accelerationPath) / horizontalDistance;

    const float maneuverDx = data.targetX - maneuverX;
    const float maneuverDy = data.targetY - maneuverY;

    const float maneuverDistance =
        std::sqrt((maneuverDx * maneuverDx) + (maneuverDy * maneuverDy));

    if (maneuverDistance == 0.0F) {
      throw std::runtime_error("maneuver distance is zero");
    }

    const float maneuverRatio = (maneuverDistance - h) / maneuverDistance;

    fireX = maneuverX + maneuverDx * maneuverRatio;
    fireY = maneuverY + maneuverDy * maneuverRatio;
  } else {
    const float ratio = (horizontalDistance - h) / horizontalDistance;

    fireX = data.xd + dx * ratio;
    fireY = data.yd + dy * ratio;
  }

  return {
      .fireX = fireX,
      .fireY = fireY,
      .flightTime = t,
      .horizontalDistance = horizontalDistance,
  };
}

void writeOutputFile(const std::string &filename, const float value1,
                     const float value2) {
  std::ofstream outFile(filename);

  if (!outFile) {
    throw std::runtime_error("cannot open output file: " + filename);
  }

  outFile << value1 << ' ' << value2 << '\n';
}