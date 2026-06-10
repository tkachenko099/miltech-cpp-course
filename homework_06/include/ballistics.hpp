#pragma once

#include <string>

constexpr float G = 9.81;

struct AmmoParams {
  float m;
  float d;
  float l;
};

struct InputData {
  float xd;
  float yd;
  float zd;

  float targetX;
  float targetY;

  float attackSpeed;
  float accelerationPath;

  std::string ammoType;

  float ammoM;
  float ammoD;
  float ammoL;
};

struct DropSolution {
  float fireX;
  float fireY;

  float flightTime;
  float horizontalDistance;
};

AmmoParams chooseAmmoParams(const std::string &typeStr);

InputData readInputFile(const std::string &filename);

float calculate_h(const InputData &data, float t);

DropSolution computeDropSolution(const InputData &data);

void writeOutputFile(const std::string &filename, float value1, float value2);