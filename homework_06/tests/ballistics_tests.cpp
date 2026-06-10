#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint) {
  const InputData input{.xd = 100.0F,
                        .yd = 100.0F,
                        .zd = 100.0F,

                        .targetX = 200.0F,
                        .targetY = 200.0F,

                        .attackSpeed = 10.0F,
                        .accelerationPath = 10.0F,

                        .ammoType = "VOG-17",

                        .ammoM = 0.35F,
                        .ammoD = 0.07F,
                        .ammoL = 0.0F};

  const DropSolution solution = computeDropSolution(input);

  EXPECT_NEAR(solution.fireX, 173.759F, 0.01F);
  EXPECT_NEAR(solution.fireY, 173.759F, 0.01F);

  EXPECT_GT(solution.flightTime, 0.0F);
}

TEST(Ballistics, ThrowsOnUnknownAmmoType) {
  EXPECT_THROW(chooseAmmoParams("UNKNOWN-AMMO"), std::runtime_error);
}

TEST(Ballistics, ThrowsOnZeroAltitude) {
  const InputData input{.xd = 0.0F,
                        .yd = 0.0F,
                        .zd = 0.0F,

                        .targetX = 100.0F,
                        .targetY = 100.0F,

                        .attackSpeed = 10.0F,
                        .accelerationPath = 5.0F,

                        .ammoType = "VOG-17",

                        .ammoM = 0.35F,
                        .ammoD = 0.07F,
                        .ammoL = 0.0F};

  EXPECT_THROW(computeDropSolution(input), std::runtime_error);
}

TEST(Ballistics, GlidingAmmoProducesPositiveFlightTime) {
  const InputData input{.xd = 0.0F,
                        .yd = 0.0F,
                        .zd = 120.0F,

                        .targetX = 300.0F,
                        .targetY = 300.0F,

                        .attackSpeed = 20.0F,
                        .accelerationPath = 15.0F,

                        .ammoType = "GLIDING-VOG",

                        .ammoM = 0.45F,
                        .ammoD = 0.10F,
                        .ammoL = 1.0F};

  const DropSolution solution = computeDropSolution(input);

  EXPECT_GT(solution.flightTime, 0.0F);
}