/**
 * @file main.cpp
 * @brief Computes drop point coordinates and writes them to output.txt.
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

constexpr float PI = 3.14159265358979323846f;
constexpr float G  = 9.81f;

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

AmmoParams chooseAmmoParams(const std::string& typeStr);
InputData readInputFile(const std::string& filename);
void writeOutputFile(const std::string& filename, float value1, float value2);

int main() {
    InputData data{};

    try {
        data = readInputFile("input.txt");

        std::cout << "xd = " << data.xd << '\n';
        std::cout << "yd = " << data.yd << '\n';
        std::cout << "zd = " << data.zd << '\n';
        std::cout << "targetX = " << data.targetX << '\n';
        std::cout << "targetY = " << data.targetY << '\n';
        std::cout << "attackSpeed = " << data.attackSpeed << '\n';
        std::cout << "accelerationPath = " << data.accelerationPath << '\n';
        std::cout << "ammoType = " << data.ammoType << '\n';
        std::cout << "m = " << data.ammoM << '\n';
        std::cout << "d = " << data.ammoD << '\n';
        std::cout << "l = " << data.ammoL << '\n';
    } catch (const std::exception& ex) {
        std::cerr << "Error while reading input.txt: " << ex.what() << '\n';
        return 1;
    }

    if (data.zd > 100.0f) {
        std::cout << "Warning: altitude is larger than 100 m, calculations might be incorrect.\n";
    }

    const float a = data.ammoD * G * data.ammoM
                  - 2.0f * data.ammoD * data.ammoD * data.ammoL * data.attackSpeed;

    const float b = -3.0f * G * data.ammoM * data.ammoM
                  + 3.0f * data.ammoD * data.ammoL * data.ammoM * data.attackSpeed;

    const float c = 6.0f * data.ammoM * data.ammoM * data.zd;

    std::cout << "a = " << a << '\n';
    std::cout << "b = " << b << '\n';
    std::cout << "c = " << c << '\n';

    if (a == 0.0f) {
        std::cerr << "Error: coefficient 'a' is zero, cannot continue calculation.\n";
        return 1;
    }

    const float p = -(std::pow(b, 2.0f) / (3.0f * std::pow(a, 2.0f)));
    const float q = (2.0f * std::pow(b, 3.0f)) / (27.0f * std::pow(a, 3.0f)) + c / a;

    std::cout << "p = " << p << '\n';
    std::cout << "q = " << q << '\n';

    if (p >= 0.0f) {
        std::cerr << "Error: p must be negative for the current formula.\n";
        return 1;
    }

    float acosArg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
    if (acosArg < -1.0f) acosArg = -1.0f;
    if (acosArg >  1.0f) acosArg =  1.0f;

    const float phi = std::acos(acosArg);
    const float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * PI) / 3.0f)
                  - b / (3.0f * a);

    const float h =
          data.attackSpeed * t
        - std::pow(t, 2.0f) * data.ammoD * data.attackSpeed / (2.0f * data.ammoM)
        + std::pow(t, 3.0f) *
          (
              6.0f * data.ammoD * G * data.ammoL * data.ammoM
            - 6.0f * std::pow(data.ammoD, 2.0f) * (std::pow(data.ammoL, 2.0f) - 1.0f) * data.attackSpeed
          ) / (36.0f * std::pow(data.ammoM, 2.0f))
        + std::pow(t, 4.0f) *
          (
              -6.0f * std::pow(data.ammoD, 2.0f) * G * data.ammoL *
              (1.0f + std::pow(data.ammoL, 2.0f) + std::pow(data.ammoL, 4.0f)) * data.ammoM
            + 3.0f * std::pow(data.ammoD, 3.0f) * std::pow(data.ammoL, 2.0f) *
              (1.0f + std::pow(data.ammoL, 2.0f)) * data.attackSpeed
            + 6.0f * std::pow(data.ammoD, 3.0f) * std::pow(data.ammoL, 4.0f) *
              (1.0f + std::pow(data.ammoL, 2.0f) * data.attackSpeed)
          ) / (36.0f * std::pow(1.0f + std::pow(data.ammoL, 2.0f), 2.0f) * std::pow(data.ammoM, 3.0f))
        + std::pow(t, 5.0f) *
          (
              3.0f * std::pow(data.ammoD, 3.0f) * G * std::pow(data.ammoL, 3.0f) * data.ammoM
            - 3.0f * std::pow(data.ammoD, 4.0f) * std::pow(data.ammoL, 2.0f) *
              (1.0f + std::pow(data.ammoL, 2.0f)) * data.attackSpeed
          ) / (36.0f * (1.0f + std::pow(data.ammoL, 2.0f)) * std::pow(data.ammoM, 4.0f));

    const float dx = data.targetX - data.xd;
    const float dy = data.targetY - data.yd;
    const float D = std::sqrt(dx * dx + dy * dy);

    std::cout << "phi = " << phi << '\n';
    std::cout << "t = " << t << '\n';
    std::cout << "h = " << h << '\n';
    std::cout << "D = " << D << '\n';

    if (D == 0.0f) {
        std::cerr << "Error: drone and target coordinates are identical, D = 0.\n";
        return 1;
    }

    float fireX = 0.0f;
    float fireY = 0.0f;
    const float ratio = (D - h) / D;

    if (h + data.accelerationPath > D) {
        std::cout << "The target is too close, additional maneuver is needed.\n";

        const float xdm = data.targetX - dx * (h + data.accelerationPath) / D;
        const float ydm = data.targetY - dy * (h + data.accelerationPath) / D;

        std::cout << "Coordinates after maneuver: x = " << xdm << "; y = " << ydm << '\n';

        fireX = xdm + (data.targetX - xdm) * ratio;
        fireY = ydm + (data.targetY - ydm) * ratio;
    } else {
        fireX = data.xd + dx * ratio;
        fireY = data.yd + dy * ratio;
    }

    std::cout << "Coordinates of drop point: x = " << fireX << "; y = " << fireY << '\n';

    try {
        writeOutputFile("output.txt", fireX, fireY);
        std::cout << "Data was successfully written to output.txt\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error while writing output.txt: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}

AmmoParams chooseAmmoParams(const std::string& typeStr) {
    if (typeStr == "VOG-17")      return {0.35f, 0.07f, 0.0f};
    if (typeStr == "M67")         return {0.60f, 0.10f, 0.0f};
    if (typeStr == "RKG-3")       return {1.20f, 0.10f, 0.0f};
    if (typeStr == "GLIDING-VOG") return {0.45f, 0.10f, 1.0f};
    if (typeStr == "GLIDING-RKG") return {1.40f, 0.10f, 1.0f};

    throw std::runtime_error("Unknown ammo type: " + typeStr);
}

InputData readInputFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("cannot open file");
    }

    InputData data{};

    if (!(file >> data.xd
               >> data.yd
               >> data.zd
               >> data.targetX
               >> data.targetY
               >> data.attackSpeed
               >> data.accelerationPath)) {
        throw std::runtime_error("not enough numeric data in file");
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

void writeOutputFile(const std::string& filename, float value1, float value2) {
    std::ofstream outFile(filename);
    if (!outFile) {
        throw std::runtime_error("cannot open output file");
    }

    outFile << value1 << ' ' << value2 << '\n';
}