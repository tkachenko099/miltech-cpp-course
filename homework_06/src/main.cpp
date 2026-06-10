#include "ballistics.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    const InputData data = readInputFile("input.txt");

    const DropSolution solution = computeDropSolution(data);

    std::cout << "Drop point:\n";
    std::cout << "x = " << solution.fireX << '\n';
    std::cout << "y = " << solution.fireY << '\n';

    writeOutputFile("output.txt", solution.fireX, solution.fireY);

  } catch (const std::exception &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }

  return 0;
}