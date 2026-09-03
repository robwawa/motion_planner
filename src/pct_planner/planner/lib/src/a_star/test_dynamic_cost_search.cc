#include <Eigen/Core>

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "a_star/a_star_search.h"

int main() {
  constexpr int kRows = 1;
  constexpr int kCols = 3;
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(kRows, kCols);
  Eigen::MatrixXd height = Eigen::MatrixXd::Zero(kRows, kCols);
  Eigen::MatrixXd elevation = Eigen::MatrixXd::Zero(kRows, kCols);
  cost(0, 1) = 100.0;
  elevation(0, 1) = 2.0;  // Static gateway exception permits this cell.

  Astar astar;
  astar.Init(20.0, 1, 1.0, 0.2, cost, height, elevation);
  const Eigen::Vector3i start(0, 0, 0);
  const Eigen::Vector3i goal(0, 2, 0);

  if (!astar.Search(start, goal)) {
    std::cerr << "static gateway route should be reachable" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<uint8_t> dynamic_cost(kRows * kCols, 0);
  dynamic_cost[1] = 100;
  astar.SetDynamicCostMap(dynamic_cost.data(), dynamic_cost.size(), 100);
  if (astar.Search(start, goal)) {
    std::cerr << "dynamic lethal cost must override gateway exception"
              << std::endl;
    return EXIT_FAILURE;
  }

  dynamic_cost[1] = 10;
  astar.SetDynamicCostMap(dynamic_cost.data(), dynamic_cost.size(), 100);
  if (!astar.Search(start, goal)) {
    std::cerr << "soft dynamic cost should remain traversable" << std::endl;
    return EXIT_FAILURE;
  }

  astar.ClearDynamicCostMap();
  if (!astar.Search(start, goal)) {
    std::cerr << "clearing dynamic cost must restore static behavior"
              << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
