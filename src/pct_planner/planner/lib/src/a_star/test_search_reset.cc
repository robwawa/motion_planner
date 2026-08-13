#include <Eigen/Core>

#include <cstdlib>
#include <iostream>

#include "a_star/a_star_search.h"

int main() {
  constexpr int kRows = 5;
  constexpr int kCols = 5;
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(kRows, kCols);
  Eigen::MatrixXd height = Eigen::MatrixXd::Zero(kRows, kCols);
  Eigen::MatrixXd elevation = Eigen::MatrixXd::Zero(kRows, kCols);

  // Split the map in two. The first search visits the upper component but
  // cannot reach the lower component.
  cost.row(2).setConstant(100.0);

  Astar astar;
  astar.Init(20.0, 1, 1.0, 0.2, cost, height, elevation);

  if (astar.Search(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(0, 4, 4))) {
    std::cerr << "expected disconnected search to fail" << std::endl;
    return EXIT_FAILURE;
  }

  // This endpoint is reachable in the previously visited component. Without
  // resetting every node after the failed search, stale g values prevent it
  // from being enqueued again.
  if (!astar.Search(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(0, 4, 1))) {
    std::cerr << "reachable search failed after a prior failed query" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
