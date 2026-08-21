#ifndef PLAN_ENV_PCT_TERRAIN_MAP_H_
#define PLAN_ENV_PCT_TERRAIN_MAP_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <pct_planner/PctTerrainMap.h>

class PctTerrainMap {
public:
  enum class QueryStatus {
    kFree,
    kOutOfMap,
    kNoElevation,
    kCostTooHigh,
    kHeightMismatch,
    kInvalidMap
  };

  struct Cell {
    int layer{-1};
    float traversability{0.0f};
    float ground_z{0.0f};
  };

  static const char* queryStatusName(QueryStatus status);

  bool setFromMessage(const pct_planner::PctTerrainMap& msg, std::string& error);
  bool valid() const { return valid_; }
  const std::string& frameId() const { return frame_id_; }

  // Mirrors TomogramPlanner.pos2idx/select_layer exactly: x is rounded into
  // row and y is rounded into col, with center-relative half dimensions.
  QueryStatus queryTraversableLayer(double x, double y, double desired_ground_z,
                                    double max_height_error,
                                    double traversability_threshold,
                                    Cell& result) const;

private:
  std::size_t address(int layer, int row, int col) const;

  bool valid_{false};
  double resolution_{0.0};
  double center_x_{0.0};
  double center_y_{0.0};
  int rows_{0};
  int cols_{0};
  int layers_{0};
  std::string frame_id_;
  std::vector<float> traversability_;
  std::vector<float> ground_elevation_;
  std::vector<uint8_t> elevation_valid_;
};

#endif
