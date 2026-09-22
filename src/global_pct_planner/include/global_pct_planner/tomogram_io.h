#pragma once

#include <string>

#include "global_pct_planner/tomogram.h"

namespace global_pct_planner {

class TomogramIO {
 public:
  static bool save(const std::string& path, const TomogramData& data,
                   std::string* error = nullptr);
  static bool load(const std::string& path, TomogramData& data,
                   std::string* error = nullptr);
};

}  // namespace global_pct_planner
