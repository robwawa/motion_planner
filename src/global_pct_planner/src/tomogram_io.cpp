#include "global_pct_planner/tomogram_io.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

namespace global_pct_planner {
namespace {

constexpr char kMagic[8] = {'G', 'P', 'C', 'T', 'M', '0', '1', '\0'};
constexpr uint32_t kVersion = 1;
constexpr uint32_t kEndianMarker = 0x01020304u;
constexpr uint32_t kChannels = 5;

template <typename T>
bool writeValue(std::ofstream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template <typename T>
bool readValue(std::ifstream& stream, T& value) {
  stream.read(reinterpret_cast<char*>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

bool writeArray(std::ofstream& stream, const std::vector<float>& values) {
  if (values.empty()) return true;
  stream.write(reinterpret_cast<const char*>(values.data()),
               static_cast<std::streamsize>(values.size() * sizeof(float)));
  return static_cast<bool>(stream);
}

bool readArray(std::ifstream& stream, std::vector<float>& values) {
  if (values.empty()) return true;
  stream.read(reinterpret_cast<char*>(values.data()),
              static_cast<std::streamsize>(values.size() * sizeof(float)));
  return static_cast<bool>(stream);
}

void setError(std::string* error, const std::string& message) {
  if (error) *error = message;
}

bool validData(const TomogramData& data, std::string* error) {
  if (data.resolution <= 0.0f || data.slice_dh <= 0.0f || data.layers == 0 ||
      data.rows == 0 || data.cols == 0) {
    setError(error, "invalid tomogram dimensions or resolution");
    return false;
  }
  if (static_cast<std::size_t>(data.layers) >
      std::numeric_limits<std::size_t>::max() / data.rows ||
      static_cast<std::size_t>(data.layers) * data.rows >
          std::numeric_limits<std::size_t>::max() / data.cols) {
    setError(error, "tomogram dimensions overflow host size");
    return false;
  }
  const std::size_t count = data.cellCount();
  if (data.traversability.size() != count ||
      data.traversability_grad_x.size() != count ||
      data.traversability_grad_y.size() != count ||
      data.ground_elevation.size() != count ||
      data.ceiling_elevation.size() != count) {
    setError(error, "tomogram array length does not match dimensions");
    return false;
  }
  return true;
}

}  // namespace

bool TomogramIO::save(const std::string& path, const TomogramData& data,
                      std::string* error) {
  if (!validData(data, error)) return false;

  const std::string temporary_path = path + ".tmp";
  std::ofstream stream(temporary_path.c_str(), std::ios::binary | std::ios::trunc);
  if (!stream) {
    setError(error, "cannot open temporary tomogram file: " + temporary_path);
    return false;
  }

  const uint32_t layers = data.layers;
  const uint32_t rows = data.rows;
  const uint32_t cols = data.cols;
  const uint64_t count = static_cast<uint64_t>(data.cellCount());
  const double resolution = data.resolution;
  const double center_x = data.center_x;
  const double center_y = data.center_y;
  const double slice_h0 = data.slice_h0;
  const double slice_dh = data.slice_dh;

  bool ok = true;
  stream.write(kMagic, sizeof(kMagic));
  ok = ok && writeValue(stream, kVersion) && writeValue(stream, kEndianMarker) &&
       writeValue(stream, layers) && writeValue(stream, rows) &&
       writeValue(stream, cols) && writeValue(stream, kChannels) &&
       writeValue(stream, resolution) && writeValue(stream, center_x) &&
       writeValue(stream, center_y) && writeValue(stream, slice_h0) &&
       writeValue(stream, slice_dh) && writeValue(stream, count);
  ok = ok && writeArray(stream, data.traversability) &&
       writeArray(stream, data.traversability_grad_x) &&
       writeArray(stream, data.traversability_grad_y) &&
       writeArray(stream, data.ground_elevation) &&
       writeArray(stream, data.ceiling_elevation);
  stream.flush();
  ok = ok && static_cast<bool>(stream);
  stream.close();
  if (!ok) {
    std::remove(temporary_path.c_str());
    setError(error, "failed while writing tomogram file: " + path);
    return false;
  }
  if (std::rename(temporary_path.c_str(), path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    std::ostringstream message;
    message << "cannot commit tomogram file " << path << ": "
            << std::strerror(errno);
    setError(error, message.str());
    return false;
  }
  return true;
}

bool TomogramIO::load(const std::string& path, TomogramData& data,
                      std::string* error) {
  std::ifstream stream(path.c_str(), std::ios::binary);
  if (!stream) {
    setError(error, "cannot open tomogram file: " + path);
    return false;
  }

  char magic[sizeof(kMagic)] = {};
  uint32_t version = 0;
  uint32_t endian = 0;
  uint32_t layers = 0;
  uint32_t rows = 0;
  uint32_t cols = 0;
  uint32_t channels = 0;
  double resolution = 0.0;
  double center_x = 0.0;
  double center_y = 0.0;
  double slice_h0 = 0.0;
  double slice_dh = 0.0;
  uint64_t count = 0;
  bool ok = static_cast<bool>(stream.read(magic, sizeof(magic))) &&
            readValue(stream, version) && readValue(stream, endian) &&
            readValue(stream, layers) && readValue(stream, rows) &&
            readValue(stream, cols) && readValue(stream, channels) &&
            readValue(stream, resolution) && readValue(stream, center_x) &&
            readValue(stream, center_y) && readValue(stream, slice_h0) &&
            readValue(stream, slice_dh) && readValue(stream, count);
  if (!ok || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0 ||
      version != kVersion || endian != kEndianMarker || channels != kChannels) {
    setError(error, "unsupported or corrupt .pctm header: " + path);
    return false;
  }
  const uint64_t layer_row_count = static_cast<uint64_t>(layers) * rows;
  const bool dimension_overflow =
      (rows != 0 && static_cast<uint64_t>(layers) >
                         std::numeric_limits<uint64_t>::max() / rows) ||
      (layer_row_count != 0 && static_cast<uint64_t>(cols) >
                                   std::numeric_limits<uint64_t>::max() /
                                       layer_row_count);
  const uint64_t expected_count =
      dimension_overflow ? 0 : layer_row_count * static_cast<uint64_t>(cols);
  if (layers == 0 || rows == 0 || cols == 0 || resolution <= 0.0 ||
      slice_dh <= 0.0 || dimension_overflow || count != expected_count ||
      count > static_cast<uint64_t>(std::numeric_limits<std::size_t>::max())) {
    setError(error, "invalid .pctm dimensions: " + path);
    return false;
  }

  TomogramData loaded;
  loaded.resolution = static_cast<float>(resolution);
  loaded.center_x = static_cast<float>(center_x);
  loaded.center_y = static_cast<float>(center_y);
  loaded.slice_h0 = static_cast<float>(slice_h0);
  loaded.slice_dh = static_cast<float>(slice_dh);
  loaded.layers = layers;
  loaded.rows = rows;
  loaded.cols = cols;
  const std::size_t size = static_cast<std::size_t>(count);
  loaded.traversability.resize(size);
  loaded.traversability_grad_x.resize(size);
  loaded.traversability_grad_y.resize(size);
  loaded.ground_elevation.resize(size);
  loaded.ceiling_elevation.resize(size);
  ok = readArray(stream, loaded.traversability) &&
       readArray(stream, loaded.traversability_grad_x) &&
       readArray(stream, loaded.traversability_grad_y) &&
       readArray(stream, loaded.ground_elevation) &&
       readArray(stream, loaded.ceiling_elevation);
  if (!ok) {
    setError(error, "truncated .pctm payload: " + path);
    return false;
  }
  data = std::move(loaded);
  return true;
}

}  // namespace global_pct_planner
