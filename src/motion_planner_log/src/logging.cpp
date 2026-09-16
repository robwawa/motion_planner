#include "motion_planner_log/logging.h"

#include <ros/console.h>
#include <ros/init.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

namespace motion_planner_log {
namespace {

constexpr const char* kFunctionMarker = "\x1fMOTION_PLANNER_FUNCTION\x1f";
constexpr const char* kMessageMarker = "\x1fMOTION_PLANNER_MESSAGE\x1f";

struct State {
  std::mutex mutex;
  std::string module;
  std::string root;
  std::unique_ptr<google::LogSink> sink;
  std::ofstream sink_file;
  std::string current_file;
  bool initialized = false;
  bool to_console = false;
  bool to_rosout = false;
  std::size_t max_bytes = 0;
};

State& state() {
  static State value;
  return value;
}

bool env_bool(const char* name, bool fallback) {
  const char* value = std::getenv(name);
  if (!value) return fallback;
  return std::string(value) == "1" || std::string(value) == "true" ||
         std::string(value) == "TRUE" || std::string(value) == "yes";
}

std::string env_string(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value && *value ? std::string(value) : fallback;
}

std::size_t env_size(const char* name) {
  const char* value = std::getenv(name);
  if (!value || !*value) return 0;
  char* end = nullptr;
  const unsigned long long megabytes = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0') return 0;
  return static_cast<std::size_t>(megabytes) * 1024U * 1024U;
}

void mkdir_recursive(const std::string& path) {
  if (path.empty() || path == "/") return;
  std::string current;
  if (path.front() == '/') current = "/";
  std::size_t start = path.front() == '/' ? 1 : 0;
  while (start <= path.size()) {
    const std::size_t slash = path.find('/', start);
    const std::string part = path.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') current += '/';
      current += part;
      ::mkdir(current.c_str(), 0755);
    }
    if (slash == std::string::npos) break;
    start = slash + 1;
  }
}

std::string home_dir() {
  const char* home = std::getenv("HOME");
  return home && *home ? std::string(home) : std::string("/tmp");
}

std::string default_log_dir() {
#ifdef MOTION_PLANNER_DEFAULT_LOG_DIR
  return MOTION_PLANNER_DEFAULT_LOG_DIR;
#else
  return "log";
#endif
}

std::string date_string(const tm* time) {
  char buffer[32] = {};
  std::strftime(buffer, sizeof(buffer), "%Y%m%d", time);
  return buffer;
}

std::string timestamp_string() {
  timeval now = {};
  gettimeofday(&now, nullptr);
  tm local = {};
  localtime_r(&now.tv_sec, &local);
  char buffer[64] = {};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local);
  std::ostringstream result;
  result << buffer << '.' << std::setfill('0') << std::setw(6) << now.tv_usec;
  return result.str();
}

const char* severity_name(google::LogSeverity severity) {
  switch (severity) {
    case google::GLOG_INFO: return "INFO";
    case google::GLOG_WARNING: return "WARN";
    case google::GLOG_ERROR: return "ERROR";
    case google::GLOG_FATAL: return "FATAL";
    default: return "INFO";
  }
}

class FileSink final : public google::LogSink {
 public:
  explicit FileSink(State* state) : state_(state) {}

  void send(google::LogSeverity severity, const char* full_filename,
            const char* base_filename, int line, const tm* tm_time,
            const char* message, std::size_t message_len) override {
    std::string function;
    std::string body(message, message_len);
    const std::string function_marker(kFunctionMarker);
    const std::string message_marker(kMessageMarker);
    const std::size_t function_begin = body.find(function_marker);
    const std::size_t message_begin = body.find(message_marker);
    if (function_begin != std::string::npos && message_begin != std::string::npos &&
        message_begin > function_begin) {
      function = body.substr(function_begin + function_marker.size(),
                             message_begin - function_begin - function_marker.size());
      body = body.substr(message_begin + message_marker.size());
    }

    const char* source = base_filename && *base_filename ? base_filename : full_filename;
    std::string source_name = source ? source : "unknown";
    const std::size_t slash = source_name.find_last_of('/');
    if (slash != std::string::npos) source_name = source_name.substr(slash + 1);

    std::ostringstream line_stream;
    line_stream << '[' << timestamp_string() << "]  ["
                 << severity_name(severity) << "] [" << state_->module << "] "
                 << source_name;
    if (!function.empty()) line_stream << '(' << function << ')';
    line_stream << ':' << line << ' ' << body << '\n';
    const std::string output = line_stream.str();

    std::lock_guard<std::mutex> lock(state_->mutex);
    const std::string current_date = date_string(tm_time);
    const std::string expected_file = state_->root + "/" + state_->module + "/" + current_date + ".log";
    if (expected_file != state_->current_file) {
      state_->sink_file.close();
      state_->current_file = expected_file;
      state_->sink_file.open(state_->current_file, std::ios::out | std::ios::app);
    }
    if (state_->to_console) {
      std::fwrite(output.data(), 1, output.size(), stderr);
      std::fflush(stderr);
    }
    if (state_->sink_file.is_open()) {
      if (state_->max_bytes > 0 && state_->sink_file.tellp() >= static_cast<std::streamoff>(state_->max_bytes)) {
        state_->sink_file.close();
        const std::string rotated = state_->current_file + ".1";
        std::remove(rotated.c_str());
        std::rename(state_->current_file.c_str(), rotated.c_str());
        state_->sink_file.open(state_->current_file, std::ios::out | std::ios::trunc);
      }
      state_->sink_file << output;
      state_->sink_file.flush();
    }
    if (state_->to_rosout && severity >= google::GLOG_WARNING && ros::isInitialized()) {
      ros::console::initialize();
      ros::console::Level ros_level = ros::console::levels::Warn;
      if (severity == google::GLOG_ERROR) ros_level = ros::console::levels::Error;
      if (severity == google::GLOG_FATAL) ros_level = ros::console::levels::Fatal;
      ros::console::print(nullptr, nullptr, ros_level, source, line,
                          function.empty() ? "" : function.c_str(), "%s", body.c_str());
    }
  }

 private:
  State* state_;
};

}  // namespace

void initialize(const std::string& module_name, const char* argv0) {
  State& base = state();
  std::lock_guard<std::mutex> lock(base.mutex);
  if (base.initialized) return;

  base.module = module_name;
  base.root = env_string("MOTION_PLANNER_LOG_DIR", default_log_dir());
  base.to_console = env_bool("MOTION_PLANNER_LOG_TO_CONSOLE", false);
  base.to_rosout = env_bool("MOTION_PLANNER_LOG_ROSOUT", false);
  base.max_bytes = env_size("MOTION_PLANNER_LOG_MAX_MB");

  const std::string module_dir = base.root + "/" + module_name;
  mkdir_recursive(module_dir);
  const std::time_t now = std::time(nullptr);
  const tm* local = std::localtime(&now);
  const std::string file = module_dir + "/" + date_string(local) + ".log";

  google::InitGoogleLogging(argv0 ? argv0 : module_name.c_str());
  google::SetLogDestination(google::GLOG_INFO, "");
  google::SetLogDestination(google::GLOG_WARNING, "");
  google::SetLogDestination(google::GLOG_ERROR, "");
  google::SetLogDestination(google::GLOG_FATAL, "");

  base.current_file = file;
  base.sink_file.open(file, std::ios::out | std::ios::app);
  if (!base.sink_file.is_open()) {
    base.root = home_dir() + "/.ros/log/motion_planner";
    const std::string fallback_dir = base.root + "/" + module_name;
    mkdir_recursive(fallback_dir);
    base.current_file = fallback_dir + "/" + date_string(local) + ".log";
    base.sink_file.open(base.current_file, std::ios::out | std::ios::app);
  }
  base.sink = std::make_unique<FileSink>(&base);
  google::AddLogSink(base.sink.get());
  const std::string level = env_string("MOTION_PLANNER_LOG_LEVEL", "INFO");
  if (level == "DEBUG") {
    FLAGS_minloglevel = 0;
    FLAGS_v = 1;
  }
  else if (level == "WARN" || level == "WARNING") FLAGS_minloglevel = 1;
  else if (level == "ERROR") FLAGS_minloglevel = 2;
  else {
    FLAGS_minloglevel = 0;
    FLAGS_v = 0;
  }
  base.initialized = true;
}

bool is_initialized() { return state().initialized; }

void log_printf(google::LogSeverity severity, const char* file, int line,
                const char* function, const char* format, ...) {
  char buffer[4096] = {};
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  google::LogMessage(file, line, severity).stream()
      << kFunctionMarker << (function ? function : "") << kMessageMarker << buffer;
}

void log_stream(google::LogSeverity severity, const char* file, int line,
                const char* function, const std::string& message) {
  google::LogMessage(file, line, severity).stream()
      << kFunctionMarker << (function ? function : "") << kMessageMarker << message;
}

}  // namespace motion_planner_log
