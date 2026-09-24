#include "motion_planner_log/logging.h"

#include <ros/console.h>
#include <ros/init.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>

namespace motion_planner_log {
namespace {

thread_local const char* active_function = nullptr;

class ScopedFunctionContext {
 public:
  explicit ScopedFunctionContext(const char* function)
      : previous_(active_function) {
    active_function = function;
  }

  ~ScopedFunctionContext() { active_function = previous_; }

 private:
  const char* previous_;
};

struct State {
  std::mutex mutex;
  std::string package;
  std::string module;
  std::string root;
  std::unique_ptr<google::LogSink> sink;
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
    const std::string function = active_function ? active_function : "";
    const std::string body(message, message_len);

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
    const std::string package_dir = state_->root + "/" + state_->package;
    const std::string expected_file = package_dir + "/" + current_date + ".log";
    const std::string lock_file = package_dir + "/.lock";
    state_->current_file = expected_file;

    const int lock_fd = ::open(lock_file.c_str(), O_RDWR | O_CREAT, 0644);
    if (lock_fd >= 0) {
      ::flock(lock_fd, LOCK_EX);
    }

    if (state_->to_console) {
      std::fwrite(output.data(), 1, output.size(), stderr);
      std::fflush(stderr);
    }

    struct stat file_stat = {};
    if (state_->max_bytes > 0 && ::stat(expected_file.c_str(), &file_stat) == 0 &&
        static_cast<std::size_t>(file_stat.st_size) >= state_->max_bytes) {
      const std::string rotated = expected_file + ".1";
      std::remove(rotated.c_str());
      std::rename(expected_file.c_str(), rotated.c_str());
    }

    const int output_fd = ::open(expected_file.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (output_fd >= 0) {
      const char* data = output.data();
      std::size_t remaining = output.size();
      while (remaining > 0) {
        const ssize_t written = ::write(output_fd, data, remaining);
        if (written <= 0) break;
        data += written;
        remaining -= static_cast<std::size_t>(written);
      }
      ::close(output_fd);
    }

    if (lock_fd >= 0) {
      ::flock(lock_fd, LOCK_UN);
      ::close(lock_fd);
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

void initialize(const std::string& package_name,
                const std::string& module_name,
                const char* argv0) {
  State& base = state();
  std::lock_guard<std::mutex> lock(base.mutex);
  if (base.initialized) return;

  base.package = package_name;
  base.module = module_name;
  base.root = env_string("MOTION_PLANNER_LOG_DIR", default_log_dir());
  base.to_console = env_bool("MOTION_PLANNER_LOG_TO_CONSOLE", false);
  base.to_rosout = env_bool("MOTION_PLANNER_LOG_ROSOUT", false);
  base.max_bytes = env_size("MOTION_PLANNER_LOG_MAX_MB");

  const std::string package_dir = base.root + "/" + package_name;
  mkdir_recursive(package_dir);
  const std::time_t now = std::time(nullptr);
  const tm* local = std::localtime(&now);
  const std::string file = package_dir + "/" + date_string(local) + ".log";

  google::InitGoogleLogging(argv0 ? argv0 : module_name.c_str());
  google::SetLogDestination(google::GLOG_INFO, "");
  google::SetLogDestination(google::GLOG_WARNING, "");
  google::SetLogDestination(google::GLOG_ERROR, "");
  google::SetLogDestination(google::GLOG_FATAL, "");

  base.current_file = file;
  const int probe_fd = ::open(file.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (probe_fd < 0) {
    base.root = home_dir() + "/.ros/log/motion_planner";
    const std::string fallback_dir = base.root + "/" + package_name;
    mkdir_recursive(fallback_dir);
    base.current_file = fallback_dir + "/" + date_string(local) + ".log";
  } else {
    ::close(probe_fd);
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

void initialize(const std::string& module_name, const char* argv0) {
  initialize(module_name, module_name, argv0);
}

bool is_initialized() { return state().initialized; }

void log_printf(google::LogSeverity severity, const char* file, int line,
                const char* function, const char* format, ...) {
  char buffer[4096] = {};
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  ScopedFunctionContext function_context(function);
  google::LogMessage(file, line, severity).stream() << buffer;
}

void log_stream(google::LogSeverity severity, const char* file, int line,
                const char* function, const std::string& message) {
  ScopedFunctionContext function_context(function);
  google::LogMessage(file, line, severity).stream() << message;
}

}  // namespace motion_planner_log
