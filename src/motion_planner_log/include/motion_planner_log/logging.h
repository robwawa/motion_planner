#ifndef MOTION_PLANNER_LOG_LOGGING_H_
#define MOTION_PLANNER_LOG_LOGGING_H_

#include <glog/logging.h>
#include <ros/time.h>

#include <atomic>
#include <sstream>
#include <string>

namespace motion_planner_log {

// Must be called once after ros::init() and before the first project log.
void initialize(const std::string& module_name, const char* argv0 = nullptr);

// printf-style entry point used by the project macros.
void log_printf(google::LogSeverity severity, const char* file, int line,
                const char* function, const char* format, ...);

// Stream-style entry point for code that previously used ROS_*_STREAM macros.
void log_stream(google::LogSeverity severity, const char* file, int line,
                const char* function, const std::string& message);

bool is_initialized();

}  // namespace motion_planner_log

#define MOTION_PLANNER_LOG_INFO(...) \
  ::motion_planner_log::log_printf(google::GLOG_INFO, __FILE__, __LINE__, \
                                   __FUNCTION__, __VA_ARGS__)
#define MOTION_PLANNER_LOG_WARN(...) \
  ::motion_planner_log::log_printf(google::GLOG_WARNING, __FILE__, __LINE__, \
                                   __FUNCTION__, __VA_ARGS__)
#define MOTION_PLANNER_LOG_ERROR(...) \
  ::motion_planner_log::log_printf(google::GLOG_ERROR, __FILE__, __LINE__, \
                                   __FUNCTION__, __VA_ARGS__)
#define MOTION_PLANNER_LOG_FATAL(...) \
  ::motion_planner_log::log_printf(google::GLOG_FATAL, __FILE__, __LINE__, \
                                   __FUNCTION__, __VA_ARGS__)
#define MOTION_PLANNER_LOG_DEBUG(...) \
  do { if (FLAGS_v >= 1) ::motion_planner_log::log_printf(google::GLOG_INFO, __FILE__, __LINE__, \
                                   __FUNCTION__, __VA_ARGS__); } while (false)
#define MOTION_PLANNER_LOG_DEBUG_STREAM(expr) \
  do { if (FLAGS_v >= 1) { std::ostringstream _motion_planner_log_stream; \
       _motion_planner_log_stream << expr; \
       ::motion_planner_log::log_stream(google::GLOG_INFO, __FILE__, __LINE__, \
                                        __FUNCTION__, _motion_planner_log_stream.str()); } } while (false)

#define MOTION_PLANNER_LOG_INFO_STREAM(expr) \
  do { std::ostringstream _motion_planner_log_stream; \
       _motion_planner_log_stream << expr; \
       ::motion_planner_log::log_stream(google::GLOG_INFO, __FILE__, __LINE__, \
                                        __FUNCTION__, _motion_planner_log_stream.str()); } while (false)
#define MOTION_PLANNER_LOG_WARN_STREAM(expr) \
  do { std::ostringstream _motion_planner_log_stream; \
       _motion_planner_log_stream << expr; \
       ::motion_planner_log::log_stream(google::GLOG_WARNING, __FILE__, __LINE__, \
                                        __FUNCTION__, _motion_planner_log_stream.str()); } while (false)
#define MOTION_PLANNER_LOG_ERROR_STREAM(expr) \
  do { std::ostringstream _motion_planner_log_stream; \
       _motion_planner_log_stream << expr; \
       ::motion_planner_log::log_stream(google::GLOG_ERROR, __FILE__, __LINE__, \
                                        __FUNCTION__, _motion_planner_log_stream.str()); } while (false)

#define MOTION_PLANNER_LOG_INFO_ONCE(...) \
  do { static std::atomic<bool> _motion_planner_log_once(false); bool _motion_planner_log_expected = false; \
       if (_motion_planner_log_once.compare_exchange_strong(_motion_planner_log_expected, true)) { \
         MOTION_PLANNER_LOG_INFO(__VA_ARGS__); } } while (false)
#define MOTION_PLANNER_LOG_WARN_ONCE(...) \
  do { static std::atomic<bool> _motion_planner_log_once(false); bool _motion_planner_log_expected = false; \
       if (_motion_planner_log_once.compare_exchange_strong(_motion_planner_log_expected, true)) { \
         MOTION_PLANNER_LOG_WARN(__VA_ARGS__); } } while (false)
#define MOTION_PLANNER_LOG_ERROR_ONCE(...) \
  do { static std::atomic<bool> _motion_planner_log_once(false); bool _motion_planner_log_expected = false; \
       if (_motion_planner_log_once.compare_exchange_strong(_motion_planner_log_expected, true)) { \
         MOTION_PLANNER_LOG_ERROR(__VA_ARGS__); } } while (false)

#define MOTION_PLANNER_LOG_INFO_IF(cond, ...) \
  do { if (cond) MOTION_PLANNER_LOG_INFO(__VA_ARGS__); } while (false)
#define MOTION_PLANNER_LOG_WARN_IF(cond, ...) \
  do { if (cond) MOTION_PLANNER_LOG_WARN(__VA_ARGS__); } while (false)
#define MOTION_PLANNER_LOG_ERROR_IF(cond, ...) \
  do { if (cond) MOTION_PLANNER_LOG_ERROR(__VA_ARGS__); } while (false)

#define MOTION_PLANNER_LOG_INFO_THROTTLE(period, ...) \
  do { static double _motion_planner_log_last = -1.0; \
       const double _motion_planner_log_now = ros::Time::now().toSec(); \
       if (_motion_planner_log_last < 0.0 || _motion_planner_log_now < _motion_planner_log_last || \
           _motion_planner_log_now - _motion_planner_log_last >= (period)) { \
         _motion_planner_log_last = _motion_planner_log_now; MOTION_PLANNER_LOG_INFO(__VA_ARGS__); } } while (false)
#define MOTION_PLANNER_LOG_WARN_THROTTLE(period, ...) \
  do { static double _motion_planner_log_last = -1.0; \
       const double _motion_planner_log_now = ros::Time::now().toSec(); \
       if (_motion_planner_log_last < 0.0 || _motion_planner_log_now < _motion_planner_log_last || \
           _motion_planner_log_now - _motion_planner_log_last >= (period)) { \
         _motion_planner_log_last = _motion_planner_log_now; MOTION_PLANNER_LOG_WARN(__VA_ARGS__); } } while (false)
#define MOTION_PLANNER_LOG_ERROR_THROTTLE(period, ...) \
  do { static double _motion_planner_log_last = -1.0; \
       const double _motion_planner_log_now = ros::Time::now().toSec(); \
       if (_motion_planner_log_last < 0.0 || _motion_planner_log_now < _motion_planner_log_last || \
           _motion_planner_log_now - _motion_planner_log_last >= (period)) { \
         _motion_planner_log_last = _motion_planner_log_now; MOTION_PLANNER_LOG_ERROR(__VA_ARGS__); } } while (false)

#define MOTION_PLANNER_LOG_DEBUG_THROTTLE(period, ...) \
  do { static double _motion_planner_log_last = -1.0; \
       const double _motion_planner_log_now = ros::Time::now().toSec(); \
       if (_motion_planner_log_last < 0.0 || _motion_planner_log_now < _motion_planner_log_last || \
           _motion_planner_log_now - _motion_planner_log_last >= (period)) { \
         _motion_planner_log_last = _motion_planner_log_now; MOTION_PLANNER_LOG_DEBUG(__VA_ARGS__); } } while (false)

#endif  // MOTION_PLANNER_LOG_LOGGING_H_
