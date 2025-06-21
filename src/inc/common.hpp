#pragma once

#include <string>
#include <syslog.h>
#include <cstdarg>

#define DEFAULT_SAMPLE_RATE (48000.0)

#define SYSLOG_DBG(fmt, ...) Logger::log(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define SYSLOG_INF(fmt, ...) Logger::log(LOG_INFO, fmt, ##__VA_ARGS__)
#define SYSLOG_WAR(fmt, ...) Logger::log(LOG_WARN, fmt, ##__VA_ARGS__)
#define SYSLOG_ERR(fmt, ...) Logger::log(LOG_ERR, fmt, ##__VA_ARGS__)

class Logger {
public:
  static void init(const std::string& appName = "MyApp");
  static void shutdown();
  static void log(int level, const char *fmt, ...);
};
