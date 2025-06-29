#pragma once

#include <string>
#include <syslog.h>
#include <cstdarg>

#define DEFAULT_SAMPLE_RATE (48000.0)

#define SYSLOG_DBG(fmt, ...) Logger::log(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define SYSLOG_INF(fmt, ...) Logger::log(LOG_INFO, fmt, ##__VA_ARGS__)
#define SYSLOG_WAR(fmt, ...) Logger::log(LOG_WARN, fmt, ##__VA_ARGS__)
#define SYSLOG_ERR(fmt, ...) Logger::log(LOG_ERR, fmt, ##__VA_ARGS__)

typedef enum ipcMessages {
  IPC_CREATE_INSTANCE = 0,
  IPC_CONNECT_PORTS,
  IPC_DISCONNECT_PORTS,
  IPC_UPDATE_CTRL_PARAM,
  IPC_DELETE_INSTANCE
} ipcMsg_t;

class Logger {
public:
  static void init(const std::string& appName = "MyApp");
  static void shutdown();
  static void log(int level, const char *fmt, ...);
};
