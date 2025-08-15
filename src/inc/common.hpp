#pragma once

#include <string>
#include <syslog.h>
#include <cstdarg>


#define SYSLOG_DBG(fmt, ...) Logger::log(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define SYSLOG_INF(fmt, ...) Logger::log(LOG_INFO, fmt, ##__VA_ARGS__)
#define SYSLOG_WAR(fmt, ...) Logger::log(LOG_WARN, fmt, ##__VA_ARGS__)
#define SYSLOG_ERR(fmt, ...) Logger::log(LOG_ERR, fmt, ##__VA_ARGS__)

typedef enum ipcMessages {
  IPC_ADD_NODE,
  IPC_UPDATE_PARAM,
  IPC_LINK,
  IPC_UNLINK,
  IPC_REMOVE_NODE,
  IPC_MAX
} ipcMsg_t;

class Logger {
public:
  static void init(const std::string& appName = "MyApp");
  static void shutdown();
  static void log(int level, const char *fmt, ...);
};
