#include "common.hpp"

void Logger::init(const std::string& ident) {
  openlog(ident.c_str(), LOG_PID | LOG_CONS, LOG_USER);
}

void Logger::log(int lvl, const char *fmt, ...) {
  va_list args;
  va_start (args, fmt);
  vsyslog(lvl, fmt, args);
  va_end(args);
}

void Logger::shutdown() {
  closelog();
}
