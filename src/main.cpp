#include <atomic>
#include <thread>
#include <unistd.h>

#include "common.hpp"
#include "config.hpp"
#include "sessionMgr.hpp"


std::atomic<bool> bExitIpcLoop(false);

int main(void) {

  Logger::init(FX_PROCESS_NAME);
  SessionMgr session;

  while (!bExitIpcLoop) {
    // TODO: Handle messages here
    switch (1) {
    case IPC_ADD_NODE:
      break;
    case IPC_UPDATE_PARAM:
      break;
    case IPC_LINK:
      break;
    case IPC_UNLINK:
      break;
    case IPC_REMOVE_NODE:
      break;
    default:
      break;
    }

    sleep(200);
  }
  return 0;
}
