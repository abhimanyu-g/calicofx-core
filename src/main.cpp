#include <atomic>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "common.hpp"
#include "config.hpp"
#include "sessionMgr.hpp"


std::atomic<bool> bExitIpcLoop(false);

void signalHandler(int sigNum) {
  bExitIpcLoop.store(true);
}

int main(void) {

  Logger::init(FX_PROCESS_NAME);
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  SessionMgr session;

  // session.sessionAddNode("http://guitarix.sourceforge.net/plugins/gxtuner#tuner");
   session.sessionAddNode("http://guitarix.sourceforge.net/plugins/gx_fuzz_#fuzz_");
  // session.sessionAddNode("http://guitarix.sourceforge.net/plugins/gx_amp_stereo#GUITARIX_ST");
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
