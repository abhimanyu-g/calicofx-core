#include "common.hpp"
#include "pw-client.hpp"
#include <atomic>
#include <thread>
#include <unistd.h>

std::atomic<bool> bExitIpcLoop(false);
void ipcMsgHandler(void) {
  ipcMsg_t ipcMsg = IPC_CREATE_INSTANCE;
  while (!bExitIpcLoop) {
    // TODO: Handle messages here
    switch (ipcMsg) {
    case IPC_CREATE_INSTANCE:
      break;
    case IPC_CONNECT_PORTS:
      break;
    case IPC_DISCONNECT_PORTS:
      break;
    case IPC_UPDATE_CTRL_PARAM:
      break;
    case IPC_DELETE_INSTANCE:
      break;
    default:
      break;
    }

    sleep(200);
  }
}

int main(void) {
  std::thread ipcHandler;
  Logger::init("calicofx");
  pwInitializeLib();

  ipcHandler = std::thread(ipcMsgHandler);
  pwRunMainLoop();

  if (ipcHandler.joinable()) {
    ipcHandler.join();
  }
  pwDeinitalizeLib();
  return 0;
}
