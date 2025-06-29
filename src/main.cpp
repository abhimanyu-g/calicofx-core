#include "common.hpp"
//#include "lv2PluginMgr.hpp"
//#include "sessionMgr.hpp"

int main(void) {

  Logger::init("calicofx");
  // SessionMgr session;

  // session.createNode("http://guitarix.sourceforge.net/plugins/gxautowah#wah");

  while (1);

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
