#include "common.hpp"
#include "pw-client.hpp"
#include <cassert>
#include "sessionMgr.hpp"


static bool initialized = false;

SessionMgr::SessionMgr() {
  if (!initialized) {
    // One time initialization
    assert(initializePwLib() == 0);

    // TODO: restore old session if available
    initialized = true;
  }
}

SessionMgr::~SessionMgr() {
  if (initialized) {
    // TODO: store the current session to a file
    teardownPwLib();
    initialized = false;
  }
}

std::string generateUUID(void){return "";}
std::string SessionMgr::sessionAddNode(std::string uri) {
  PipewireClient *pClient = new PipewireClient();
  pClient->pwInitClient(uri, PLUGIN_TYPE_LV2);
  return "";
}
int SessionMgr::sessionRemoveNode(std::string uuid){return 0;}
int SessionMgr::sessionUpdateNodeParam(int paramIdx, float val){return 0;}
int SessionMgr::sessionLinkPort(std::string srcNodeName,
                                std::string srcPortName,
                                std::string dstNodeName,
                                std::string dstPortName) {
  return PipewireClient::pwLinkClientPorts(srcNodeName, srcPortName,
                                           dstNodeName, dstPortName);
}

int SessionMgr::sessionUnlinkPort(std::string srcNodeName,
                                  std::string srcPortName,
                                  std::string dstNodeName,
                                  std::string dstPortName) {
  return PipewireClient::pwUnlinkClientPorts(srcNodeName, srcPortName,
                                             dstNodeName, dstPortName);
}


