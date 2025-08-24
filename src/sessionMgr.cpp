#include "sessionMgr.hpp"
#include "common.hpp"
#include "pw-client.hpp"
#include <cassert>
#include <unordered_map>

static bool initialized = false;
static std::unordered_map<std::string, PipewireClient *> sessionNodesMap;

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
    for (auto it = sessionNodesMap.begin(); it != sessionNodesMap.end(); it++) {
      PipewireClient *pClient = it->second;
      if (pClient)
        delete pClient;
      sessionNodesMap.erase(it);
    }
    initialized = false;
  }
}

std::string SessionMgr::sessionAddNode(std::string uri) {
  PipewireClient *pClient = new PipewireClient();
  pClient->pwInitClient(uri, PLUGIN_TYPE_LV2);
  sessionNodesMap[pClient->filterNodeName] = pClient;
  return "";
}

int SessionMgr::sessionRemoveNode(std::string uuid) { return 0; }
int SessionMgr::sessionUpdateNodeParam(std::string nodeName, int paramIdx,
                                       float val) {
  PipewireClient *pClient = nullptr;
  if (sessionNodesMap.find(nodeName) == sessionNodesMap.end())
    return -ENOENT;

  pClient = sessionNodesMap[nodeName];
  pClient->pwUpdateClientParam(paramIdx, val);
  return 0;
}
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
