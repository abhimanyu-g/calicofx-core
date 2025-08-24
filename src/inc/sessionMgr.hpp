#pragma once

#include <string>
#include <unordered_map>

#include "pw-client.hpp"

enum ipcCommand {
  CALICOFX_ADD_NODE = 0,
  CALICOFX_UPDATE_PARAM,
  CALICOFX_LINK,
  CALICOFX_UNLINK,
  CALICOFX_REMOVE_NODE
};

class SessionMgr {

public:
  std::string sessionAddNode(std::string uri);
  int sessionRemoveNode(std::string uuid);
  int sessionUpdateNodeParam(std::string nodeName, int paramIdx, float val);
  int sessionLinkPort(std::string srcNodeName, std::string srcPortName,
                      std::string dstNodeName, std::string dstPortName);
  int sessionUnlinkPort(std::string srcNodeName, std::string srcPortName,
                        std::string dstNodeName, std::string dstPortName);
  SessionMgr();
  ~SessionMgr();

private:
  std::unordered_map<std::string, class PipeWireClient *> clientMap;
  std::string generateUUID(void);
};
