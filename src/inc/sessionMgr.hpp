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
  int sessionUpdateNodeParam(int paramIdx, float val);
  int sessionLinkPort(std::string srcNodeUUID, int srcPortIdx,
                      std::string dstNodeUUID, int dstPortIdx);
  int sessionUnlinkNode(std::string srcNodeUUID, int srcPortIdx,
                        std::string dstNodeUUID, int dstPortIdx);
  SessionMgr();
  ~SessionMgr();

private:
  std::unordered_map<std::string, class PipeWireClient*> clientMap;
  std::string generateUUID(void);
};
