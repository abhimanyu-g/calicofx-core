#pragma once

#include "lv2PluginMgr.hpp"
#include "pw-client.hpp"

typedef struct node{

}node_t;

/*
 *TODOs
 * - createNode with lv2 plugin and pw client
 * - deleteNode and free plugin and pw client instances
 * - connectPort srcPluginName:portName -> dstPluginName:portName
 * - disconnectPort srcPluginName:portName x-x dstPluginName:portName
 * - activateNode to activate the lv2 and pw client instances
 */
class SessionMgr {
private:
  static bool initialized;
  /////////////////////////////////////////////////////////////////////////////
  //       Callbacks specific to the media layer (Pipewire, jack,...).       //
  /////////////////////////////////////////////////////////////////////////////

  // Pipewire Filter process cb ///////////////////////////////////////////////
  static int process_cb(struct bufferDesc *buffDesc, void *userData);

  public:
    SessionMgr();
    //  int initialize();
    int createNode(const std::string &uid);
    //int connectPort(const char *srcPort, const char *dstPort);
    //int activateNode();
    ~SessionMgr();
  };
