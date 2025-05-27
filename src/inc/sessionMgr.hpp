#pragma once

typedef struct node{

}node_t;

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
