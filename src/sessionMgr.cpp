#include "common.hpp"
#include "sessionMgr.hpp"

//plugins
#include "lv2PluginMgr.hpp"
#include "pw-client.hpp"

// TODO: mediabackends

bool SessionMgr::initialized = false;

int SessionMgr::process_cb(struct bufferDesc *buffDesc, void *userData) {return 0;}

SessionMgr::SessionMgr() {
  if (!initialized) {
    // One time initializations

    // plugin managers/handlers ///////////////////////////////////////////////
    LV2PluginHandler::initalizeLv2Lib();

    // media managers /////////////////////////////////////////////////////////
    PipeWireClient::initializePwLib();

    initialized = true;
  }
}

int SessionMgr::createNode(const std::string &uid) {
  LV2PluginHandler *p = new LV2PluginHandler(uid, DEFAULT_SAMPLE_RATE);
  PipeWireClient *pw = new PipeWireClient(p->getPluginName(), (void*)p);

  //  pw->registerProcessCb(SessionMgr::process_cb);
  p->activateInstance();
  return 0;
}

SessionMgr::~SessionMgr() {
  if (initialized) {
    // TODO: Uninitialize
    initialized = false;
  }
}
