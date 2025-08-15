#include "pw-client.hpp"
#include "common.hpp"
#include "config.hpp"
#include "pipewire/context.h"
#include "pipewire/core.h"
#include "pipewire/filter.h"
#include "pipewire/main-loop.h"
#include "pipewire/pipewire.h"
#include "pipewire/properties.h"
#include "pipewire/thread-loop.h"
#include "pluginHandlers/lv2Handler.hpp"

static struct pw_thread_loop *loop = nullptr;
static struct pw_context *context = nullptr;
static struct pw_core *core = nullptr;

int initializePwLib() {
  pw_init(0, NULL);
  loop = pw_thread_loop_new(FX_PROCESS_NAME, NULL);
  context = pw_context_new(pw_thread_loop_get_loop(loop), NULL, 0);
  core = pw_context_connect(context, NULL, 0);

  // Initialize all plugin managers (LV2, VST...)  ////////////////////////////
  SYSLOG_DBG("Initializing plugin managers..\n");
#ifdef PLUGIN_INCLUDE_LV2
  pluginLv2Initialize();
#endif // PLUGIN_INCLUDE_LV2

  SYSLOG_DBG("Compiled with libpipewire %s\n"
             "Linked with libpipewire %s\n",
             pw_get_headers_version(), pw_get_library_version());

  SYSLOG_INF("Starting pw main loop...");
  pw_thread_loop_start(loop);

  return 0;
}

int teardownPwLib() {
  // De-Initialize all plugin managers (LV2, VST...) //////////////////////////
#ifdef PLUGIN_INCLUDE_LV2
  pluginLv2Deinitalize();
#endif // PLUGIN_INCLUDE_LV2

  pw_core_disconnect(core);
  pw_context_destroy(context);
  pw_thread_loop_stop(loop);
  pw_thread_loop_destroy(loop);
  pw_deinit();

  return 0;
}

// Functionality to be updated ////////////////////////////////////////////////
static void on_process(void *userdata,
                                struct spa_io_position *position) {
//  struct bufferDesc desc = {};
  // if (!runProcessCb)
  //   return;
}

int PipewireClient::pwInitClient(std::string uri, enum pluginType pluginType){return 0;}
int PipewireClient::pwUpdateClientParam(int clientPortIdx, float value){return 0;}
int PipewireClient::pwLinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                                      std::string dstNodeUUID, int dstPortIdx){return 0;}

int PipewireClient::pwUnlinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                                        std::string dstNodeUUID, int dstPortIdx){return 0;}

PipewireClient::~PipewireClient(){}

#if 0
PipeWireClient::PipeWireClient(const std::string &name, void *userData) {
  struct pw_filter_events events = {
      PW_VERSION_FILTER_EVENTS,
      .process = on_process,
  };
  filter = pw_filter_new_simple(
      pw_main_loop_get_loop(loop), name.c_str(),
      pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio", PW_KEY_MEDIA_CATEGORY,
                        "Source", PW_KEY_MEDIA_ROLE, "DSP", PW_KEY_MEDIA_CLASS,
                        "Stream/Output/Audio", PW_KEY_NODE_AUTOCONNECT, "true",
                        NULL),
      &events, userData);
}
#endif
