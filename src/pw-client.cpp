#include "pw-client.hpp"
#include "common.hpp"
#include "pipewire/context.h"
#include "pipewire/core.h"
#include "pipewire/filter.h"
#include "pipewire/main-loop.h"
#include "pipewire/pipewire.h"
#include "pipewire/properties.h"
#include "pluginHandlers/lv2Handler.hpp"

static struct pw_main_loop *loop = nullptr;
static struct pw_context *context = nullptr;
static struct pw_core *core = nullptr;

int pwInitializeLib() {
  pw_init(0, NULL);
  loop = pw_main_loop_new(NULL);
  context = pw_context_new(pw_main_loop_get_loop(loop),
                           NULL, 0);
  core = pw_context_connect(context, NULL, 0);

  // Initialize all plugin managers (LV2, VST...)  ////////////////////////////
  SYSLOG_DBG("Initializing plugin managers..\n");
  pluginLv2Initialize();

  SYSLOG_DBG("Compiled with libpipewire %s\n"
             "Linked with libpipewire %s\n",
             pw_get_headers_version(), pw_get_library_version());
  return 0;
}

int pwDeinitalizeLib() {
  pw_core_disconnect(core);
  pw_context_destroy(context);
  pw_main_loop_destroy(loop);
  pw_deinit();

  // De-Initialize all plugin managers (LV2, VST...) //////////////////////////
  pluginLv2Deinitalize();

  return 0;
}


void pwRunMainLoop() {
  SYSLOG_DBG("Starting pipewire main-loop\n");

  // Blocking call to process all pipewire events
  pw_main_loop_run(loop);
}

int pwActionDispatchHandler(action_t dispatchAction, void *payload) {
  (void)payload;
  switch (dispatchAction) {
  case ACTION_CREATE_INSTANCE:
    break;
  case ACTION_CONNECT_PORTS:
    break;
  case ACTION_DISCONNECT_PORTS:
    break;
  case ACTION_UPDATE_CTRL_PARAM:
    break;
  case ACTION_DELETE_INSTANCE:
    break;
  default:
    break;
  }
  return 0;
}

// Functionality to be updated ////////////////////////////////////////////////
void PipeWireClient::on_process(void *userdata,
                                struct spa_io_position *position) {
  struct bufferDesc desc = {};
  // if (!runProcessCb)
  //   return;
}

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
