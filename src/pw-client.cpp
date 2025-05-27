#include "pw-client.hpp"
#include "common.hpp"
#include "pipewire/filter.h"
#include "pipewire/main-loop.h"
#include "pipewire/properties.h"

struct pw_main_loop *PipeWireClient::loop = nullptr;
struct pw_context *PipeWireClient::context = nullptr;

int PipeWireClient::initializePwLib() {
  pw_init(0, NULL);
  loop = pw_main_loop_new(NULL);
  context = pw_context_new(pw_main_loop_get_loop(loop), NULL, 0);

  SYSLOG_DBG("Compiled with libpipewire %s\n"
             "Linked with libpipewire %s\n",
             pw_get_headers_version(), pw_get_library_version());

  return 0;
}

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
      &events,
      userData);
}

