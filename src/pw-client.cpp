#include <spa/param/latency-utils.h>
#include <spa/pod/builder.h>
#include <string>

#include <pipewire/filter.h>
#include <pipewire/impl-node.h>
#include <pipewire/pipewire.h>
#include <unistd.h>
#include <vector>

#include "common.hpp"
#include "config.hpp"
#include "pw-client.hpp"

#include "pipewire/context.h"
#include "pipewire/thread-loop.h"
#include "pluginHandlers/basePluginHandler.hpp"
#include "pluginHandlers/lv2Handler.hpp"
#include "spa/param/latency.h"
#include "spa/utils/defs.h"

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
  // DE-Initialize all plugin managers (LV2, VST...) //////////////////////////
#ifdef PLUGIN_INCLUDE_LV2
  pluginLv2Deinitalize();
#endif // PLUGIN_INCLUDE_LV2

  pw_core_disconnect(core);
  pw_context_destroy(context);
  pw_thread_loop_stop(loop);
  pw_thread_loop_destroy(loop);
  pw_deinit();

  SYSLOG_INF("Tore down pw main loop...");
  return 0;
}

static void on_process(void *userData, struct spa_io_position *position) {
  PipewireClient *client = (PipewireClient *)userData;
  std::vector<struct pwUserPortDesc *> &pwPortList =
      client->getPwPortDescList();
  PluginBase *plugin = (PluginBase *)client->getPluginMgr();
  uint32_t nSamples = position->clock.duration;

  for (struct pwUserPortDesc *portDesc : pwPortList) {
    float *pBuff = (float *)pw_filter_get_dsp_buffer(portDesc, nSamples);
    if (!pBuff) {
      return;
    }

    plugin->pluginConnectPort(portDesc->pluginPortIdx, pBuff);
    // SYSLOG_DBG("Connecting port [%p] at idx [%d] to [%p]\n", portDesc,
    //            portDesc->pluginPortIdx, pBuff);
  }

  plugin->pluginRun(nSamples);
  // SYSLOG_DBG("processing done\n");
}

void *PipewireClient::getPluginMgr() { return (void *)pluginMgr; }
void *PipewireClient::getFilterNode() { return (void *)filter; }
std::vector<struct pwUserPortDesc *> &PipewireClient::getPwPortDescList() {
  return userPortDescList;
}

int PipewireClient::pwAddInputPorts() {

  for (int port = 0; port < pluginMgr->nAudioInPorts; port++) {
    struct pwUserPortDesc *pUserPortDesc;
    std::string portName = pluginMgr->audioInPortDesc[port].label;

    pUserPortDesc = (struct pwUserPortDesc *)pw_filter_add_port(
        filter, PW_DIRECTION_INPUT, PW_FILTER_PORT_FLAG_MAP_BUFFERS,
        sizeof(struct pwUserPortDesc),
        pw_properties_new(PW_KEY_FORMAT_DSP, "32 bit float mono audio",
                          PW_KEY_PORT_NAME, portName.c_str(), NULL),
        NULL, 0);
    pUserPortDesc->pluginPortIdx = pluginMgr->audioInPortDesc[port].index;
    userPortDescList.push_back(pUserPortDesc);

    SYSLOG_DBG("Added audio port [%s] @ [%p] \n", portName.c_str(),
               pUserPortDesc);
  }
  return 0;
}

int PipewireClient::pwAddOutputPorts() {

  for (int port = 0; port < pluginMgr->nAudioOutPorts; port++) {
    struct pwUserPortDesc *pUserPortDesc;
    std::string portName = pluginMgr->audioOutPortDesc[port].label;

    pUserPortDesc = (struct pwUserPortDesc *)pw_filter_add_port(
        filter, PW_DIRECTION_OUTPUT, PW_FILTER_PORT_FLAG_MAP_BUFFERS,
        sizeof(struct pwUserPortDesc),
        pw_properties_new(PW_KEY_FORMAT_DSP, "32 bit float mono audio",
                          PW_KEY_PORT_NAME, portName.c_str(), NULL),
        NULL, 0);
    pUserPortDesc->pluginPortIdx = pluginMgr->audioOutPortDesc[port].index;
    userPortDescList.push_back(pUserPortDesc);
    SYSLOG_DBG("Added audio port [%s] @ [%p] \n", portName.c_str(),
               pUserPortDesc);
  }

  return 0;
}

int PipewireClient::pwInitClient(std::string uri, enum pluginType pluginType) {

  const struct spa_pod *params[1];
  uint8_t buffer[1024];
  struct spa_pod_builder spaBuilder =
      SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
  struct spa_process_latency_info spaLatInfo;

  struct pw_filter_events events = {
      PW_VERSION_FILTER_EVENTS,
      .process = on_process,
  };

  switch (pluginType) {
#ifdef PLUGIN_INCLUDE_LV2
  case PLUGIN_TYPE_LV2:
    pluginMgr = new LV2PluginHandler();
    pluginMgr->pluginInit((void *)uri.c_str());
    break;
#endif // PLUGIN_INCLUDE_LV2
  default:
    pluginMgr = nullptr;
    break;
  }

  if (!pluginMgr) {
    SYSLOG_ERR("No suitable handler found for plugin [%s].\n", uri.c_str());
    return -EINVAL;
  }

  pw_thread_loop_lock(loop);
  filter = pw_filter_new_simple(
      pw_thread_loop_get_loop(loop), pluginMgr->pluginName.c_str(),
      pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio", PW_KEY_MEDIA_CATEGORY,
                        "Filter", PW_KEY_MEDIA_ROLE, "DSP", NULL),
      &events, this);

  pwAddInputPorts();
  pwAddOutputPorts();

  spaLatInfo.ns = 10 * SPA_NSEC_PER_MSEC;
  params[0] = spa_process_latency_build(&spaBuilder, SPA_PARAM_ProcessLatency,
                                        &spaLatInfo);

  if (pw_filter_connect(filter, PW_FILTER_FLAG_RT_PROCESS, params, 1) < 0) {
    SYSLOG_ERR("Failed to connect plugin [%s]. Try again\n",
               pluginMgr->pluginName.c_str());
    return -EAGAIN;
  }
  pluginMgr->pluginActivate();
  pw_thread_loop_unlock(loop);

  return 0;
}

int PipewireClient::pwUpdateClientParam(int clientPortIdx, float value) {
  return 0;
}

int PipewireClient::pwLinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                                      std::string dstNodeUUID, int dstPortIdx) {
  return 0;
}

int PipewireClient::pwUnlinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                                        std::string dstNodeUUID,
                                        int dstPortIdx) {
  return 0;
}

PipewireClient::~PipewireClient() {
  pluginMgr->pluginDeactivate();
  pw_filter_destroy(filter);
}
