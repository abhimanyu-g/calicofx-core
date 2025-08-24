#include <cstddef>
#include <cstdint>
#include <list>
#include <spa/param/latency-utils.h>
#include <spa/pod/builder.h>
#include <string>

#include <pipewire/filter.h>
#include <pipewire/impl-node.h>
#include <pipewire/pipewire.h>
#include <random>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "common.hpp"
#include "config.hpp"
#include "pw-client.hpp"

#include "pluginHandlers/basePluginHandler.hpp"
#include "pluginHandlers/lv2/lv2Handler.hpp"
static struct pw_thread_loop *loop = nullptr;
static struct pw_context *context = nullptr;
static struct pw_core *core = nullptr;
static struct pw_registry *registry;
static struct spa_hook regListener;

enum clientObjType { CLIENT_OBJ_NODE = 0, CLIENT_OBJ_PORT, CLIENT_OBJ_LINK };

struct clientNodeParams {
  std::string node;
  std::string port;
  std::string dir;
};

struct clientNodeDesc {
  std::string id;
  std::string label;
  enum clientObjType type;
  const struct pw_properties *props;
  std::vector<std::string> portIDs;
};
static std::unordered_map<std::string, struct clientNodeDesc> nodeGraph;

struct clientLinkDesc {
  std::string id;
  std::string srcNodeId;
  std::string srcPortId;
  std::string dstNodeId;
  std::string dstPortId;
  const struct pw_properties *props;
};
static std::list<struct clientLinkDesc> linkList;

static void regEventGlobal(void *data, uint32_t id, uint32_t permissions,
                           const char *type, uint32_t version,
                           const struct spa_dict *props);
static void on_process(void *userData, struct spa_io_position *position);

static const struct pw_filter_events events = {PW_VERSION_FILTER_EVENTS,
                                               .process = on_process};

static const struct pw_registry_events regEvents = {PW_VERSION_REGISTRY_EVENTS,
                                                    .global = &regEventGlobal};

int initializePwLib() {

  pw_init(0, NULL);
  loop = pw_thread_loop_new(FX_PROCESS_NAME, NULL);
  context = pw_context_new(pw_thread_loop_get_loop(loop), NULL, 0);
  core = pw_context_connect(context, NULL, 0);
  registry = pw_core_get_registry(core, PW_VERSION_REGISTRY, 0);

  pw_registry_add_listener(registry, &regListener, &regEvents, nullptr);

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

static void regEventGlobal(void *data, uint32_t id, uint32_t permissions,
                           const char *type, uint32_t version,
                           const struct spa_dict *props) {

  (void)data;
  (void)permissions;
  (void)version;

  if (!props) {
    SYSLOG_ERR("No props found!\n");
    return;
  }

  if (spa_streq(type, PW_TYPE_INTERFACE_Node)) {

    struct clientNodeDesc nodeDesc = {
        .id = std::to_string(id),
        .label = spa_dict_lookup(props, PW_KEY_NODE_NAME),
        .type = CLIENT_OBJ_NODE,
        .props = pw_properties_new_dict(props),
        .portIDs = {}};
    nodeGraph[nodeDesc.id] = nodeDesc;

  } else if (spa_streq(type, PW_TYPE_INTERFACE_Port)) {
    std::string parentNodeId = spa_dict_lookup(props, PW_KEY_NODE_ID);
    struct clientNodeDesc nodeDesc = {
        .id = std::to_string(id),
        .label = spa_dict_lookup(props, PW_KEY_PORT_NAME),
        .type = CLIENT_OBJ_PORT,
        .props = pw_properties_new_dict(props),
        .portIDs = {}};

    nodeGraph[nodeDesc.id] = nodeDesc;
    nodeGraph[parentNodeId].portIDs.push_back(nodeDesc.id);

  } else if (spa_streq(type, PW_TYPE_INTERFACE_Link)) {
    struct clientLinkDesc linkDesc = {
        .id = std::to_string(id),
        .srcNodeId = spa_dict_lookup(props, PW_KEY_LINK_OUTPUT_NODE),
        .srcPortId = spa_dict_lookup(props, PW_KEY_LINK_OUTPUT_PORT),
        .dstNodeId = spa_dict_lookup(props, PW_KEY_LINK_INPUT_NODE),
        .dstPortId = spa_dict_lookup(props, PW_KEY_LINK_INPUT_PORT),
        .props = pw_properties_new_dict(props)};

    linkList.push_back(linkDesc);
  }
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
//  SYSLOG_DBG("processing done\n");
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
  std::random_device rd;
  std::mt19937 gen(rd()); // Mersenne Twister  Engine
  std::uniform_int_distribution<> dist(1000, 9999);

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
  // Append a rand number to allow multiple instance of the same plugin
  filterNodeName =
      pluginMgr->pluginName + "_" + std::to_string(dist(gen));

  filter = pw_filter_new_simple(
      pw_thread_loop_get_loop(loop), filterNodeName.c_str(),
      pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio", PW_KEY_MEDIA_CATEGORY,
                        "Filter", PW_KEY_MEDIA_ROLE, "DSP", NULL),
      &events, this);

  pwAddInputPorts();
  pwAddOutputPorts();
  // TODO: Add MIDI ports

  //  spaLatInfo.ns = 10 * SPA_NSEC_PER_MSEC;
  spaLatInfo.quantum = (float)DEFAULT_MAX_BLOCK_LEN;
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
  pluginMgr->pluginUpdateParam(clientPortIdx, value);
  return 0;
}

static int clientFetchNodeID(struct clientNodeParams *params,
                             std::string &nodeID, std::string &portID) {

  for (auto it = nodeGraph.begin(); it != nodeGraph.end(); it++) {
    struct clientNodeDesc *desc = &it->second;
    if (desc->type == CLIENT_OBJ_NODE && desc->label == params->node) {
      nodeID = desc->id;
      for (size_t idx = 0; idx < desc->portIDs.size(); idx++) {
        // Iterate over all the port of the source node
        struct clientNodeDesc *port = &nodeGraph[desc->portIDs[idx]];
        if (port->label == params->port &&
            spa_dict_lookup(&port->props->dict, PW_KEY_PORT_DIRECTION) ==
                params->dir) {

          portID = port->id;
          return 0;
        }
      }
    }
  }

  return -ENOENT;
}

int PipewireClient::pwLinkClientPorts(std::string srcNodeName,
                                      std::string srcPortName,
                                      std::string dstNodeName,
                                      std::string dstPortName) {
  std::string srcNodeId, srcPortId, dstNodeId, dstPortId;
  struct pw_properties *props;
  struct clientNodeParams srcParams = {
      .node = srcNodeName, .port = srcPortName, .dir = "out"};
  struct clientNodeParams dstParams = {
      .node = dstNodeName, .port = dstPortName, .dir = "in"};
  int res = 0;

  pw_thread_loop_lock(loop);
  // Get source objects
  if (clientFetchNodeID(&srcParams, srcNodeId, srcPortId) != 0 ||
      clientFetchNodeID(&dstParams, dstNodeId, dstPortId) != 0) {
    SYSLOG_ERR("Failed to create Links. Nodes/ports not found\n");
    res = -ENOENT;
    goto return_status;
  }

  props = pw_properties_new(PW_KEY_LINK_OUTPUT_NODE, srcNodeId.c_str(),
                            PW_KEY_LINK_OUTPUT_PORT, srcPortId.c_str(),
                            PW_KEY_LINK_INPUT_NODE, dstNodeId.c_str(),
                            PW_KEY_LINK_INPUT_PORT, dstPortId.c_str(), NULL);

  if (!pw_core_create_object(core, "link-factory", PW_TYPE_INTERFACE_Link,
                             PW_VERSION_LINK, &props->dict,
                             sizeof(struct pw_link_info))) {
    SYSLOG_ERR("Failed to creating the link between %s:%s -> %s:%s",
               srcNodeId.c_str(), srcPortId.c_str(), dstNodeId.c_str(),
               dstPortId.c_str());
    res = -ENOENT;
    goto return_status;
  }

  SYSLOG_DBG("Successfully created the link between %s:%s -> %s:%s",
             srcNodeId.c_str(), srcPortId.c_str(), dstNodeId.c_str(),
             dstPortId.c_str());

  // TODO: Link listner required ?
return_status:
  pw_thread_loop_unlock(loop);
  return res;
}

int PipewireClient::pwUnlinkClientPorts(std::string srcNodeName,
                                        std::string srcPortName,
                                        std::string dstNodeName,
                                        std::string dstPortName) {
  std::string srcNodeId, srcPortId, dstNodeId, dstPortId;
  struct clientNodeParams srcParams = {
      .node = srcNodeName, .port = srcPortName, .dir = "out"};
  struct clientNodeParams dstParams = {
      .node = dstNodeName, .port = dstPortName, .dir = "in"};
  int res = 0;

  pw_thread_loop_lock(loop);
  // Get source/Destination IDs
  if (clientFetchNodeID(&srcParams, srcNodeId, srcPortId) != 0 ||
      clientFetchNodeID(&dstParams, dstNodeId, dstPortId) != 0) {
    SYSLOG_ERR("Failed to create Links. Nodes/ports not found\n");
    res = -ENOENT;
    goto return_status;
  }

  for (auto it = linkList.begin(); it != linkList.end(); it++) {
    struct clientLinkDesc *desc = &(*it);
    if (desc->srcNodeId == srcNodeId && desc->srcPortId == srcPortId &&
        desc->dstNodeId == dstNodeId && desc->dstPortId == dstPortId) {
      SYSLOG_INF("Unlinking %s:%s --> %s:%s\n", srcNodeName.c_str(),
                 srcPortName.c_str(), dstNodeName.c_str(), dstPortName.c_str());
      pw_registry_destroy(registry, std::stoi(desc->id));
      linkList.erase(it);
      break;
    }
  }

return_status:
  pw_thread_loop_unlock(loop);
  return res;
}

PipewireClient::~PipewireClient() {
  pluginMgr->pluginDeactivate();
  pw_filter_destroy(filter);
}
