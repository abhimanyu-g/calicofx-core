#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <list>
#include <random>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include <pipewire/filter.h>
#include <pipewire/impl-node.h>
#include <pipewire/pipewire.h>
#include <spa/control/control.h>
#include <spa/param/latency-utils.h>
#include <spa/pod/builder.h>

#include "common.hpp"
#include "config.hpp"
#include "pipewire/stream.h"
#include "pluginHandlers/basePluginHandler.hpp"
#include "pluginHandlers/lv2/lv2Handler.hpp"
#include "spa/buffer/buffer.h"
#include "spa/pod/iter.h"
#include "spa/pod/pod.h"
#include "spa/utils/type.h"
#include "pw-client.hpp"

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

static const struct pw_filter_events events = {PW_VERSION_FILTER_EVENTS,
                                               .process = PipewireClient::pwOnProcess};

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

  // pw_thread_loop_lock(loop);
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

static int rawMidiOutCb(uint32_t offset, uint32_t size, const void *evData,
                        void *userData) {
  struct spa_pod_builder *midiPodBuilder = nullptr;
  if (!userData || !evData)
    return -EINVAL;

  SYSLOG_DBG("raw midi callback\n");
  midiPodBuilder = (struct spa_pod_builder *)userData;
  spa_pod_builder_control(midiPodBuilder, offset, SPA_CONTROL_Midi);
  spa_pod_builder_bytes(midiPodBuilder, evData, size);
  return 0;
}

int PipewireClient::pwPreProcess(struct clientPortDesc *port,
                                 uint32_t nSamples) {

  struct pw_buffer *buff = nullptr;
  struct spa_data *spaData = nullptr;
  struct portDesc *pluginPortDesc = nullptr;

  if (!port || !(buff = pw_filter_dequeue_buffer(port))) {
    /* No buffer available. Is port connected? */
    return -ENOENT;
  }

  pluginPortDesc = port->pluginPortDesc;
  /* Collect the data from buffer. Each pw_buffer structure holds a spa_buffer
   * structure (`buffer`) which inturn holds data planes of type struct
   * spa_data. In our case, for audio/midi, we use only plane-0. Each plane
   * points to a void pointer of data that would be later typecasted to the
   * port type data (audio/midi). So, in summary, we get
   * (struct pw_buffer)->(struct spa_buffer)->(struct spa_data)[0]->buff data
   */
  spaData = &buff->buffer->datas[0];
  switch (pluginPortDesc->type) {

  case PORT_TYPE_AUDIO: {
    float *data = (float *)spaData->data;
    pluginMgr->pluginConnectPort(pluginPortDesc->index, data);
    if (pluginPortDesc->dir == PORT_DIR_OUTPUT) {
      spaData->chunk->offset = 0;
      spaData->chunk->size =
        std::min((uint32_t)(nSamples * sizeof(float)), spaData->maxsize);
      spaData->chunk->stride = sizeof(float);
      spaData->chunk->flags = 0;
    }
  } break;

  case PORT_TYPE_MIDI:
    if (pluginPortDesc->dir == PORT_DIR_INPUT) {
      struct spa_pod_sequence *seq = (struct spa_pod_sequence *)spaData->data;
      struct spa_pod_control *ctrl;

      /* Initialize the plugin's implementation of Plain Old data (POD). For
       * example, LV2 uses Atom
       */
      pluginMgr->pluginInitPod(pluginPortDesc->index);
      SPA_POD_SEQUENCE_FOREACH(seq, ctrl) {
        if (ctrl->type == SPA_CONTROL_Midi) {
          // We only care about raw MIDI bytes. UMP is not yet supported
          const uint8_t *midiBytes =
              (const uint8_t *)SPA_POD_BODY_CONST(&ctrl->value);
          uint32_t len = SPA_POD_BODY_SIZE(&ctrl->value);
          pluginMgr->pluginAppendPodEvent(ctrl->offset, len, midiBytes);
        }
      }
      pluginMgr->pluginPodFinalize();
    }
    break;

  default:
    SYSLOG_ERR("Unsupported pre-process port type %d\n", pluginPortDesc->type);
    break;
  }

  pw_filter_queue_buffer((void *)port, buff);
  return 0;
}

int PipewireClient::pwRunProcess(uint32_t nSamples) {
  pluginMgr->pluginRun(nSamples);

  // TODO: Handle if any worker thread completetion here ?
  return 0;
}

int PipewireClient::pwPostProcess(struct clientPortDesc *port,
                                  uint32_t nSamples) {
  struct portDesc *pluginPortDesc = port->pluginPortDesc;
  struct spa_data *spaData = nullptr;
  struct pw_buffer *buff = nullptr;

  if (pluginPortDesc->dir == PORT_DIR_INPUT ||
      pluginPortDesc->type == PORT_TYPE_AUDIO ||
      !(buff = pw_filter_dequeue_buffer(port))) {
    // Processing is done. We have nothing to do for input ports or audio ports
    return -EINVAL;
  }

  spaData = &buff->buffer->datas[0];
  if (pluginPortDesc->type == PORT_TYPE_MIDI) {
    // Handling MIDI output ports
    struct spa_pod_frame midiFrame;
    struct spa_pod_builder midiPod =
        SPA_POD_BUILDER_INIT(spaData->data, spaData->maxsize);

    // SYSLOG_DBG("processing midi out for %d\n", pluginPortDesc->index);
    spa_pod_builder_push_sequence(&midiPod, &midiFrame, SPA_CONTROL_Midi);
    /* Let the pluginManager handler MIDI out data and call our callback
     * `rawMidiOutCb` for every event data
     */
    pluginMgr->pluginPodGetMidiOut(pluginPortDesc->index, &rawMidiOutCb,
                                   (void *)&midiPod);
    spa_pod_builder_pop(&midiPod, &midiFrame);

    spaData->chunk->offset = 0;
    spaData->chunk->size = midiPod.state.offset;
    spaData->chunk->stride = 1;
    spaData->chunk->flags = 0;
  }

  pw_filter_queue_buffer((void *)port, buff);
  return 0;
}

void PipewireClient::pwOnProcess(void *userData, struct spa_io_position *position) {
  PipewireClient *client = (PipewireClient *)userData;
  uint32_t nSamples = position->clock.duration;
  int res = 0;

  for (struct clientPortDesc *portDesc : client->clientPortList) {
    struct portDesc *pluginPortDesc = portDesc->pluginPortDesc;

    res = client->pwPreProcess(portDesc, nSamples);
    if (pluginPortDesc->type == PORT_TYPE_AUDIO && (res < 0)) {
      // Our audio ports are not yet connected. Delay processing
      return;
    }
  }

  // Process the Input buffers
  client->pwRunProcess(nSamples);

  for (struct clientPortDesc *portDesc : client->clientPortList) {
    client->pwPostProcess(portDesc, nSamples);
  }
}

void *PipewireClient::getPluginMgr() { return (void *)pluginMgr; }
void *PipewireClient::getFilterNode() { return (void *)filter; }

int PipewireClient::pwAddInputPorts() {

  size_t nAudioPorts = pluginMgr->audioInPortDesc.size();
  for (size_t port = 0; port < nAudioPorts; port++) {
    struct clientPortDesc *pUserPortDesc;
    std::string portName = pluginMgr->audioInPortDesc[port].label;

    pUserPortDesc = (struct clientPortDesc *)pw_filter_add_port(
        filter, PW_DIRECTION_INPUT, PW_FILTER_PORT_FLAG_NONE,
        sizeof(struct clientPortDesc),
        pw_properties_new(PW_KEY_FORMAT_DSP, "32 bit float mono audio",
                          PW_KEY_PORT_NAME, portName.c_str(), NULL),
        NULL, 0);
    pUserPortDesc->pluginPortDesc = &pluginMgr->audioInPortDesc[port];
    clientPortList.push_back(pUserPortDesc);

    SYSLOG_DBG("Added audio port [%s] @ [%p] \n", portName.c_str(),
               pUserPortDesc);
  }
  return 0;
}

int PipewireClient::pwAddOutputPorts() {
  size_t nAudioOutPorts = pluginMgr->audioOutPortDesc.size();
  for (size_t port = 0; port < nAudioOutPorts; port++) {
    struct clientPortDesc *pUserPortDesc;
    std::string portName = pluginMgr->audioOutPortDesc[port].label;

    pUserPortDesc = (struct clientPortDesc *)pw_filter_add_port(
        filter, PW_DIRECTION_OUTPUT, PW_FILTER_PORT_FLAG_NONE,
        sizeof(struct clientPortDesc),
        pw_properties_new(PW_KEY_FORMAT_DSP, "32 bit float mono audio",
                          PW_KEY_PORT_NAME, portName.c_str(), NULL),
        NULL, 0);
    pUserPortDesc->pluginPortDesc = &pluginMgr->audioOutPortDesc[port];
    clientPortList.push_back(pUserPortDesc);
    SYSLOG_DBG("Added audio port [%s] @ [%p] \n", portName.c_str(),
               pUserPortDesc);
  }

  return 0;
}

int PipewireClient::pwAddMidiPorts() {
  size_t nMidiPorts = pluginMgr->midiPortDesc.size();

  for (size_t port = 0; port < nMidiPorts; port++) {
    struct clientPortDesc *pClientPortDesc;
    struct portDesc *pPluginPortDesc = &pluginMgr->midiPortDesc[port].portInfo;
    std::string portName = pPluginPortDesc->label;
    enum spa_direction midiPortDir =
        pPluginPortDesc->dir == PORT_DIR_INPUT
            ? SPA_DIRECTION_INPUT
            : SPA_DIRECTION_OUTPUT;

    pClientPortDesc = (struct clientPortDesc *)pw_filter_add_port(
        filter, midiPortDir, PW_FILTER_PORT_FLAG_NONE,
        sizeof(struct clientPortDesc),
        pw_properties_new(PW_KEY_PORT_NAME, portName.c_str(), PW_KEY_MEDIA_TYPE,
                          "application/control", PW_KEY_FORMAT_DSP,
                          "8 bit raw midi", NULL),
        NULL, 0);
    pClientPortDesc->pluginPortDesc = pPluginPortDesc;
    clientPortList.push_back(pClientPortDesc);
    SYSLOG_DBG("Added Midi port [%s] @ [%p] \n", portName.c_str(),
               pClientPortDesc);
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
  filterNodeName = pluginMgr->pluginName + "_4444";// + std::to_string(dist(gen));

  filter = pw_filter_new_simple(
      pw_thread_loop_get_loop(loop), filterNodeName.c_str(),
      pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio", PW_KEY_MEDIA_CATEGORY,
                        "Filter", PW_KEY_MEDIA_ROLE, "DSP", NULL),
      &events, this);

  pwAddInputPorts();
  pwAddOutputPorts();
  pwAddMidiPorts();

  // Process a pre-configured block every on_process cycle
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
  std::string nodeID = "";

  if (pluginMgr) {
    pluginMgr->pluginDeactivate();
    pluginMgr->pluginDestroy();

    delete pluginMgr; // Delete the object itself
  }

  // Evict any links that might exist.
  for (auto it = linkList.begin(); it != linkList.end(); it++) {
    struct clientLinkDesc *pLinkDesc = &(*it);
    if (pLinkDesc->srcNodeId == nodeID || pLinkDesc->dstNodeId == nodeID) {
      linkList.erase(it);
    }
  }

  for (auto it = nodeGraph.begin(); it != nodeGraph.end(); it++) {
    struct clientNodeDesc *nodeDesc = &it->second;
    if (nodeDesc->label == filterNodeName) {
      nodeID = nodeDesc->id;
      break;
    }
  }

  if (nodeID != "") {
    struct clientNodeDesc *nodeDesc = &nodeGraph[nodeID];
    // Delete all the node's ports
    for (std::string &portID : nodeDesc->portIDs) {
      nodeGraph.erase(nodeGraph.find(portID));
    }

    // Delete the node itself
    nodeGraph.erase(nodeGraph.find(nodeID));
  }

  pw_thread_loop_lock(loop);
  pw_filter_disconnect(filter);
  pw_filter_destroy(filter);
  pw_thread_loop_unlock(loop);
}
