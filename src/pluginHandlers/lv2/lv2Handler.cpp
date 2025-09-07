#include <errno.h>
#include <lilv/lilv.h>
#include <limits>
#include <lv2/atom/util.h>
#include <lv2/core/lv2.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <lv2/atom/atom.h>
#include <lv2/atom/forge.h>
#include <lv2/buf-size/buf-size.h>
#include <lv2/midi/midi.h>
#include <lv2/options/options.h>
#include <lv2/parameters/parameters.h>
#include <lv2/urid/urid.h>
#include <lv2/worker/worker.h>

#include "common.hpp"
#include "config.hpp"
#include "pluginHandlers/basePluginHandler.hpp"
#include "pluginHandlers/lv2/lv2Handler.hpp"

#define PLUGIN_ATOM_SEQ_BUFSIZE (4096)

using namespace std;

static LilvWorld *world = nullptr;
static LilvPlugins *plugins = nullptr;
static LilvNode *lilvNodeAudioPort = nullptr;
static LilvNode *lilvNodeInputPort = nullptr;
static LilvNode *lilvNodeOutputPort = nullptr;
static LilvNode *lilvNodeControlPort = nullptr;
static LilvNode *lilvNodeScalePoint = nullptr;
static LilvNode *lilvNodeAtomPort = nullptr;
static LilvNode *lilvNodeSupportsMidiEvent = nullptr;
static LilvNode *lilvNodeWorkerInterface = nullptr;
static LilvNode *lilvNodeWorkerSchedule = nullptr;
static vector<string> nodesURI;

static const float pluginSampleRate = DEFAULT_SAMPLE_RATE;
static const int32_t pluginMinBlockLen = DEFAULT_MIN_BLOCK_LEN;
static const int32_t pluginMaxBlockLen = DEFAULT_MAX_BLOCK_LEN;
static const int32_t pluginSeqBufLen = DEFAULT_SEQ_BUF_SIZE;
///////////////////////////////////////////////////////////////////////////////
//                          Host supported features                          //
///////////////////////////////////////////////////////////////////////////////

// Map///////////////////////////////////////////////////////////////
static LV2_URID nextURID = 1;
static std::unordered_map<std::string, LV2_URID> uridMap;
LV2_URID lv2_urid_map(LV2_URID_Map_Handle handle, const char *uri);
LV2_URID_Map pluginUridMapDesc = {NULL, lv2_urid_map};
LV2_Feature pluginFeatMapUrid = {LV2_URID__map, &pluginUridMapDesc};

// UnMap ////////////////////////////////////////////////////////////////
static std::unordered_map<LV2_URID, std::string> uridUnmap;
const char *lv2_urid_unmap(LV2_URID_Unmap_Handle, const LV2_URID urid);
LV2_URID_Unmap pluginUridUnmapDesc = {NULL, lv2_urid_unmap};
LV2_Feature pluginFeatUnmapUrid = {LV2_URID__unmap, &pluginUridUnmapDesc};

// Options ////////////////////////////////////////////////////////////////////
const LV2_Options_Option pluginOptions[] = {
    {LV2_OPTIONS_INSTANCE, 0, lv2_urid_map(NULL, LV2_PARAMETERS__sampleRate),
     sizeof(float), lv2_urid_map(NULL, LV2_ATOM__Float), &pluginSampleRate},
    {LV2_OPTIONS_INSTANCE, 0, lv2_urid_map(NULL, LV2_BUF_SIZE__maxBlockLength),
     sizeof(int32_t), lv2_urid_map(NULL, LV2_ATOM__Int), &pluginMaxBlockLen},
    {LV2_OPTIONS_INSTANCE, 0, lv2_urid_map(NULL, LV2_BUF_SIZE__minBlockLength),
     sizeof(int32_t), lv2_urid_map(NULL, LV2_ATOM__Int), &pluginMinBlockLen},
    {LV2_OPTIONS_INSTANCE, 0, lv2_urid_map(NULL, LV2_BUF_SIZE__sequenceSize),
     sizeof(int32_t), lv2_urid_map(NULL, LV2_ATOM__Int), &pluginSeqBufLen},
    {LV2_OPTIONS_INSTANCE, 0, 0, 0, 0, NULL}};

LV2_Feature pluginFeatOptions = {LV2_OPTIONS__options, (void *)pluginOptions};
// end of Host supported Features /////////////////////////////////////////////

LV2_URID lv2_urid_map(LV2_URID_Map_Handle handle, const char *uri) {
  (void)handle;
  auto it = uridMap.find(uri);
  if (it != uridMap.end())
    return it->second;

  LV2_URID id = nextURID++;
  uridMap[uri] = id;
  uridUnmap[id] = uri;
  return id;
}

const char *lv2_urid_unmap(LV2_URID_Unmap_Handle, const LV2_URID urid) {
  auto it = uridUnmap.find(urid);
  if (it == uridUnmap.end())
    return nullptr;

  return it->second.c_str();
}

int pluginLv2Initialize() {
  world = lilv_world_new();
  if (!world)
    return -EAGAIN;

  // Load all available bundles
  lilv_world_load_all(world);

  // load all plugins from bundles
  plugins = (void *)lilv_world_get_all_plugins(world);

  // FIXME: Push this to a local database accessible from the UI
  LILV_FOREACH(plugins, i, plugins) {
    const LilvPlugin *p = lilv_plugins_get(plugins, i);
    string node = string(lilv_node_as_uri(lilv_plugin_get_uri(p)));
    nodesURI.push_back(node);
    //    SYSLOG_DBG("%s\n", node.c_str());
  }

  lilvNodeAudioPort = lilv_new_uri(world, LV2_CORE__AudioPort);
  lilvNodeInputPort = lilv_new_uri(world, LV2_CORE__InputPort);
  lilvNodeOutputPort = lilv_new_uri(world, LV2_CORE__OutputPort);
  lilvNodeControlPort = lilv_new_uri(world, LV2_CORE__ControlPort);
  lilvNodeScalePoint = lilv_new_uri(world, LV2_CORE__ScalePoint);
  lilvNodeAtomPort = lilv_new_uri(world, LV2_ATOM__AtomPort);
  lilvNodeSupportsMidiEvent = lilv_new_uri(world, LV2_MIDI__MidiEvent);
  lilvNodeWorkerInterface = lilv_new_uri(world, LV2_WORKER__interface);
  lilvNodeWorkerSchedule = lilv_new_uri(world, LV2_WORKER__schedule);
  SYSLOG_DBG("LV2 plugin library initialized\n");
  return 0;
}

int pluginLv2Deinitalize() {
  lilv_node_free(lilvNodeAudioPort);
  lilv_node_free(lilvNodeInputPort);
  lilv_node_free(lilvNodeOutputPort);
  lilv_node_free(lilvNodeControlPort);
  lilv_node_free(lilvNodeScalePoint);
  lilv_node_free(lilvNodeAtomPort);
  lilv_node_free(lilvNodeSupportsMidiEvent);
  lilv_node_free(lilvNodeWorkerInterface);
  lilv_node_free(lilvNodeWorkerSchedule);

  lilv_world_free(world);
  SYSLOG_DBG("LV2 plugin library de-initialized\n");
  return 0;
}

void LV2PluginHandler::populatePorts(const LilvPlugin *plugin) {
  int nPorts = -1;

  nPorts = lilv_plugin_get_num_ports(plugin);
  for (uint8_t idx = 0; idx < nPorts; ++idx) {
    // nPorts would always have [0...nPorts] sequentially
    struct portDesc desc = {};
    const LilvPort *port = lilv_plugin_get_port_by_index(plugin, idx);
    LilvNode *portNameNode = lilv_port_get_name(plugin, port);

    desc.label = string(lilv_node_as_string(portNameNode));
    desc.index = idx;
    lilv_node_free(portNameNode);

    // audio ports
    if (lilv_port_is_a(plugin, port, lilvNodeAudioPort)) {
      desc.type = PORT_TYPE_AUDIO;
      if (lilv_port_is_a(plugin, port, lilvNodeInputPort)) {
        // audio-in ports
        desc.dir = PORT_DIR_INPUT;
        this->audioInPortDesc.push_back(desc);
      } else {
        // audio-out ports
        desc.dir = PORT_DIR_OUTPUT;
        this->audioOutPortDesc.push_back(desc);
      }
    }

    // Atom ports
    else if (lilv_port_is_a(plugin, port, lilvNodeAtomPort)) {

      if (lilv_port_supports_event(plugin, port, lilvNodeSupportsMidiEvent)) {
        struct midiPortDesc portDesc;
        LV2_Atom_Sequence *seqBuff = nullptr;
        desc.dir = lilv_port_is_a(plugin, port, lilvNodeInputPort)
                       ? PORT_DIR_INPUT
                       : PORT_DIR_OUTPUT;
        desc.type = PORT_TYPE_MIDI;

        portDesc.portInfo = desc;
        seqBuff = (LV2_Atom_Sequence *)portDesc.portData;
        seqBuff->atom.type = lv2_urid_map(NULL, LV2_ATOM__Sequence);
        seqBuff->atom.size = sizeof(LV2_Atom_Sequence_Body);

        pluginConnectPort(desc.index, portDesc.portData);
        midiPortDesc.push_back(portDesc);
        SYSLOG_DBG("configuring MIDI port at idx [%d]\n", desc.index);
      } else {
        SYSLOG_ERR("Unsupported ATOM type for idx [%d]\n", desc.index);
      }
    }

    // control ports
    else if (lilv_port_is_a(plugin, port, lilvNodeControlPort)) {
      LilvNode *def, *min, *max;
      LilvScalePoints *sp;
      struct controlPortDesc ctrlPortDesc = {};

      desc.type = PORT_TYPE_CONTROL;
      ctrlPortDesc.portInfo = desc;
      ctrlPortDesc.def = ctrlPortDesc.min = ctrlPortDesc.max =
          ctrlPortDesc.val = numeric_limits<float>::max();
      ctrlPortDesc.hasScalePoints = false;

      lilv_port_get_range(plugin, port, &def, &min, &max);

      if (def) {
        ctrlPortDesc.def = lilv_node_as_float(def);
        lilv_node_free(def);
      }
      if (min) {
        ctrlPortDesc.min = lilv_node_as_float(min);
        lilv_node_free(min);
      }
      if (max) {
        ctrlPortDesc.max = lilv_node_as_float(max);
        lilv_node_free(max);
      }

      ctrlPortDesc.val = ctrlPortDesc.def;
      sp = lilv_port_get_scale_points(plugin, port);
      if (sp) {
        ctrlPortDesc.hasScalePoints = true;

        LILV_FOREACH(scale_points, spNode, sp) {
          struct scalePointsDesc scalePointDesc;
          const LilvScalePoint *pScalePoint = lilv_scale_points_get(sp, spNode);

          const LilvNode *pScalePointLabelNode =
              lilv_scale_point_get_label(pScalePoint);
          scalePointDesc.label =
              string(lilv_node_as_string(pScalePointLabelNode));

          const LilvNode *pScalePointValNode =
              lilv_scale_point_get_value(pScalePoint);
          scalePointDesc.val = lilv_node_as_float(pScalePointValNode);

          ctrlPortDesc.scalePoints.push_back(scalePointDesc);
        }
        lilv_scale_points_free(sp);
      }

      SYSLOG_DBG("configuring control port at idx [%d]\n",
                 ctrlPortDesc.portInfo.index);
      pluginConnectPort(ctrlPortDesc.portInfo.index, &ctrlPortDesc.val);
      controlPortDesc.push_back(ctrlPortDesc);
    }
  }
}

int LV2PluginHandler::pluginRun(int nSamples) {
  lilv_instance_run(this->pluginInstance, nSamples);
  if (pWorkerHdl) {
    pWorkerHdl->workerJobResponse();
  }
  return 0;
}

int LV2PluginHandler::pluginConnectPort(uint8_t portIdx, void *buf) {
  lilv_instance_connect_port(this->pluginInstance, portIdx, buf);
  return 0;
}

int LV2PluginHandler::pluginActivate() {
  lilv_instance_activate(this->pluginInstance);
  return 0;
}

int LV2PluginHandler::pluginInit(void *uri) {
  LilvNode *pluginNode = nullptr, *pluginUriNode = nullptr;
  LilvPlugin *plugin = nullptr;

  pWorkerHdl = nullptr;
  LV2_Worker_Schedule scheduleDesc = {this, &workerStaticScheduler};
  LV2_Feature pluginFeatWorker = {LV2_WORKER__schedule, (void *)&scheduleDesc};

  if (!uri) {
    SYSLOG_ERR("Invalid usage. LV2 plugin URI required.\n");
    return -EINVAL;
  }

  pluginURI = (const char *)uri;
  pluginUriNode = lilv_new_uri(world, pluginURI.c_str());

  plugin = (LilvPlugin *)lilv_plugins_get_by_uri(plugins, pluginUriNode);
  if (!plugin) {
    SYSLOG_ERR("plugin URI [%s] not found\n", pluginURI.c_str());
    lilv_node_free(pluginUriNode);
    return -EAGAIN;
  }
  lilv_node_free(pluginUriNode);

  pluginNode = lilv_plugin_get_name(plugin);
  if (pluginNode) {
    pluginName = string(lilv_node_as_string(pluginNode));
    lilv_node_free(pluginNode);
  }

  static const LV2_Feature *pluginSupportedFeatures[] = {
      &pluginFeatUnmapUrid, &pluginFeatMapUrid, &pluginFeatOptions,
      &pluginFeatWorker, NULL};

  // Instantiate plugin
  pluginInstance = lilv_plugin_instantiate(plugin, DEFAULT_SAMPLE_RATE,
                                           pluginSupportedFeatures);
  if (!pluginInstance) {
    SYSLOG_ERR("Failed to instantiate plugin [%s]. Try again\n",
               pluginName.c_str());
    return -EAGAIN;
  }

  // Is worker thread required?
  if (lilv_plugin_has_feature((const LilvPlugin *)plugin,
                              lilvNodeWorkerSchedule)) {
    pWorkerHdl = new Worker(
        lilv_instance_get_extension_data(pluginInstance, LV2_WORKER__interface),
        lilv_instance_get_handle(pluginInstance));
  }

  /* Plugin should be instantiated before connecting the ports because the API
   * demands so */
  populatePorts(plugin);

  SYSLOG_DBG("Added plugin [%s] successfully\n", pluginName.c_str());
  return 0;
}

int LV2PluginHandler::pluginUpdateParam(uint8_t idx, float val) {
  for (struct controlPortDesc &ctrlPort : controlPortDesc) {
    if (ctrlPort.portInfo.index == idx) {
      if (ctrlPort.min < ctrlPort.max &&
          (val < ctrlPort.min || val > ctrlPort.max)) {
        return -EINVAL;
      }
      ctrlPort.val = val;
      break;
    }
  }
  return 0;
}

int LV2PluginHandler::pluginInitPod(uint8_t portIdx) {
  for (struct midiPortDesc &midiPortDesc : midiPortDesc) {
    if (midiPortDesc.portInfo.index == portIdx) {
      seq = (LV2_Atom_Sequence *)midiPortDesc.portData;

      // Initialize forge to append the Atom Events to the sequence buffer
      lv2_atom_forge_set_buffer(&forge, (uint8_t *)&seq->atom,
                                sizeof(LV2_Atom_Sequence));
      lv2_atom_forge_sequence_head(&forge, &frame, 0);
      return 0;
    }
  }
  return -ENOENT;
}

int LV2PluginHandler::pluginAppendPodEvent(size_t offset, uint32_t len,
                                           const uint8_t *data) {
  // Assuming forge was setup earlier
  lv2_atom_forge_frame_time(&forge, offset);
  lv2_atom_forge_atom(&forge, len, lv2_urid_map(NULL, LV2_ATOM__Event));
  lv2_atom_forge_write(&forge, (const void*)data, len);
  return 0;
}

int LV2PluginHandler::pluginPodFinalize() {
  lv2_atom_forge_pop(&forge, &frame);
  return 0;
}

int LV2PluginHandler::pluginPodGetMidiOut(uint32_t portIdx, rawMidiOutCb_t cb, void *userData) {
  const LV2_Atom_Sequence *seq;

  if (!cb)
    return -EINVAL;

  for (struct midiPortDesc &midiPortDesc : midiPortDesc) {
    if (midiPortDesc.portInfo.index == portIdx) {
      seq = (LV2_Atom_Sequence *)midiPortDesc.portData;
      break;
    }
  }

  LV2_ATOM_SEQUENCE_FOREACH(seq, ev) {
    if (ev->body.type == lv2_urid_map(NULL, LV2_ATOM__Event)) {
      const uint8_t *eventData = (const uint8_t *)(ev + 1);
      cb(ev->time.frames, ev->body.size, eventData, userData);
    }
  }
  return 0;
}

int LV2PluginHandler::pluginDeactivate() {
  lilv_instance_deactivate(pluginInstance);
  return 0;
}

int LV2PluginHandler::pluginDestroy() {
  // Delete worker threads
  if (pWorkerHdl)
    delete pWorkerHdl;

  // Clear seq bufs
  for (void *buff : heapBuffers) {
    if (buff)
      free(buff);
  }

  lilv_instance_free(pluginInstance);
  return 0;
}

LV2_Worker_Status
LV2PluginHandler::workerStaticScheduler(LV2_Worker_Schedule_Handle handle,
                                        uint32_t size, const void *data) {
  Worker *pHdl = nullptr;

  if (!handle || !(pHdl = ((LV2PluginHandler *)handle)->pWorkerHdl))
    return LV2_WORKER_ERR_UNKNOWN;

  SYSLOG_DBG("Scheduling job..\n");
  pHdl->workerScheduleJob(size, data);
  return LV2_WORKER_SUCCESS;
}
