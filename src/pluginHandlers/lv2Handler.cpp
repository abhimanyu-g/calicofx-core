#include <errno.h>
#include <limits>
#include <vector>

#include "common.hpp"
#include "lilv/lilv.h"
#include "pluginHandlers/lv2Handler.hpp"

using namespace std;

static LilvWorld *world = nullptr;
static LilvPlugins *plugins = nullptr;
static LilvNode *lilv_audio_class = nullptr;
static LilvNode *lilv_input_class = nullptr;
static LilvNode *lilv_output_class = nullptr;
static LilvNode *lilv_control_class = nullptr;
static LilvNode *lilv_scale_point_class = nullptr;

static vector<string> nodesURI;

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
    SYSLOG_DBG("%s\n", node.c_str());
  }

  lilv_audio_class = lilv_new_uri(world, LV2_CORE__AudioPort);
  lilv_input_class = lilv_new_uri(world, LV2_CORE__InputPort);
  lilv_output_class = lilv_new_uri(world, LV2_CORE__OutputPort);
  lilv_control_class = lilv_new_uri(world, LV2_CORE__ControlPort);
  lilv_scale_point_class = lilv_new_uri(world, LV2_CORE__ScalePoint);
  SYSLOG_DBG("LV2 plugin library initialized\n");
  return 0;
}

int pluginLv2Deinitalize() {
  // TODO: Have clean-up routine
  return 0;
}


void LV2PluginHandler::populatePorts(const LilvPlugin *plugin, uint8_t nPorts) {
  nAudioInPorts = 0;
  nAudioOutPorts = 0;
  nControlPorts = 0;

  for (uint8_t idx = 0; idx < nPorts; ++idx) {
    // nPorts would always have [0...nPorts] sequentially
    struct portDesc desc = {};
    struct controlPortDesc ctrlPortDesc = {};
    const LilvPort *port = lilv_plugin_get_port_by_index(plugin, idx);
    LilvNode *portName = lilv_port_get_name(plugin, port);

    desc.label = string(lilv_node_as_string(portName));
    desc.index = idx;

    // audio ports
    if (lilv_port_is_a(plugin, port, lilv_audio_class)) {
      if (lilv_port_is_a(plugin, port, lilv_input_class)) {
        // audio-in ports
        this->audioInPortDesc.push_back(desc);
        this->nAudioInPorts++;
      } else {
        // audio-out ports
        this->audioOutPortDesc.push_back(desc);
        this->nAudioOutPorts++;
      }
    }

      // control ports
    else if (lilv_port_is_a(plugin, port, lilv_control_class)) {
      LilvNode *def, *min, *max;
      LilvScalePoints *sp;

      ctrlPortDesc.portInfo = desc;
      ctrlPortDesc.def = ctrlPortDesc.min = ctrlPortDesc.max =
        ctrlPortDesc.val = numeric_limits<float>::max();
      ctrlPortDesc.hasScalePoints = false;

      this->nControlPorts++;
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

      sp = lilv_port_get_scale_points(plugin, port);
      if (sp) {
        ctrlPortDesc.hasScalePoints = true;
        // TODO: Parse and add scale points
        lilv_scale_points_free(sp);
      }
    }
    lilv_node_free(portName);
  }
}

int LV2PluginHandler::pluginRun( int sampleRate) {
  lilv_instance_run(this->pluginInstance, sampleRate);
  return 0;
}

int LV2PluginHandler::pluginConnectPort(uint8_t portIdx, float *buf) {
  lilv_instance_connect_port
    (this->pluginInstance, portIdx, buf);
  return 0;
}

int LV2PluginHandler::pluginActivate() {
  lilv_instance_activate(this->pluginInstance);
  return 0;
}

int LV2PluginHandler::pluginInit(void *uri) {
  uint8_t nPorts = 0;
  string lv2PluginURI = (const char*)uri;
  LilvNode *node;
  pluginUriNode = lilv_new_uri(world, lv2PluginURI.c_str());
  const LilvPlugin *plugin = lilv_plugins_get_by_uri(plugins, pluginUriNode);

  if (!plugin) {
    SYSLOG_ERR("plugin URI [%s] not found\n", lv2PluginURI.c_str());
    lilv_node_free(pluginUriNode);
    return -EAGAIN;
  }

  pluginNode = lilv_plugin_get_name(plugin);
  if (pluginNode) {
    this->pluginName = string(lilv_node_as_string(node));
    lilv_node_free(pluginNode);
  }

  nPorts = lilv_plugin_get_num_ports(plugin);
  populatePorts(plugin, nPorts);
  lilv_node_free(pluginUriNode);

  return 0;
}

int LV2PluginHandler::pluginUpdateParam(uint8_t idx, float val) {return 0;}

int LV2PluginHandler::pluginDeactivate() {return 0;}

int LV2PluginHandler::pluginDestroy() {return 0;}
