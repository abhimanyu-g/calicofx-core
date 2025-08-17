#include <errno.h>
#include <lilv/lilv.h>
#include <limits>
#include <vector>

#include "common.hpp"
#include "config.hpp"
#include "pluginHandlers/basePluginHandler.hpp"
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
  lilv_node_free(lilv_audio_class);
  lilv_node_free(lilv_input_class);
  lilv_node_free(lilv_output_class);
  lilv_node_free(lilv_control_class);
  lilv_node_free(lilv_scale_point_class);

  lilv_world_free(world);
  SYSLOG_DBG("LV2 plugin library de-initialized\n");
  return 0;
}

void LV2PluginHandler::populatePorts(const LilvPlugin *plugin) {
  int nPorts = -1;
  nAudioInPorts = 0;
  nAudioOutPorts = 0;
  nControlPorts = 0;

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
      struct controlPortDesc ctrlPortDesc = {};

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
      nControlPorts++;
    }
  }
}

int LV2PluginHandler::pluginRun(int nSamples) {
  lilv_instance_run(this->pluginInstance, nSamples);
  return 0;
}

int LV2PluginHandler::pluginConnectPort(uint8_t portIdx, float *buf) {
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

  // Instantiate plugin
  pluginInstance = lilv_plugin_instantiate(plugin, DEFAULT_SAMPLE_RATE, NULL);
  if (!pluginInstance) {
    SYSLOG_ERR("Failed to instantiate plugin [%s]. Try again\n",
               pluginName.c_str());
    return -EAGAIN;
  }

  populatePorts(plugin);
  SYSLOG_DBG("Added plugin [%s] successfully\n", pluginName.c_str());
  return 0;
}

int LV2PluginHandler::pluginUpdateParam(uint8_t idx, float val) { return 0; }

int LV2PluginHandler::pluginDeactivate() {
  lilv_instance_deactivate(pluginInstance);
  return 0;
}

int LV2PluginHandler::pluginDestroy() {
  lilv_instance_free(pluginInstance);
  ;
  return 0;
}
