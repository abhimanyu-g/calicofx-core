#pragma once
#include <lilv/lilv.h>
#include <vector>
#include <string>

#include "basePluginHandler.hpp"

struct portDesc{
  std::string label;
  uint8_t index;
};

struct controlPortDesc{
  struct portDesc portInfo;
  float def, max, min, val;
  bool hasScalePoints;
};

// Global initialize variable
int pluginLv2Initialize();
int pluginLv2Deinitalize();

class LV2PluginHandler:public BasePluginHandler{
private:
  LilvInstance *instance;
  uint8_t nAudioInPorts, nAudioOutPorts, nControlPorts;
  std::string pluginName;
  std::vector<struct portDesc> audioInPortDesc, audioOutPortDesc;
  std::vector<struct controlPortDesc> controlPortDesc;

  // Private Prototypes
  void populatePorts(const LilvPlugin *plugin, uint8_t nPorts);

public:
  int pluginInit(std::string &pluginURI) override;
  int pluginActivateInstance() override;
  int pluginRun(int sampleRate) override;

  int pluginConnectPort(int portIdx, float &var) override;
  int pluginConnectPort(int portIdx, int &var) override;

  LV2PluginHandler();
  ~LV2PluginHandler();

  // TODO: Helper functions
};
