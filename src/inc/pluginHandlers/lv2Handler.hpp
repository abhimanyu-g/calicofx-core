#pragma once
#include <lilv/lilv.h>
#include <vector>
#include <string>

#include "basePluginHandler.hpp"

// Global initialize variable
/**
 * @brief      Initialize Lv2 plugin handler
 * @details    The function is responsible to initialize
 *             the lv2 world and load all its plugins
 * @param      void
 * @return     < 0 for failure, 0 otherwise
 */
int pluginLv2Initialize();
int pluginLv2Deinitalize();

class LV2PluginHandler : public PluginBase {
  
public:      
  int pluginInit(void *pluginURI) override;
  int pluginActivate() override;
  int pluginConnectPort(uint8_t portIdx, float *buf) override;
  int pluginRun(int sampleRate) override;
  int pluginUpdateParam(uint8_t idx, float val) override;
  int pluginDeactivate() override;
  int pluginDestroy() override;

private:
  std::string uri;
  LilvNode *pluginUriNode;
  LilvNode *pluginNode;
  LilvInstance *pluginInstance;
  void populatePorts(const LilvPlugin *plugin,
                                       uint8_t nPorts);
};
