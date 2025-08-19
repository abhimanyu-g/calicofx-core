#ifndef BASEPLUGINHANDLER_H
#define BASEPLUGINHANDLER_H

#include <cstdint>
#include <string>
#include <vector>

struct portDesc {
  std::string label;
  uint8_t index;
};

struct scalePointsDesc {
  std::string label;
  float val;
};

struct controlPortDesc {
  struct portDesc portInfo;
  float def;
  float max;
  float min;
  float val;
  bool hasScalePoints;
  std::vector<struct scalePointsDesc> scalePoints;
};

class PluginBase {
public:
  std::string pluginName;
  uint8_t nAudioInPorts;
  uint8_t nAudioOutPorts;
  uint8_t nControlPorts;
  std::vector<struct portDesc> audioInPortDesc;
  std::vector<struct portDesc> audioOutPortDesc;
  std::vector<struct portDesc> midiPortDesc;
  std::vector<struct controlPortDesc> controlPortDesc;

  // plugin prototypes ////////////////////////////////////////////////////////e
  virtual int pluginInit(void *pluginURI) = 0;
  virtual int pluginActivate() = 0;
  virtual int pluginConnectPort(uint8_t portIdx, void *buf) = 0;
  virtual int pluginRun(int nSamples) = 0;
  virtual int pluginUpdateParam(uint8_t idx, float val) = 0;
  virtual int pluginDeactivate() = 0;
  virtual int pluginDestroy() = 0;
};

#endif /* BASEPLUGINHANDLER_H */
