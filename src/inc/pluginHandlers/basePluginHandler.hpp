#ifndef BASEPLUGINHANDLER_H
#define BASEPLUGINHANDLER_H

#include <cstdint>
#include <string>
#include <vector>
#include "config.hpp"

enum portDirection { PORT_DIR_INPUT = 0, PORT_DIR_OUTPUT };
enum portType { PORT_TYPE_CONTROL = 0, PORT_TYPE_AUDIO, PORT_TYPE_MIDI };

struct portDesc {
  std::string label;
  uint8_t index;
  enum portDirection dir;
  enum portType type;
};

struct midiPortDesc {
  struct portDesc portInfo;
  uint8_t portData[DEFAULT_SEQ_BUF_SIZE];
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

/**
 * @brief      rawMidiOutCb
 * @details    Callback function pointer from Plugin Layer to the pipewire
               client layer to return raw MIDI events
 * @param[in] offset Timing offset of the event in current block
 * @param[in] size Event data size
 * @param[in] data event data of size `size`
 * @param[in] userData User data to be passed back during the callback
 * @returns   result
 */
typedef int (*rawMidiOutCb_t)(uint32_t offset, uint32_t size,
                              const void *evData, void *userData);

class PluginBase {
public:
  std::string pluginName;
  std::vector<struct portDesc> audioInPortDesc;
  std::vector<struct portDesc> audioOutPortDesc;
  std::vector<struct midiPortDesc> midiPortDesc;
  std::vector<struct controlPortDesc> controlPortDesc;

  // plugin prototypes ////////////////////////////////////////////////////////e
  virtual int pluginInit(void *pluginURI) = 0;
  virtual int pluginActivate() = 0;

  /**
   * @brief      Connecting Audio ports
   * @details    Connect Audio ports from pipewire-client to the plugin ports
   * @param[in]  portIdx Index of the port as described in the plugin
   * @param[in]  buf pw-client buffer as seen during `on_process` operation
   * @returns    0 on success, -ve value otherwise
   */
  virtual int pluginConnectPort(uint8_t portIdx, void *buf) = 0;
  virtual int pluginRun(int nSamples) = 0;
  virtual int pluginUpdateParam(uint8_t idx, float val) = 0;

  virtual int pluginDeactivate() = 0;
  virtual int pluginDestroy() = 0;

  // Plain-old-data (POD) events. (say MIDI)
  virtual int pluginInitPod(uint8_t portIdx) = 0;
  virtual int pluginAppendPodEvent(size_t offset, uint32_t len,
                                   const uint8_t *data) = 0;
  virtual int pluginPodFinalize() = 0;

  // Each POD type would have a dedicated callback ////////////////////////////
  /**
   * @brief      pluginPodGetMidiOut
   * @details    Works on each raw MIDI event of the sequence and calls the `cb` for every
                 midi event passing user args. This is expected to be used by pw-client
   * @param[in]  cb User callback for each Raw MIDI event
   * @param[in]  userData userData to be passed to the plugin
   * @returns    0 on success -ve otherwise
   */
  virtual int pluginPodGetMidiOut(uint32_t portIdx, rawMidiOutCb_t cb,
                                  void *userData) = 0;
};

#endif /* BASEPLUGINHANDLER_H */
