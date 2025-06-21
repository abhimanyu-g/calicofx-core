#pragma once
#include <lilv/lilv.h>
#include <vector>
#include <string>

struct portDesc{
  std::string label;
  uint8_t index;
};

struct controlPortDesc{
  struct portDesc portInfo;
  float def, max, min, val;
  bool hasScalePoints;
};

class LV2PluginHandler {
private:
  LilvInstance *instance;
  uint8_t nAudioInPorts, nAudioOutPorts, nControlPorts;
  std::string pluginName;
  std::vector<struct portDesc> audioInPortDesc, audioOutPortDesc;
  std::vector<struct controlPortDesc> controlPortDesc;

  // Private Prototypes
  void populatePorts(const LilvPlugin *plugin, uint8_t nPorts);
  void connectControlPorts(void);
public:
  /**
   * @brief      Constructor
   * @details    Initializes lilvplugin library
   * @param      URI: Plugin's unique identifier
   * @param      sample_rate: rate of audio sampling
   * @return     N/A
   */
  LV2PluginHandler(const std::string &URI, double sample_rate);

  /**
   * @brief      Destructor
   * @details    Un-initializes Lilv library
   * @param      void
   * @return     N/A
   */
  //  ~LV2PluginHandler();

  /**
   * @brief      Helper function to Activate plugin
   * @details    Activate plugin and start processing the data
   * @param      void
   * @return     void
   */
  void activateInstance();

  std::string getPluginName() {
    return this->pluginName;
  }

  /**
   * @brief      Initialize Lv2 plugin handler
   * @details    The function is responsible to initialize
   *             the lv2 world and load all its plugins
   * @param      void
   * @return     < 0 for failure, 0 otherwise
   */
  static int initalizeLv2Lib();

};
