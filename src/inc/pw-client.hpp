#pragma once
#include <vector>
#include <string>
#include <pipewire/pipewire.h>
#include <pipewire/filter.h>

#if 0
struct bufferDesc{
  uint8_t nInputs;
  uint8_t nOuptputs;
  std::vector<float *> inBuff;
  std::vector<float *> outBuff;
};

typedef enum actions{
  ACTION_CREATE_INSTANCE = 0,
  ACTION_CONNECT_PORTS,
  ACTION_DISCONNECT_PORTS,
  ACTION_UPDATE_CTRL_PARAM,
  ACTION_DELETE_INSTANCE
}action_t;

int pwInitializeLib();
int pwDeinitalizeLib();
void pwRunMainLoop();

int pwActionDispatchHandler(action_t dispatchAction, void *payload);

class PipeWireClient{
private:
  struct pw_filter *filter;

  typedef int (*process_cb)(struct bufferDesc *, void *);

  /**
   * @brief      Pipewire process callback
   * @details    Process callback to modify data
   * @param      userData : Custom user data passed during creation
   * @param      position : TBD
   * @return     void
   */
  static void on_process(void *userdata, struct spa_io_position *position);

  // Custom callback
  process_cb *runProcessCb = nullptr;

public:
  /**
   * @brief      Constructor
   * @details    Create a pipewire filter node and plug into the process block
   * @param      name: Plugin name as seen in lv2
   * @param      useData : Custom user data to be later passed in process callback
   * @return     N/A
   */
  PipeWireClient(const std::string &name, void *userData);

  //void registerProcessCb(process_cb cb) { runProcessCb = &cb; }


  /**
   * @brief      Initialize pipewire library
   * @details    One time initialization of Pipewire constructs
   * @param      void
   * @return     <0 for error, 0 otherwise
   */
  static int initializePwLib();

};

#endif
// Platform setup and teardown ////////////////////////////////////////////////
int initializePwLib();
int teardownPwLib();

enum pluginType { PLUGIN_TYPE_LV2 = 0 };

class PipewireClient {
private:
  struct pw_filter *filter;
  class PluginBase *pluginMgr;
  
public:
  int pwInitClient(std::string uri, enum pluginType pluginType);
  int pwUpdateClientParam(int clientPortIdx, float value);
  static int pwLinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                               std::string dstNodeUUID, int dstPortIdx);

  static int pwUnlinkClientPorts(std::string srcNodeUUID, int srcPortIdx,
                                 std::string dstNodeUUID, int dstPortIdx);
    
  ~PipewireClient();
};
