#pragma once
#include <lilv/lilv.h>
#include <lv2/atom/atom.h>
#include <lv2/atom/forge.h>
#include <lv2/core/lv2.h>
#include <string>
#include <vector>

#include "pluginHandlers/basePluginHandler.hpp"
#include "pluginHandlers/lv2/worker.hpp"

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
  int pluginConnectPort(uint8_t portIdx, void *buf) override;
  int pluginRun(int sampleRate) override;
  int pluginUpdateParam(uint8_t idx, float val) override;

  int pluginDeactivate() override;
  int pluginDestroy() override;

  // Plain-old-data (POD) events. (say MIDI)
  int pluginInitPod(uint8_t portIdx) override;
  int pluginAppendPodEvent(size_t offset, uint32_t len,
                           const uint8_t *data) override;
  int pluginPodFinalize() override;
  int pluginPodGetMidiOut(uint32_t portIdx, rawMidiOutCb_t cb, void* userData) override;

private:
  Worker *pWorkerHdl;
  std::string pluginURI;
  std::vector<void *> heapBuffers;
  std::unique_ptr<Worker> *worker;
  LilvInstance *pluginInstance;
  LV2_Atom_Forge forge;
  LV2_Atom_Forge_Frame frame;
  LV2_Atom_Sequence *seq;
  void populatePorts(const LilvPlugin *plugin);
  static LV2_Worker_Status
  workerStaticScheduler(LV2_Worker_Schedule_Handle handle, uint32_t size,
                        const void *data);
};
