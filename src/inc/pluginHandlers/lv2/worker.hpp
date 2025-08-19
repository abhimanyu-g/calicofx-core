#pragma once

#include <condition_variable>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <lv2/core/lv2.h>
#include <lv2/worker/worker.h>

struct workerRespDesc {
  void *data;
  uint32_t size;
};

class Worker {
public:
  Worker(const void *iface, LV2_Handle hdl);
  ~Worker();
  LV2_Worker_Status workerScheduleJob(uint32_t size, const void *data);
  void workerJobResponse();
  void workerStop();

private:
  const LV2_Worker_Interface *iface_;
  LV2_Handle pluginHdl_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cond_;
  std::queue<void *> jobQueue_;
  std::queue<struct workerRespDesc> jobResponseQueue_;
  bool isRunning_;

  LV2_Worker_Schedule workerScheduleIface_;
  LV2_Feature workerFeat_;

  void workerThread();
  static LV2_Worker_Status workerPluginRespond(LV2_Worker_Respond_Handle handle,
                                               const uint32_t size,
                                               const void *data);
};
