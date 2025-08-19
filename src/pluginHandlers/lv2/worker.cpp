#include <cstdlib>
#include <mutex>
#include <thread>

#include <lv2/worker/worker.h>

#include "common.hpp"
#include "pluginHandlers/lv2/worker.hpp"

Worker::Worker(const void *iface, LV2_Handle hdl)
    : iface_((const LV2_Worker_Interface *)iface), pluginHdl_(hdl),
      isRunning_(true) {
  thread_ = std::thread(&Worker::workerThread, this);
}

Worker::~Worker() { workerStop(); }

LV2_Worker_Status Worker::workerScheduleJob(uint32_t size, const void *data) {
  void *job = malloc(size);
  if (!job)
    return LV2_WORKER_ERR_NO_SPACE;

  std::memcpy(job, data, size);
  std::lock_guard<std::mutex> lock(mutex_);
  jobQueue_.push(job);
  cond_.notify_one();

  return LV2_WORKER_SUCCESS;
}

void Worker::workerStop() {
  if (!isRunning_)
    return;

  std::unique_lock<std::mutex> lock(mutex_);
  isRunning_ = false;
  lock.unlock();

  cond_.notify_one();
  if (thread_.joinable())
    thread_.join();

  while (!jobResponseQueue_.empty()) {
    // Flush responses
    workerJobResponse();
  }
}

void Worker::workerJobResponse() {
  std::unique_lock<std::mutex> lock(mutex_);
  // To be called as a job response in Plugin's Run()
  if (!jobResponseQueue_.empty()) {
    struct workerRespDesc respDesc = jobResponseQueue_.front();
    jobResponseQueue_.pop();
    iface_->work_response(pluginHdl_, respDesc.size, respDesc.data);

    if (respDesc.data)
      free(respDesc.data);
  }

  if (iface_->end_run)
    iface_->end_run(pluginHdl_);
}

void Worker::workerThread() {
  while (true) {
    void *job = nullptr;
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [&] { return !jobQueue_.empty() || !isRunning_; });

    if (!isRunning_ && jobQueue_.empty())
      break;

    job = jobQueue_.front();
    jobQueue_.pop();
    lock.unlock();

    SYSLOG_DBG("Running job...\n");

    // Run plugin's worker
    iface_->work(pluginHdl_, &Worker::workerPluginRespond, this, 0, job);
    free(job);
  }
}

LV2_Worker_Status Worker::workerPluginRespond(LV2_Worker_Respond_Handle handle,
                                              const uint32_t size,
                                              const void *data) {
  struct workerRespDesc respDesc;
  Worker *pWorkerHdl = (Worker *)handle;
  if (!pWorkerHdl)
    return LV2_WORKER_ERR_UNKNOWN;

  std::unique_lock<std::mutex> lock(pWorkerHdl->mutex_);
  void *respData = malloc(size);
  if (!respData)
    return LV2_WORKER_ERR_NO_SPACE;

  memcpy(respData, data, size);
  respDesc.data = respData;
  respDesc.size = size;
  pWorkerHdl->jobResponseQueue_.push(std::move(respDesc));

  return LV2_WORKER_SUCCESS;
}
