#pragma once

#include "index_manager.hpp"
#include "message_cache.hpp"
#include "types.hpp"
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <rosbag2_cpp/reader.hpp>

namespace rosbag_deck_core {

class BagWorker {
public:
  BagWorker(const IndexManager &index_manager);
  ~BagWorker();

  void start();
  void stop();
  std::future<bool> request_range(size_t start_frame, size_t end_frame);
  std::future<bool> request_seek(const Timestamp &target_time);
  void set_cache(std::shared_ptr<MessageCache> cache);

private:
  void worker_thread();
  void process_request(const CacheRequest &request);
  BagMessage load_message_at_frame(size_t frame_index);
  void open_bag_readers();

  const IndexManager &index_manager_;
  std::shared_ptr<MessageCache> cache_;
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> readers_;

  std::thread worker_thread_;
  std::queue<CacheRequest> request_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> should_stop_;
};

} // namespace rosbag_deck_core