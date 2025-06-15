#include <gtest/gtest.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"
#include <chrono>
#include <thread>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

class BagWorkerTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    data_generator_ = std::make_unique<TestDataGenerator>();
    
    // Create a unique test bag name for each test
    auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string bag_name = std::string("worker_") + test_info->test_case_name() + "_" + test_info->name() + ".db3";
    
    // Create a test bag and build index
    test_bag_path_ = bag_creator_->create_test_bag(bag_name, 20, {"/test_topic"});
    index_manager_ = std::make_unique<IndexManager>();
    index_manager_->build_index({test_bag_path_});
    
    // Create cache and worker
    cache_ = std::make_shared<MessageCache>(100);
    worker_ = std::make_unique<BagWorker>(*index_manager_);
    worker_->set_cache(cache_);
  }

  void TearDown() override {
    if (worker_) {
      worker_->stop();
    }
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<TestDataGenerator> data_generator_;
  std::unique_ptr<IndexManager> index_manager_;
  std::shared_ptr<MessageCache> cache_;
  std::unique_ptr<BagWorker> worker_;
  std::string test_bag_path_;
};

TEST_F(BagWorkerTest, BasicConstruction) {
  // Worker should be constructible with index manager
  EXPECT_NO_THROW(BagWorker worker(*index_manager_));
}

TEST_F(BagWorkerTest, StartAndStop) {
  // Start worker
  EXPECT_NO_THROW(worker_->start());
  
  // Should be able to start multiple times (idempotent)
  EXPECT_NO_THROW(worker_->start());
  
  // Stop worker
  EXPECT_NO_THROW(worker_->stop());
  
  // Should be able to stop multiple times (idempotent)
  EXPECT_NO_THROW(worker_->stop());
}

TEST_F(BagWorkerTest, RequestRange) {
  worker_->start();
  
  // Request a range of messages
  auto future = worker_->request_range(0, 5);
  
  // Should complete successfully
  EXPECT_TRUE(future.get());
  
  // Messages should be in cache
  for (size_t i = 0; i <= 5; ++i) {
    EXPECT_TRUE(cache_->is_frame_cached(i));
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, RequestSeek) {
  worker_->start();
  
  if (index_manager_->total_frames() > 0) {
    // Request seek to start time
    auto start_time = index_manager_->start_time();
    auto future = worker_->request_seek(start_time);
    
    // Should complete successfully
    EXPECT_TRUE(future.get());
    
    // Frame 0 should be in cache
    EXPECT_TRUE(cache_->is_frame_cached(0));
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, SetCache) {
  // Create new cache
  auto new_cache = std::make_shared<MessageCache>(50);
  
  // Set cache
  EXPECT_NO_THROW(worker_->set_cache(new_cache));
  
  worker_->start();
  
  // Request range
  auto future = worker_->request_range(0, 3);
  future.get();
  
  // Messages should be in new cache
  for (size_t i = 0; i <= 3; ++i) {
    EXPECT_TRUE(new_cache->is_frame_cached(i));
  }
  
  // Old cache should be empty
  for (size_t i = 0; i <= 3; ++i) {
    EXPECT_FALSE(cache_->is_frame_cached(i));
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, MultipleRequests) {
  worker_->start();
  
  // Submit multiple requests
  std::vector<std::future<bool>> futures;
  
  futures.push_back(worker_->request_range(0, 3));
  futures.push_back(worker_->request_range(5, 8));
  futures.push_back(worker_->request_range(10, 13));
  
  // All should complete successfully
  for (auto& future : futures) {
    EXPECT_TRUE(future.get());
  }
  
  // All requested frames should be cached
  for (size_t i = 0; i <= 3; ++i) {
    EXPECT_TRUE(cache_->is_frame_cached(i));
  }
  for (size_t i = 5; i <= 8; ++i) {
    EXPECT_TRUE(cache_->is_frame_cached(i));
  }
  for (size_t i = 10; i <= 13; ++i) {
    EXPECT_TRUE(cache_->is_frame_cached(i));
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, LargeRangeRequest) {
  worker_->start();
  
  // Request entire bag
  size_t total_frames = index_manager_->total_frames();
  if (total_frames > 0) {
    auto future = worker_->request_range(0, total_frames - 1);
    EXPECT_TRUE(future.get());
    
    // All frames should be cached (up to cache limit)
    size_t cached_count = 0;
    for (size_t i = 0; i < total_frames; ++i) {
      if (cache_->is_frame_cached(i)) {
        cached_count++;
      }
    }
    EXPECT_GT(cached_count, 0);
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, InvalidRangeRequest) {
  worker_->start();
  
  // Request invalid range (beyond available frames)
  size_t total_frames = index_manager_->total_frames();
  auto future = worker_->request_range(total_frames + 10, total_frames + 20);
  
  // Should handle gracefully (may return false or complete without caching)
  EXPECT_NO_THROW(future.get());
  
  worker_->stop();
}

TEST_F(BagWorkerTest, ConcurrentRequests) {
  worker_->start();
  
  // Launch multiple threads making requests
  std::vector<std::thread> threads;
  std::vector<bool> results(4, false);
  
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([this, i, &results]() {
      auto future = worker_->request_range(i * 3, i * 3 + 2);
      results[i] = future.get();
    });
  }
  
  // Wait for all threads
  for (auto& thread : threads) {
    thread.join();
  }
  
  // All requests should succeed
  for (bool result : results) {
    EXPECT_TRUE(result);
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, WorkerThreadLifecycle) {
  // Worker should handle start/stop cycles
  for (int cycle = 0; cycle < 3; ++cycle) {
    worker_->start();
    
    // Make a request
    auto future = worker_->request_range(0, 2);
    EXPECT_TRUE(future.get());
    
    worker_->stop();
  }
}

TEST_F(BagWorkerTest, StopDuringWork) {
  worker_->start();
  
  // Start a large request
  auto future = worker_->request_range(0, 19);
  
  // Stop worker while working
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  worker_->stop();
  
  // Future should complete (may be true or false)
  EXPECT_NO_THROW(future.get());
}

TEST_F(BagWorkerTest, WorkerWithEmptyIndex) {
  // Create empty index
  auto empty_index = std::make_unique<IndexManager>();
  auto empty_bag = bag_creator_->create_test_bag("empty_worker.db3", 0, {});
  empty_index->build_index({empty_bag});
  
  // Create worker with empty index
  auto empty_worker = std::make_unique<BagWorker>(*empty_index);
  empty_worker->set_cache(cache_);
  
  empty_worker->start();
  
  // Request should complete (likely false due to no data)
  auto future = empty_worker->request_range(0, 5);
  EXPECT_NO_THROW(future.get());
  
  empty_worker->stop();
}

TEST_F(BagWorkerTest, MultipleSeekRequests) {
  worker_->start();
  
  if (index_manager_->total_frames() > 5) {
    // Make multiple seek requests
    std::vector<std::future<bool>> futures;
    
    auto start_time = index_manager_->start_time();
    auto end_time = index_manager_->end_time();
    auto duration = end_time - start_time;
    
    // Seek to different time points
    futures.push_back(worker_->request_seek(start_time));
    futures.push_back(worker_->request_seek(start_time + duration / 4));
    futures.push_back(worker_->request_seek(start_time + duration / 2));
    futures.push_back(worker_->request_seek(start_time + 3 * duration / 4));
    
    // All should complete
    for (auto& future : futures) {
      EXPECT_NO_THROW(future.get());
    }
  }
  
  worker_->stop();
}

TEST_F(BagWorkerTest, RequestWithNullCache) {
  // Create worker without cache
  auto worker_no_cache = std::make_unique<BagWorker>(*index_manager_);
  
  worker_no_cache->start();
  
  // Request should handle null cache gracefully
  auto future = worker_no_cache->request_range(0, 3);
  EXPECT_NO_THROW(future.get());
  
  worker_no_cache->stop();
}

TEST_F(BagWorkerTest, HighFrequencyRequests) {
  worker_->start();
  
  // Submit many small requests rapidly
  std::vector<std::future<bool>> futures;
  
  for (int i = 0; i < 10; ++i) {
    futures.push_back(worker_->request_range(i, i));
  }
  
  // All should complete
  int successful = 0;
  for (auto& future : futures) {
    if (future.get()) {
      successful++;
    }
  }
  
  EXPECT_GT(successful, 0);
  
  worker_->stop();
}

TEST_F(BagWorkerTest, WorkerDestructorWhileRunning) {
  {
    auto temp_worker = std::make_unique<BagWorker>(*index_manager_);
    temp_worker->set_cache(cache_);
    temp_worker->start();
    
    // Start some work
    auto future = temp_worker->request_range(0, 5);
    
    // Worker destructor should clean up properly
  }
  
  // This test mainly verifies no crashes during cleanup
  SUCCEED();
}