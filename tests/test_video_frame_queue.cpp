#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"
#include "joint_sensor_orchestrator/video_frame_queue.hpp"
#include "test_framework.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <thread>

namespace {

joint_sensor_orchestrator::QueuedVideoFrame frame(std::uint64_t index) {
  joint_sensor_orchestrator::QueuedVideoFrame value;
  value.slot.output_index = index;
  value.bytes =
      std::make_shared<const joint_sensor_orchestrator::VideoFrameBuffer>(4);
  return value;
}

}  // namespace

using joint_sensor_orchestrator::VideoFrameQueue;

TEST_CASE(video_queue_enforces_capacity_and_tracks_waterline) {
  VideoFrameQueue queue(3);
  CHECK(queue.push(frame(0)));
  CHECK(queue.push(frame(1)));
  CHECK(queue.push(frame(2)));
  CHECK(queue.size() == 3);
  CHECK(queue.max_waterline() == 3);

  auto blocked = std::async(std::launch::async, [&] {
    return queue.push(frame(3));
  });
  CHECK(blocked.wait_for(std::chrono::milliseconds(20)) ==
        std::future_status::timeout);
  CHECK(queue.pop().has_value());
  CHECK(blocked.get());
  CHECK(queue.size() == 3);
}

TEST_CASE(video_queue_close_signals_eof_after_drain) {
  VideoFrameQueue queue(3);
  CHECK(queue.push(frame(7)));
  queue.close();

  const auto item = queue.pop();
  CHECK(item.has_value());
  CHECK(item->slot.output_index == 7);
  CHECK(!queue.pop().has_value());
  CHECK(queue.eof());
  CHECK(!queue.push(frame(8)));
}

TEST_CASE(video_queue_cancellation_wakes_blocked_push_and_pop) {
  VideoFrameQueue full_queue(3);
  CHECK(full_queue.push(frame(0)));
  CHECK(full_queue.push(frame(1)));
  CHECK(full_queue.push(frame(2)));
  auto push = std::async(std::launch::async, [&] {
    return full_queue.push(frame(3));
  });

  VideoFrameQueue empty_queue(3);
  auto pop = std::async(std::launch::async, [&] {
    return empty_queue.pop();
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  full_queue.cancel();
  empty_queue.cancel();

  CHECK(push.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  CHECK(!push.get());
  CHECK(pop.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  CHECK(!pop.get().has_value());
}

TEST_CASE(video_queue_cancellation_stops_consuming_buffered_frames) {
  VideoFrameQueue queue(3);
  CHECK(queue.push(frame(0)));
  queue.cancel();
  CHECK(!queue.pop().has_value());
  CHECK(queue.size() == 1);
}

TEST_CASE(video_queue_immediate_cancel_has_no_lost_wakeup_under_contention) {
  for (unsigned iteration = 0; iteration < 500; ++iteration) {
    VideoFrameQueue empty(3);
    auto pop = std::async(std::launch::async, [&] { return empty.pop(); });
    empty.cancel();
    const auto pop_status = pop.wait_for(std::chrono::milliseconds(20));
    if (pop_status != std::future_status::ready) {
      empty.cancel();
    }
    CHECK(pop_status == std::future_status::ready);
    CHECK(!pop.get().has_value());

    VideoFrameQueue full(3);
    CHECK(full.push(frame(0)));
    CHECK(full.push(frame(1)));
    CHECK(full.push(frame(2)));
    auto push =
        std::async(std::launch::async, [&] { return full.push(frame(3)); });
    full.cancel();
    const auto push_status = push.wait_for(std::chrono::milliseconds(20));
    if (push_status != std::future_status::ready) {
      full.cancel();
    }
    CHECK(push_status == std::future_status::ready);
    CHECK(!push.get());
  }
}

TEST_CASE(video_queue_rejects_capacity_outside_three_to_six) {
  CHECK_THROWS(VideoFrameQueue(2));
  CHECK_THROWS(VideoFrameQueue(7));
}

TEST_CASE(frame_buffer_pool_allocates_lazily_but_never_beyond_capacity) {
  joint_sensor_orchestrator::FrameBufferPool pool(3, 16);
  joint_sensor_player::CancellationToken cancellation;
  CHECK(pool.allocated_buffers() == 0);
  auto first = pool.acquire(cancellation);
  CHECK(first && first->size() == 16);
  CHECK(pool.allocated_buffers() == 1);
  auto second = pool.acquire(cancellation);
  auto third = pool.acquire(cancellation);
  CHECK(second && third);
  CHECK(pool.allocated_buffers() == 3);
  CHECK(pool.max_in_use() == 3);
}
