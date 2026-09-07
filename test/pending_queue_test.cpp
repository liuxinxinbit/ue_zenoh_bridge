#include "pending_queue.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

struct Sample { std::string key; std::size_t payload_size; int sequence; };
using ue_bridge::PendingQueue;
using ue_bridge::PushResult;
using ue_bridge::QueueBudget;

void check(bool condition, const char * message)
{
  if (!condition) { throw std::runtime_error(message); }
}

int main()
{
  QueueBudget budget(100);
  {
    PendingQueue<Sample> queue(budget, 2, 3);
    for (int i = 0; i < 3; ++i) {
      check(queue.push({"lidar", 10, i}, true) == PushResult::accepted, "FIFO accepts burst");
    }
    check(queue.push({"lidar", 10, 3}, true) == PushResult::full, "FIFO bound rejects new frame");
    check(queue.push({"imu", 5, 0}, false) == PushResult::accepted, "independent topic");
    check(queue.push({"imu", 7, 1}, false) == PushResult::replaced, "IMU latest policy");
    check(budget.used() == 37, "replacement byte accounting");
    check(queue.push({"third", 1, 0}, false) == PushResult::full, "distinct key bound");
    check(queue.pop().sequence == 0, "oldest lidar retained on overflow");
    auto imu = queue.pop();
    check(imu.key == "imu" && imu.sequence == 1, "round robin fairness and latest IMU");
    check(queue.pop().sequence == 1 && queue.pop().sequence == 2, "FIFO ordering");
    check(queue.empty() && budget.used() == 0, "draining releases bytes");
  }
  {
    PendingQueue<Sample> a(budget, 3, 30), b(budget, 3, 30);
    check(a.push({"front", 60, 0}, true) == PushResult::accepted, "first lane budget");
    check(b.push({"rear", 41, 0}, true) == PushResult::full, "shared budget across lanes");
    check(b.push({"imu", 40, 0}, false) == PushResult::accepted, "exact byte capacity");
    check(b.push({"imu", 41, 1}, false) == PushResult::full, "replacement cannot exceed budget");
    check(b.push({"imu", 30, 2}, false) == PushResult::replaced, "smaller replacement");
    check(budget.used() == 90, "shrinking releases bytes");
    a.pop();
    check(b.push({"rear", 60, 1}, true) == PushResult::accepted, "budget reusable after pop");
  }
  check(budget.used() == 0, "destruction releases all remaining frames");
  {
    // A batched second of all scalar sensors must be retained, in order.
    QueueBudget recording_budget(1u << 20);
    PendingQueue<Sample> queue(recording_budget, 4, 500);
    for (int i = 0; i < 500; ++i) {
      check(queue.push({"imu", 128, i}, true) == PushResult::accepted, "recording IMU FIFO");
      if (i % 5 == 0) {
        check(queue.push({"gps", 256, i / 5}, true) == PushResult::accepted, "recording GPS FIFO");
        check(queue.push({"odom", 512, i / 5}, true) == PushResult::accepted, "recording odom FIFO");
      }
    }
    check(queue.push({"imu", 128, 500}, true) == PushResult::full, "recording FIFO explicitly rejects overflow");
    int imu = 0, gps = 0, odom = 0;
    while (!queue.empty()) {
      const auto sample = queue.pop();
      int & next = sample.key == "imu" ? imu : (sample.key == "gps" ? gps : odom);
      check(sample.sequence == next++, "all recording topics retain order without replacement");
    }
    check(imu == 500 && gps == 100 && odom == 100, "all accepted sensor samples drain exactly once");
    check(recording_budget.used() == 0, "recording drain releases byte budget");
  }
  std::cout << "FIFO burst recording, overflow, fairness, legacy latest policy and shared byte budget: PASS\n";
}
