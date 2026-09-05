#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace ue_bridge
{
// Shared across lanes. Counts queued payload bytes (not DDS copies or in-flight frames).
class QueueBudget
{
public:
  explicit QueueBudget(std::size_t limit) : limit_(limit) {}
  bool reserve(std::size_t bytes)
  {
    auto used = used_.load();
    do {
      if (bytes > limit_ - used) { return false; }
    } while (!used_.compare_exchange_weak(used, used + bytes));
    return true;
  }
  void release(std::size_t bytes) { used_.fetch_sub(bytes); }
  std::size_t used() const { return used_.load(); }
private:
  const std::size_t limit_;
  std::atomic<std::size_t> used_{0};
};

enum class PushResult { accepted, replaced, full };

// Caller holds the lane mutex. T has key and payload_size members and is movable.
// Round-robin scheduling preserves per-key FIFO order without starving other keys.
template<typename T>
class PendingQueue
{
public:
  PendingQueue(QueueBudget & budget, std::size_t max_keys, std::size_t fifo_depth)
  : budget_(budget), max_keys_(max_keys), fifo_depth_(fifo_depth) {}
  PendingQueue(const PendingQueue &) = delete;
  PendingQueue & operator=(const PendingQueue &) = delete;
  ~PendingQueue()
  {
    for (const auto & entry : topics_) {
      for (const auto & sample : entry.second) { budget_.release(sample.payload_size); }
    }
  }

  PushResult push(T sample, bool fifo)
  {
    const std::string key = sample.key;
    auto it = topics_.find(key);
    if (it != topics_.end() && !fifo) {
      auto & old = it->second.back();
      const auto old_bytes = old.payload_size;
      const auto new_bytes = sample.payload_size;
      if (new_bytes > old_bytes && !budget_.reserve(new_bytes - old_bytes)) {
        return PushResult::full;
      }
      old = std::move(sample);
      if (old_bytes > new_bytes) { budget_.release(old_bytes - new_bytes); }
      return PushResult::replaced;
    }
    if ((it == topics_.end() && topics_.size() >= max_keys_) ||
      (it != topics_.end() && it->second.size() >= fifo_depth_))
    {
      return PushResult::full;
    }
    const auto bytes = sample.payload_size;
    if (!budget_.reserve(bytes)) { return PushResult::full; }
    try {
      if (it == topics_.end()) {
        it = topics_.try_emplace(key).first;
        try {
          ready_.push_back(key);
        } catch (...) {
          topics_.erase(it);
          throw;
        }
        try {
          it->second.push_back(std::move(sample));
        } catch (...) {
          ready_.pop_back();
          topics_.erase(it);
          throw;
        }
      } else {
        it->second.push_back(std::move(sample));
      }
    } catch (...) {
      budget_.release(bytes);
      throw;
    }
    ++size_;
    return PushResult::accepted;
  }

  T pop()
  {
    if (empty()) { throw std::logic_error("pop from empty queue"); }
    const auto key = ready_.front();
    auto it = topics_.find(key);
    // Allocate before modifying the queue, preserving it if allocation fails.
    if (it->second.size() > 1) { ready_.push_back(key); }
    T sample = std::move(it->second.front());
    it->second.pop_front();
    ready_.pop_front();
    if (it->second.empty()) { topics_.erase(it); }
    --size_;
    budget_.release(sample.payload_size);
    return sample;
  }
  bool empty() const { return size_ == 0; }
  std::size_t size() const { return size_; }
  std::size_t depth(const std::string & key) const
  {
    const auto it = topics_.find(key);
    return it == topics_.end() ? 0 : it->second.size();
  }
  const T * front(const std::string & key) const
  {
    const auto it = topics_.find(key);
    return it == topics_.end() ? nullptr : &it->second.front();
  }
private:
  QueueBudget & budget_;
  const std::size_t max_keys_;
  const std::size_t fifo_depth_;
  std::deque<std::string> ready_;
  std::unordered_map<std::string, std::deque<T>> topics_;
  std::size_t size_{0};
};
}  // namespace ue_bridge
