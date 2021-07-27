#ifndef THIRD_PARTY_MEM_PROVIDER_H_
#define THIRD_PARTY_MEM_PROVIDER_H_

// This file defines the MemoryProvider interface class and several common
// implementations.

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace third_party {

// MemoryProvider represents a particular reservoir of memory on a machine. The
// most typical memory provider is the system main memory backed by DRAM.
class MemoryProvider {
 public:
  MemoryProvider(const MemoryProvider&) = delete;
  MemoryProvider& operator=(const MemoryProvider&) = delete;

  MemoryProvider() = default;
  virtual ~MemoryProvider() = default;

  // Returns total capacity in bytes.
  virtual size_t BytesTotal() = 0;
  // Returns total free bytes.
  virtual size_t BytesFree() = 0;

  // Map `length` bytes from this provider to virtual `addr`. Returns true on
  // success.
  virtual bool MapAt(void* addr, size_t length) = 0;
};

enum class MapOption {
  kDefault,         // Use process or system default.
  kNumaInterleave,  // Interleave allocations across backing NUMA nodes.
};

class Numa {
 public:
  class Node {
   public:
    explicit Node(int index) : index_(index) {}

    int index() { return index_; }
    size_t bytes_total();
    size_t bytes_free();
    bool is_far();
    std::set<int> cpus();

   private:
    int index_;
  };

  Numa(const Numa&) = delete;
  Numa& operator=(const Numa&) = delete;

  Numa();
  ~Numa() = default;

  std::vector<Node> GetAllNodes();
};

// Memory exposed via NUMA nodes.
class NumaMemory : public MemoryProvider {
 public:
  NumaMemory(const NumaMemory&) = delete;
  NumaMemory& operator=(const NumaMemory&) = delete;

  explicit NumaMemory(std::set<int>& nodes,
                      MapOption map_option = MapOption::kDefault)
      : map_option_(map_option), node_set_(nodes) {}

  size_t BytesTotal() final;
  size_t BytesFree() final;
  bool MapAt(void* addr, size_t length) final;

 private:
  MapOption map_option_;
  std::set<int> node_set_;
  Numa numa_;
};

class MainMemory : public MemoryProvider {
 public:
  MainMemory(const MainMemory&) = delete;
  MainMemory& operator=(const MainMemory&) = delete;

  explicit MainMemory(MapOption map_option = MapOption::kDefault);

  size_t BytesTotal() final { return numa_memory_->BytesTotal(); }
  size_t BytesFree() final { return numa_memory_->BytesFree(); }
  bool MapAt(void* addr, size_t length) override {
    return numa_memory_->MapAt(addr, length);
  }

 private:
  std::unique_ptr<NumaMemory> numa_memory_;
};

// Memory exposed via device files.
class DeviceMemory : public MemoryProvider {
 public:
  DeviceMemory(const DeviceMemory&) = delete;
  DeviceMemory& operator=(const DeviceMemory&) = delete;

  // - pathname: pathname of the device.
  // - offset: offset into the device file at which all MapAt(...) requests are
  // fullfilled.
  explicit DeviceMemory(const std::string& pathname, size_t offset = 0)
      : pathname_(pathname), offset_(offset) {}
  virtual ~DeviceMemory() = default;

  size_t BytesTotal() final;
  size_t BytesFree() final;

  bool MapAt(void* addr, size_t length) override;

 private:
  std::string pathname_;
  size_t offset_;
};
}  // namespace third_party

#endif  // THIRD_PARTY_MEM_PROVIDER_H_
