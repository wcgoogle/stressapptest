#include "mem_provider.h"

#include <assert.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <numa.h>

namespace {

void die(const char* msg) {
  fprintf(stderr, "Error: %s\n", msg);
  abort();
}

bool MapFileAt(void* addr, size_t length, int fd, size_t offset) {
  void* buffer = mmap(addr, length, PROT_READ | PROT_WRITE,
                      MAP_SHARED | MAP_FIXED, fd, offset);
  if (buffer == MAP_FAILED) {
    perror(__func__);
    return false;
  }

  assert(buffer == addr);
  return true;
}

}  // namespace

namespace third_party {

Numa::Numa() {
  // numa_init();
  if (numa_available() != 0) {
    die("System is not NUMA-aware");
  }
}

std::vector<Numa::Node> Numa::GetAllNodes() {
  struct bitmask* mems_allowed = numa_get_mems_allowed();
  if (mems_allowed == nullptr) {
    die("out of memory");
  }

  // Could change, so query on each call.
  std::vector<Node> all_nodes;
  for (int node = 0; node <= numa_max_node(); node++) {
    if (numa_bitmask_isbitset(mems_allowed, node)) {
      all_nodes.push_back(Node(node));
    }
  }

  numa_bitmask_free(mems_allowed);
  return all_nodes;
}

size_t Numa::Node::bytes_total() {
  long long size{numa_node_size64(index_, nullptr)};
  if (size < 0) {
    size = 0;
  }
  return static_cast<size_t>(size);
}

size_t Numa::Node::bytes_free() {
  long long free_size = 0;
  numa_node_size64(index_, &free_size);
  if (free_size < 0) {
    free_size = 0;
  }
  return static_cast<size_t>(free_size);
}

namespace {

std::set<int> NodeToCpus(int node) {
  auto cpumask = numa_allocate_cpumask();
  if (cpumask == nullptr) {
    die("out of memory");
  }

  std::set<int> cpuset;

  int ret = numa_node_to_cpus(node, cpumask);
  if (ret != 0) {
    fprintf(stderr, "numa_node_to_cpus() failed.\n");
    return cpuset;
  }

  for (int i = 0; i < cpumask->size; i++) {
    if (numa_bitmask_isbitset(cpumask, i)) {
      cpuset.insert(i);
    }
  }

  numa_bitmask_free(cpumask);
  return cpuset;
}

int GetNeighborNode(int node) {
  struct bitmask* mems = numa_get_mems_allowed();
  if (mems == nullptr) {
    die("out of memory");
  }

  int neighbor = -1;
  int min_distance = std::numeric_limits<int>::max();
  for (int i = 0; i <= numa_max_node() && i != node; i++) {
    int distance = numa_distance(node, i);

    if (distance == 0) {
      fprintf(stderr, "numa_distance(%d, %d) failed.\n", node, i);
      neighbor = -1;
      break;
    }

    if (distance < min_distance) {
      neighbor = i;
      min_distance = distance;
    }
  }

  numa_bitmask_free(mems);
  return neighbor;
}

}  // namespace

bool Numa::Node::is_far() {
  auto cpus = NodeToCpus(index_);
  return (cpus.empty());
}

std::set<int> Numa::Node::cpus() {
  std::set<int> cpus = NodeToCpus(index_);
  if (cpus.empty()) {
    // A far NUMA node does not have any explicit associated CPUs but it is
    // connected to the same socket as their closest neighbor node.
    int neighbor = GetNeighborNode(index_);
    if (neighbor >= 0) {
      cpus = NodeToCpus(neighbor);
    }
  }
  return cpus;
}

size_t NumaMemory::BytesTotal() {
  size_t total = 0;
  for (auto& node : numa_.GetAllNodes()) {
    if (node_set_.find(node.index()) != node_set_.end()) {
      total += node.bytes_total();
    }
  }
  return total;
}

size_t NumaMemory::BytesFree() {
  size_t total = 0;
  for (auto node : numa_.GetAllNodes()) {
    if (node_set_.find(node.index()) != node_set_.end()) {
      total += node.bytes_free();
    }
  }
  return total;
}

bool NumaMemory::MapAt(void* addr, size_t length) {
  void* buffer = mmap(addr, length, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
  if (buffer == MAP_FAILED) {
    perror(__func__);
    return false;
  }
  assert(addr == buffer);

  switch (map_option_) {
    case MapOption::kDefault:
      break;

    case MapOption::kNumaInterleave: {
      struct bitmask* bmp = numa_allocate_nodemask();
      if (bmp == nullptr) {
        die("out of memory");
      }
      for (auto node : numa_.GetAllNodes()) {
        if (node_set_.find(node.index()) != node_set_.end()) {
          numa_bitmask_setbit(bmp, node.index());
        }
      }
      numa_interleave_memory(addr, length, bmp);
      numa_bitmask_free(bmp);
    } break;

    default:
      die("Mapping option not supported");
      break;
  }

  return true;
}

MainMemory::MainMemory(MapOption map_option) {
  std::set<int> nodes;
  for (auto& node : Numa().GetAllNodes()) {
    if (!node.is_far()) {
      nodes.insert(node.index());
    }
  }
  numa_memory_.reset(new NumaMemory(nodes, map_option));
}

size_t DeviceMemory::BytesTotal() {
  static bool has_queried = false;
  static size_t total_bytes = 0;

  if (has_queried > 0) {
    return total_bytes;
  }

  int fd = open(pathname_.c_str(), O_RDONLY, 0);
  if (fd < 0) {
    perror("open");
    return 0;
  }

  if (0 != ioctl(fd, BLKGETSIZE64, &total_bytes)) {
    // OK, probably this is a file on disk rather than a special file. Stat it.
    struct stat buf = {};
    if (0 != fstat(fd, &buf)) {
      perror("fstat");
    } else {
      total_bytes = buf.st_size;
    }
  }

  close(fd);
  return total_bytes;
}

// It is complicated and unecessary to track unmapped bytes of a device. Assume
// all are available.
size_t DeviceMemory::BytesFree() { return BytesTotal(); }

bool DeviceMemory::MapAt(void* addr, size_t length) {
  int fd = open(pathname_.c_str(), O_RDWR, 0);
  if (fd < 0) {
    perror(__func__);
    return false;
  }

  bool result = MapFileAt(addr, length, fd, offset_);
  close(fd);
  return result;
}

}  // namespace third_party
