#ifndef THIRD_PARTY_MMAPPER_H_
#define THIRD_PARTY_MMAPPER_H_

#include <sys/mman.h>
#include <memory>
#include <vector>

#include "mem_provider.h"

namespace third_party {

class MMappedBlock {
 public:
  MMappedBlock(const MMappedBlock&) = delete;
  MMappedBlock& operator=(const MMappedBlock&) = delete;

  explicit MMappedBlock(void* data, size_t length)
      : data_(data), length_(length) {}
  ~MMappedBlock() { munmap(data_, length_); }

  void* data() { return data_; }
  const void* data() const { return data_; }
  size_t length() const { return length_; }

 private:
  void* data_;
  size_t length_;
};

// TL;DR: MMapper creates a contiguous virtual memory block with different parts
// of its address range mapped to different memory "providers".
//
// Usage:
//   using memtester::DeviceMemory;
//   using memtester::MainMemory;
//   auto block =
//       memtester::MMapper()
//           .AddRequest(absl::make_unique<MainMemory>(), 64 * 1024))
//           .AddRequest(absl::make_unique<DeviceMemory>("/dev/foo"), 64 * 1024)
//           .Build();
//   ASSERT_TRUE(block);
//   EXPECT_NE(block->data(), nullptr);
//   EXPECT_EQ(block->length(), 128 * 1024);
//
class MMapper {
 public:
  MMapper(const MMapper&) = delete;
  MMapper& operator=(const MMapper&) = delete;

  MMapper();

  // Add a mapping request.
  // - provider: the backing memory provider.
  // - length: number of bytes to map from `provider`.
  MMapper& AddRequest(std::unique_ptr<MemoryProvider> provider, size_t length);

  // Set the alignment used for aligning all mapping addresses.
  // Since no gaps are allowed between sub-mappings, the lengths of all but the
  // last mapping request has to be aligned on `alignment` too. `alignment` must
  // be multiples of system pagesize (typically 4K). The default alignment is
  // system pagesize.
  MMapper& SetAlignment(size_t alignment);

  // Build an mmapped memory block by combining all mapping requests in request
  // order. Returns std::unique_ptr<nullptr> on failure.
  std::unique_ptr<MMappedBlock> Build();

 private:
  std::vector<std::pair<std::unique_ptr<MemoryProvider>, size_t>> requests_;
  size_t alignment_;
};

}  // namespace third_party
#endif  // THIRD_PARTY_MMAPPER_H_
