#include "mmapper.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace {

void die(const char* msg) {
  fprintf(stderr, "Error: %s\n", msg);
  abort();
}

size_t GetSystemPageSize() {
  size_t size = sysconf(_SC_PAGESIZE);
  if (size <= 0) {
    die("sysconf(_SC_PAGESIZE) failed.");
  }
  return size;
}

bool IsPowerOf2(size_t n) { return (n > 0 && (n & (n - 1)) == 0); }

size_t RoundUp(size_t size, size_t alignment) {
  if (!IsPowerOf2(alignment)) {
    die("Alignment is not power of two");
  }
  return ((size + alignment - 1) & ~(alignment - 1));
}

char* ReserveMapping(size_t length) {
  void* buffer = mmap(nullptr, length, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (buffer == MAP_FAILED) {
    perror("mmap");
    return nullptr;
  }

  return static_cast<char*>(buffer);
}

}  // namespace

namespace third_party {

MMapper::MMapper() { alignment_ = GetSystemPageSize(); }

MMapper& MMapper::SetAlignment(size_t alignment) {
  alignment_ = alignment;
  return *this;
}

MMapper& MMapper::AddRequest(std::unique_ptr<MemoryProvider> provider,
                             size_t length) {
  requests_.push_back({std::move(provider), length});
  return *this;
}

std::unique_ptr<MMappedBlock> MMapper::Build() {
  size_t pagesize = GetSystemPageSize();

  if (alignment_ < pagesize || (alignment_ % pagesize) > 0) {
    fprintf(stderr, "Bad alignment: %zu\n", alignment_);
    return nullptr;
  }

  size_t combined_length = 0;
  for (const auto& it : requests_) {
    combined_length += it.second;
  }

  if (combined_length == 0) {
    return nullptr;
  }

  size_t map_length = combined_length + alignment_ - pagesize;
  char* buffer = ReserveMapping(map_length);
  if (buffer == nullptr) {
    fprintf(stderr, "Failed to reserve mapping.\n");
    return nullptr;
  }

  char* aligned_buffer = reinterpret_cast<char*>(
      RoundUp(reinterpret_cast<uintptr_t>(buffer), alignment_));
  if (aligned_buffer > buffer) {
    munmap(buffer, aligned_buffer - buffer);
    map_length -= (aligned_buffer - buffer);
    if (map_length > combined_length) {
      munmap(aligned_buffer + combined_length, map_length - combined_length);
    }
  }

  std::unique_ptr<MMappedBlock> block(
      new MMappedBlock(aligned_buffer, combined_length));
  for (int i = 0; i < requests_.size(); i++) {
    MemoryProvider& provider = *requests_[i].first;
    size_t req_length = requests_[i].second;
    if (req_length == 0) {
      // Ignore zero lengths.
      continue;
    }

    // All but the last requested length must be aligned on alignment_ to avoid
    // gaps in between remaps.
    if (i < (requests_.size() - 1) && req_length % alignment_) {
      fprintf(stderr, "Length (%zu) does not align on (%zu).\n", req_length,
              alignment_);
      return nullptr;
    }

    if (!provider.MapAt(aligned_buffer, req_length)) {
      return nullptr;
    }

    aligned_buffer += req_length;
  }

  return block;
}

}  // namespace third_party
