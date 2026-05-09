// include/environments/fnv_hash.h

#pragma once

#include <cstdint>

// FNV-1a hash for fixed-size array state types.
// Used as the hasher for phmap::flat_hash_map state lookup tables.
struct FnvHash {
  template <typename Container>
  uint32_t operator()(const Container& s) const {
    uint32_t hash = 2166136261u;
    for (auto val : s) {
      hash ^= static_cast<uint32_t>(val);
      hash *= 16777619u;
    }
    return hash;
  }
};
