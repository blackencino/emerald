#pragma once

#include <tbb/concurrent_unordered_map.h>
#include <algorithm>
#include <cstdint>

namespace emerald::sph2d_box {

struct Block {
    uint32_t begin_index = std::numeric_limits<uint32_t>::max();
    uint32_t end_index = std::numeric_limits<uint32_t>::min();
};

Block extend_by(Block const block, uint32_t const index) {
    return {std::min(block.begin_index, index),
            std::max(block.end_index, index + 1)};
}
//-*****************************************************************************
// A concurrent hash table that maps Zindices to blocks.
using Block_map = tbb::concurrent_unordered_map<uint64_t, Block>;

}  // namespace emerald::sph2d_box
