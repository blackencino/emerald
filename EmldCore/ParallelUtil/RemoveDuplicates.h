//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#ifndef _EmldCore_ParallelUtil_RemoveDuplicates_h_
#define _EmldCore_ParallelUtil_RemoveDuplicates_h_

#include "ContiguousBlocks.h"
#include "Foundation.h"
#include "Sort.h"

#include <EmldCore/Util/VectorUtil.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
//-*****************************************************************************
// ENHANCED SIMPLE BLOCK
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T, typename INDEX>
struct RDSimpleBlock : public SimpleBlock<INDEX> {
    typedef T value_type;
    typedef INDEX index_type;

    RDSimpleBlock()
      : SimpleBlock<INDEX>() {
    }

    RDSimpleBlock(const T& i_val, index_type i_begin, index_type i_end)
      : SimpleBlock<INDEX>(i_begin, i_end)
      , value(i_val) {
    }

    T value;
};

//-*****************************************************************************
//-*****************************************************************************
// FUNCTORS FOR COPYING TO & FROM ENHANCED BLOCKS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T, typename BLOCK, typename INDEX>
struct CopyValuesToBlocks
  : public ZeroForEachFunctorI<CopyValuesToBlocks<T, BLOCK, INDEX>, INDEX> {
    const T* InValues;
    BLOCK* OutBlocks;

    void operator()(INDEX i) const {
        BLOCK& b = OutBlocks[i];
        b.value = InValues[b.Begin];
    }
};

//-*****************************************************************************
template <typename T, typename BLOCK, typename INDEX>
struct CopyValuesFromBlocks
  : public ZeroForEachFunctorI<CopyValuesFromBlocks<T, BLOCK, INDEX>, INDEX> {
    const BLOCK* InBlocks;
    T* OutValues;

    void operator()(INDEX i) const {
        OutValues[i] = InBlocks[i].value;
    }
};

//-*****************************************************************************
template <typename T, typename BLOCK, typename INDEX, typename SIZE>
struct CopyValuesAndSizesFromBlocks
  : public ZeroForEachFunctorI<
      CopyValuesAndSizesFromBlocks<T, BLOCK, INDEX, SIZE>,
      INDEX> {
    const BLOCK* InBlocks;
    T* OutValues;
    SIZE* OutSizes;

    void operator()(INDEX i) const {
        const BLOCK& b = InBlocks[i];
        OutValues[i] = b.value;
        OutSizes[i] = static_cast<SIZE>(b.size());
    }
};

//-*****************************************************************************
//-*****************************************************************************
// REMOVE DUPLICATES AND GET SIZES IN PLACE.
//-*****************************************************************************
//-*****************************************************************************

template <typename VECTOR, typename SVECTOR>
void VectorRemoveDuplicatesGetSizesInPlaceEqual(VECTOR& io_items,
                                                SVECTOR& o_sizes) {
    // Typedefs
    typedef std::ptrdiff_t INDEX;
    typedef typename VECTOR::value_type T;
    typedef T value_type;
    typedef RDSimpleBlock<T, INDEX> block_type;
    typedef std::vector<block_type> RDSimpleBlockVector;
    typedef typename SVECTOR::value_type size_type;
    typedef std::vector<INDEX> IndexVector;

    // Sort the stuff
    VectorSort<VECTOR>(io_items);

    // Make block index vector.
    IndexVector blockIndices;
    blockIndices.resize(io_items.size());
    INDEX numItems = 1 + VectorContiguousBlockIndicesEqual<VECTOR, IndexVector>(
                           io_items, blockIndices);

    // Make block vector.
    RDSimpleBlockVector blocks;
    blocks.resize(std::size_t(numItems));
    VectorContiguousBlockSetBeginEndSimple<IndexVector, RDSimpleBlockVector>(
      blockIndices, blocks);

    // Copy elements to blocks
    {
        CopyValuesToBlocks<T, block_type, INDEX> F;
        F.InValues = vector_cdata(io_items);
        F.OutBlocks = vector_data(blocks);
        F.execute(numItems);
    }

    // Resize unduplicated items
    io_items.resize(numItems);
    o_sizes.resize(numItems);

    // Copy blocks to elements & sizes
    {
        CopyValuesAndSizesFromBlocks<T, block_type, INDEX, size_type> F;
        F.InBlocks = vector_cdata(blocks);
        F.OutValues = vector_data(io_items);
        F.OutSizes = vector_data(o_sizes);
        F.execute(numItems);
    }
}

//-*****************************************************************************
//-*****************************************************************************
// REMOVE DUPLICATES IN PLACE.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VECTOR>
void VectorRemoveDuplicatesInPlaceEqual(VECTOR& io_items) {
    // Typedefs
    typedef std::ptrdiff_t INDEX;
    typedef typename VECTOR::value_type T;
    typedef T value_type;
    typedef RDSimpleBlock<T, INDEX> block_type;
    typedef std::vector<block_type> RDSimpleBlockVector;
    typedef std::vector<INDEX> IndexVector;

    // Sort the stuff
    VectorSort<VECTOR>(io_items);
    // std::cout << "Sorted items" << std::endl;

    // Make block index vector.
    IndexVector blockIndices;
    blockIndices.resize(io_items.size());
    INDEX numItems = 1 + VectorContiguousBlockIndicesEqual<VECTOR, IndexVector>(
                           io_items, blockIndices);
    // std::cout << "Made contiguous block indices." << std::endl
    //          << "Num contiguous blocks: " << numItems << std::endl;

    // Make block vector.
    RDSimpleBlockVector blocks;
    blocks.resize(std::size_t(numItems));
    VectorContiguousBlockSetBeginEndSimple<IndexVector, RDSimpleBlockVector>(
      blockIndices, blocks);
    // std::cout << "Made blocks." << std::endl;

    // Copy elements to blocks
    {
        CopyValuesToBlocks<T, block_type, INDEX> F;
        F.InValues = vector_cdata(io_items);
        F.OutBlocks = vector_data(blocks);
        F.execute(numItems);
    }
    // std::cout << "Copied values to blocks." << std::endl;

    // Resize unduplicated items
    io_items.resize(numItems);
    // std::cout << "Resized items." << std::endl;

    // Copy blocks to elements & sizes
    {
        CopyValuesFromBlocks<T, block_type, INDEX> F;
        F.InBlocks = vector_cdata(blocks);
        F.OutValues = vector_data(io_items);
        F.execute(numItems);
    }
    // std::cout << "Copied blocks to values." << std::endl;
}

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
