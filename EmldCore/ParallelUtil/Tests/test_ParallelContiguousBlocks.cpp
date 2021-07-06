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

#include <EmldCore/ParallelUtil/ContiguousBlocks.h>
#include <EmldCore/Util/Exception.h>

#include <boost/format.hpp>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
typedef std::vector<std::string> Strings;

//-*****************************************************************************
// Make a bunch of "buckets", which have a random number of copies of
// a string.
struct Bucket {
    Bucket()
      : text("")
      , copies(0) {
    }

    Bucket(const std::string& i_str, int i_copies = 1)
      : text(i_str)
      , copies(i_copies) {
    }

    void emit(Strings& o_strings) const {
        for (int i = 0; i < copies; ++i) { o_strings.push_back(text); }
    }

    std::string text;
    int copies;
};

typedef std::vector<Bucket> Buckets;

//-*****************************************************************************
struct strLenEqual {
    bool operator()(const std::string& i_a, const std::string& i_b) const {
        return i_a.length() == i_b.length();
    }
};

//-*****************************************************************************
struct intPairSetBeginEnd {
    typedef std::pair<int, int> block_type;
    typedef int index_type;

    void set_begin(block_type& o_block, index_type i_begin) const {
        o_block.first = i_begin;
    }

    void set_end(block_type& o_block, index_type i_end) const {
        o_block.second = i_end;
    }
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    Buckets buckets;
    buckets.push_back(Bucket("Cream", 7));
    buckets.push_back(Bucket("Sugar", 8));
    buckets.push_back(Bucket("Sweetener", 3));
    buckets.push_back(Bucket("Decaf", 11));
    buckets.push_back(Bucket("Mocha", 14));
    buckets.push_back(Bucket("Latte", 6));
    buckets.push_back(Bucket("Hot Chocolate", 31));
    buckets.push_back(Bucket("Other", 4));
    buckets.push_back(Bucket("Premium Roast", 1));
    std::cout << "Made " << buckets.size() << " buckets." << std::endl;

    Strings strings;
    for (Buckets::const_iterator iter = buckets.begin(); iter != buckets.end();
         ++iter) {
        (*iter).emit(strings);
    }
    std::size_t N = strings.size();
    std::cout << "Emitted " << N << " strings." << std::endl;

    // Okay, try it out.
    std::vector<int> bucketIndices(N);
    int foundLastBucketIndex =
      VectorContiguousBlockIndicesEqual(strings, bucketIndices);
    std::cout << "Found Last Bucket Index = " << foundLastBucketIndex
              << std::endl;

    EMLD_ASSERT((foundLastBucketIndex + 1) == buckets.size(),
                "Num buckets should be the same.");

    for (int i = 0; i < N; ++i) {
        std::cout << (boost::format("BI[%02d]: %s") % bucketIndices[i] %
                      strings[i])
                  << std::endl;
    }

    // Create some contiguous blocks.
    typedef SimpleBlock<int> SBI;
    typedef std::vector<SBI> SBIs;
    SBIs blocks;
    VectorContiguousBlockSetBeginEndSimple(bucketIndices, blocks);
    auto numBlocks = static_cast<int>(blocks.size());
    std::cout << "Set block begin & end indices." << std::endl;
    for (int i = 0; i < numBlocks; ++i) {
        std::cout << (boost::format("Bucket %d goes from [%d to %d]") % i %
                      blocks[i].Begin % blocks[i].End)
                  << std::endl;
    }

    // Create some contiguous blocks with std::pair.
    typedef std::pair<int, int> PBI;
    typedef std::vector<PBI> PBIs;
    PBIs pairBlocks;
    intPairSetBeginEnd ipsbe;
    VectorContiguousBlockSetBeginEnd(bucketIndices, pairBlocks, ipsbe);
    numBlocks = static_cast<int>(pairBlocks.size());
    std::cout << "Set pair-block begin & end indices." << std::endl;
    for (int i = 0; i < numBlocks; ++i) {
        std::cout << (boost::format("Bucket %d goes from [%d to %d]") % i %
                      pairBlocks[i].first % pairBlocks[i].second)
                  << std::endl;
    }

    // Try it again, based on length.
    strLenEqual comp;
    int foundLastEqLenBucketIndex =
      VectorContiguousBlockIndices(strings, bucketIndices, comp);
    std::cout << "Found Last EqLen Bucket Index = " << foundLastEqLenBucketIndex
              << std::endl;

    EMLD_ASSERT((foundLastEqLenBucketIndex) == 5,
                "Should be 5 equal-length buckets.");

    for (int i = 0; i < N; ++i) {
        std::cout << (boost::format("BI[%02d]: %s") % bucketIndices[i] %
                      strings[i])
                  << std::endl;
    }

    return 0;
}
