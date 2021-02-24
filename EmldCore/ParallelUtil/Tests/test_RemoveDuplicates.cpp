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

#include <EmldCore/ParallelUtil/RemoveDuplicates.h>
#include <EmldCore/Util/Exception.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
typedef std::vector<std::string> Strings;
typedef std::vector<unsigned char> Counts;

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
    VectorRemoveDuplicatesInPlaceEqual(strings);
    std::cout << "Number of unique strings: " << strings.size() << std::endl;

    EMLD_ASSERT(strings.size() == buckets.size(),
                "Num buckets should be the same.");

    EMLD_ASSERT(strings[0] == "Cream" && strings[1] == "Decaf" &&
                  strings[2] == "Hot Chocolate" && strings[3] == "Latte" &&
                  strings[4] == "Mocha" && strings[5] == "Other" &&
                  strings[6] == "Premium Roast" && strings[7] == "Sugar" &&
                  strings[8] == "Sweetener",
                "Mismatched strings & counts");

    // Try it again.
    strings.clear();
    for (Buckets::const_iterator iter = buckets.begin(); iter != buckets.end();
         ++iter) {
        (*iter).emit(strings);
    }
    std::cout << "Re-Emitted " << N << " strings." << std::endl;

    // Make some counts.
    Counts counts;
    VectorRemoveDuplicatesGetSizesInPlaceEqual(strings, counts);

    std::cout << "Number of unique strings: " << strings.size() << std::endl;

    EMLD_ASSERT(strings.size() == buckets.size(),
                "Num buckets should be the same.");

    std::cout << "Sizes: ";
    for (int i = 0; i < counts.size(); ++i) {
        std::cout << strings[i] << "[" << (int)(counts[i]) << "]";
        if (i != (counts.size() - 1)) { std::cout << ", "; }
    }
    std::cout << std::endl;

    EMLD_ASSERT(
      strings[0] == "Cream" && counts[0] == 7 && strings[1] == "Decaf" &&
        counts[1] == 11 && strings[2] == "Hot Chocolate" && counts[2] == 31 &&
        strings[3] == "Latte" && counts[3] == 6 && strings[4] == "Mocha" &&
        counts[4] == 14 && strings[5] == "Other" && counts[5] == 4 &&
        strings[6] == "Premium Roast" && counts[6] == 1 &&
        strings[7] == "Sugar" && counts[7] == 8 && strings[8] == "Sweetener" &&
        counts[8] == 3,
      "Mismatched strings & counts");

    std::cout << "All good." << std::endl;

    return 0;
}
