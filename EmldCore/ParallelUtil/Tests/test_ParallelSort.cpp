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

#include <EmldCore/ParallelUtil/Sort.h>
#include <EmldCore/Util/Exception.h>
#include <EmldCore/Util/Random.h>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
int main(int argc, char* argv[]) {
    std::vector<float> values(4096);
    UniformRand urand(-100.0, 100.0);
    for (std::size_t i = 0; i < values.size(); ++i) { values[i] = urand(); }
    std::cout << "Created base set of values." << std::endl;

    std::vector<float> newValues(4096);
    std::copy(values.begin(), values.end(), newValues.begin());
    std::cout << "Copied values." << std::endl;

    VectorSort(values);
    std::cout << "Parallel-sorted values." << std::endl;

    std::sort(newValues.begin(), newValues.end());
    std::cout << "Serial-sorted newValues." << std::endl;

    for (std::size_t i = 0; i < values.size(); ++i) {
        EMLD_ASSERT(values[i] == newValues[i], "sort out of order at: " << i);
    }
    std::cout << "Verified result." << std::endl;

    return 0;
}
