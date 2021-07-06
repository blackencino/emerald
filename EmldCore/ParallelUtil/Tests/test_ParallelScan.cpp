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

#include <EmldCore/ParallelUtil/Scan.h>
#include <EmldCore/Util/Exception.h>
#include <EmldCore/Util/Random.h>

#include <boost/format.hpp>

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
int main(int argc, char* argv[]) {
    int N = 4096;
    std::vector<int> values(N);
    UniformRand urand(0.0, 100.1);
    for (int i = 0; i < values.size(); ++i) {
        values[i] = (int)floor(urand());
    }
    std::cout << "Created base set of values." << std::endl;

    std::vector<int> sumValues(values.size());
    std::cout << "Created sum vector." << std::endl;

    int sum = VectorPrefixSum(values, sumValues);
    std::cout << "Prefix sum = " << sum << std::endl;

    for (int i = 0; i < values.size(); ++i) {
        std::cout << (boost::format("Value[%04d] = %03d; Sum[%04d] = %08d;") %
                      i % values[i] % i % sumValues[i])
                  << std::endl;
    }

    N = 100;
    IndexValuesFunctor<int> ivf;
    sumValues.resize(N);
    sum = FunctorVectorPrefixSum(ivf, sumValues);
    std::cout << "Sum N where N = 100: " << sum << std::endl;
    for (int i = 0; i < N; ++i) {
        std::cout << (boost::format("Value[%04d] = %03d; Sum[%04d] = %08d;") %
                      i % ivf[i] % i % sumValues[i])
                  << std::endl;
    }
    EMLD_ASSERT(sum == (N * (N - 1)) / 2, "Gauss says differently.");

    return 0;
}
