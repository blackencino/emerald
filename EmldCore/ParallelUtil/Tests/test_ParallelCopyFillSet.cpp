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

#include <EmldCore/Util/All.h>
#include <EmldCore/ParallelUtil/CopyFillSet.h>
#include <ImathVec.h>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    std::vector<float> values( 4096 );
    UniformRand urand( -100.0, 100.0 );
    for ( std::size_t i = 0; i < values.size(); ++i )
    {
        values[i] = urand();
    }
    std::cout << "Created base set of values." << std::endl;

    std::vector<float> newValues( 4096 );
    VectorCopy( values, newValues );
    std::cout << "Parallel-copied values." << std::endl;

    for ( std::size_t i = 0; i < values.size(); ++i )
    {
        EMLD_ASSERT( values[i] == newValues[i], 
                       "copy mismatch at: " << i );
    }
    std::cout << "Verified result." << std::endl;


    static const float testVal = 7.34f;
    VectorFill( newValues, testVal );
    std::cout << "Parallel-set values." << std::endl;
    for ( std::size_t i = 0; i < values.size(); ++i )
    {
        EMLD_ASSERT( newValues[i] == testVal, 
                       "bad set value at: " << i );
    }
    std::cout << "Verified result." << std::endl;

    VectorZeroBits( newValues );
    std::cout << "Parallel-zero'd values." << std::endl;
    for ( std::size_t i = 0; i < values.size(); ++i )
    {
        EMLD_ASSERT( newValues[i] == 0.0f,
                       "bad zero value at: " << i );
    }
    std::cout << "Verified result." << std::endl;

    std::vector<int> orderedIndices( 7153 );
    VectorSetOrderedIndices( orderedIndices );
    std::cout << "Parallel set ordered indices." << std::endl;
    for ( std::size_t i = 0; i < orderedIndices.size(); ++i )
    {
        EMLD_ASSERT( orderedIndices[i] == i,
                        "bad ordered index at: " << i );
    }
    std::cout << "Verified result." << std::endl;

    return 0;
}
