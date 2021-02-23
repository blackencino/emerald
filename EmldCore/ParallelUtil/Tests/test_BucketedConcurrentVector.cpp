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

#include <EmldCore/ParallelUtil/BucketedConcurrentVector.h>

#include <EmldCore/Util/All.h>
#include <EmldCore/ParallelUtil/Sort.h>
#include <EmldCore/ParallelUtil/For.h>

#include <ImathVec.h>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
struct EmitIndices
    : public ZeroForEachFunctorI<EmitIndices,int>
{
    BucketedConcurrentVector<int>* IndicesPtr;

    void operator()( int i ) const
    {
        IndicesPtr->push_back( i );
    }
};

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    BucketedConcurrentVector<int> values;
    values.set_bucket_size( 64 );
    int N = 4096;
    {
        EmitIndices F;
        F.IndicesPtr = &values;
        F.execute( N );
    }
    std::cout << "Parallel emission of integers" << std::endl;

    EMLD_ASSERT( values.size() == N, 
                    "Wrong number of parallel-emitted indices." );

    VectorSort( values );
    std::cout << "Parallel-sorted values." << std::endl;

    for ( std::size_t i = 0; i < values.size(); ++i )
    {
        EMLD_ASSERT( values[i] == i, 
                        "sort out of order at: " << i );
    }
    std::cout << "Verified result." << std::endl;

    return 0;
}
