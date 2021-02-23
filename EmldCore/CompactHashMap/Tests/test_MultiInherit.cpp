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

#include "EmldCore/ParallelUtil/All.h"
#include "EmldCore/Util/All.h"
#include <algorithm>
#include <utility>

//-*****************************************************************************
template <typename T>
struct SubDataStorage
{
    typedef T value_type;
    typedef T* pointer;
    T used[16];
    T unused[16];

    SubDataStorage()
    {
        static EmldCore::Util::zero_bits<T> zero;
        std::fill( used, used + 16, zero.bits );
        std::fill( unused, unused + 16, zero.bits );
    }
};

//-*****************************************************************************
template <typename DERIVED>
struct DataStorage
{
    template <typename T>
    T* getUsed()
    {
        DERIVED* derived = static_cast<DERIVED*>( this );
        SubDataStorage<T>* super = static_cast<SubDataStorage<T>*>( derived );
        return &( super->used[0] );
    }

    template <typename T>
    T* getUnused()
    {
        DERIVED* derived = static_cast<DERIVED*>( this );
        SubDataStorage<T>* super = static_cast<SubDataStorage<T>*>( derived );
        return &( super->unused[0] );
    }
};

//-*****************************************************************************
class MyDataStorage
    : public DataStorage<MyDataStorage>
    , public SubDataStorage<int>
    , public SubDataStorage<float>
    , public SubDataStorage<double>
{
    // Nothing
};

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    MyDataStorage data;

    int* iused = data.getUsed<int>();
    float* fused = data.getUsed<float>();

    iused[3] = 7;
    fused[3] = -93.4f;

    data.getUnused<double>()[11] = 17.3;

    EMLD_ASSERT( iused[3] == 7, "Should work." );

    return 0;
};
