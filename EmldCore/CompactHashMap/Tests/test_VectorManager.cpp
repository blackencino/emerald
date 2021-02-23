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

#include "EmldCore/CompactHashMap/VectorManager.h"
#include "EmldCore/ParallelUtil/All.h"
#include "EmldCore/Util/All.h"
#include <algorithm>
#include <utility>


//-*****************************************************************************
using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;
using namespace EmldCore::CompactHashMap;

//-*****************************************************************************
class VectorManager
    : public VectorManagerWrapper<VectorManager>
    , public SubVectorManager<storage_bytes_1>
    , public SubVectorManager<storage_bytes_2>
    , public SubVectorManager<storage_bytes_4>
    , public SubVectorManager<storage_bytes_8>
    , public SubVectorManager<storage_bytes_12>
    , public SubVectorManager<storage_bytes_16>
    , public SubVectorManager<storage_bytes_24>
{
public:
    VectorManager()
        : VectorManagerWrapper<VectorManager> ()
        , SubVectorManager<storage_bytes_1>()
        , SubVectorManager<storage_bytes_2>()
        , SubVectorManager<storage_bytes_4>()
        , SubVectorManager<storage_bytes_8>()
        , SubVectorManager<storage_bytes_12>()
        , SubVectorManager<storage_bytes_16>()
        , SubVectorManager<storage_bytes_24>()
    {}

    explicit VectorManager( std::size_t i_bucketSize )
        : VectorManagerWrapper<VectorManager> ()
        , SubVectorManager<storage_bytes_1>( i_bucketSize )
        , SubVectorManager<storage_bytes_2>( i_bucketSize )
        , SubVectorManager<storage_bytes_4>( i_bucketSize )
        , SubVectorManager<storage_bytes_8>( i_bucketSize )
        , SubVectorManager<storage_bytes_12>( i_bucketSize )
        , SubVectorManager<storage_bytes_16>( i_bucketSize )
        , SubVectorManager<storage_bytes_24>( i_bucketSize )
    {}

    void setBucketSize( std::size_t i_bucketSize )
    {
        SubVectorManager<storage_bytes_1>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_2>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_4>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_8>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_12>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_16>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_24>::setBucketSize( i_bucketSize );
    }
};

//-*****************************************************************************
typedef VectorManager::TypedManagedCvectorHandle<atomic_int8_t> AtomicChar_MVH;
typedef VectorManager::TypedManagedCvectorHandle<int8_t> Char_MVH;
typedef VectorManager::TypedManagedCvectorHandle<int32_t> Int_MVH;
typedef VectorManager::TypedManagedCvectorHandle<uint32_t> Uint_MVH;
typedef VectorManager::TypedManagedCvectorHandle<float16_t> Half_MVH;
typedef VectorManager::TypedManagedCvectorHandle<float32_t> Float_MVH;
typedef VectorManager::TypedManagedCvectorHandle<V3i> V3i_MVH;
typedef VectorManager::TypedManagedCvectorHandle<V3f> V3f_MVH;
typedef VectorManager::TypedManagedCvectorHandle<V3d> V3d_MVH;


//-*****************************************************************************
int main( int argc, char* argv[] )
{
    VectorManager vm( 65536 );

    std::size_t N = 9096;

    AtomicChar_MVH atomicChar_vec = vm.get<atomic_int8_t>( N );
    Char_MVH char_vec = vm.get<int8_t>( N );
    Int_MVH int_vec = vm.get<int32_t>( N );
    Uint_MVH uint_vec = vm.get<uint32_t>( N );

    atomicChar_vec.fill_with_zero();

    return 0;
};
