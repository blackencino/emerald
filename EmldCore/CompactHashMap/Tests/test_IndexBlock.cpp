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

#include "EmldCore/CompactHashMap/IndexBlock.h"
#include <iostream>

using namespace EmldCore::CompactHashMap;
using namespace EmldCore::ParallelUtil;
using namespace EmldCore::Util;

int main( int argc, char *argv[] )
{
    typedef Imath::Vec3<int16_t> ctype;
    typedef std::pair<int64_t, int32_t> vtype;
    typedef IndexBlock<ctype,vtype,128> BlockI_128;

    BlockI_128 b;
    b.init( ctype( 1, 2, 3 ), vtype( 17, 19 ) );

    std::cout << "Size of BlockI_128: " << sizeof( b ) << std::endl
              << "num values: " << b.max_block_size << std::endl
              << "pad bytes: " << b.pad_bytes << std::endl
              << "size of TempDataBlock: " << sizeof( TempIndexBlock<ctype> )
              << std::endl
              << "size of next block: " << sizeof( b.m_nextBlock ) << std::endl
              << "size of values: " << sizeof( b.m_indices ) << std::endl
              << "size of block size: " << sizeof( b.m_sizeThisBlock )
              << std::endl 
              << "size of padding: " << sizeof( b.__pad ) << std::endl
              << "b.size(): " << b.size() << std::endl
              << "b.empty(): " << b.empty() << std::endl
              << "b.contains( bad ): " 
              << b.contains( vtype( 19, 31 ) ) << std::endl
              << "b.contains( good ): " 
              << b.contains( vtype( 17, 19 ) ) << std::endl;

    return 0;
}