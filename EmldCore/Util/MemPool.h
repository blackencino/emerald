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

#ifndef _EmldCore_Util_MemPool_h_
#define _EmldCore_Util_MemPool_h_

#include "Foundation.h"

#include <boost/pool/singleton_pool.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace EmldCore {
namespace Util {

//-*****************************************************************************
struct MemPoolTag {};

//-*****************************************************************************
#define EMLD_UTIL_MEM_POOL_MALLOC(TYPE)                                 \
(boost::singleton_pool< ::EmldCore::Util::MemPoolTag,                \
     sizeof(TYPE)>::malloc())

#define EMLD_UTIL_MEM_POOL_MALLOC_N(TYPE,N)                             \
(boost::singleton_pool< ::EmldCore::Util::MemPoolTag,                \
     sizeof(TYPE)>::ordered_malloc(N))

#define EMLD_UTIL_MEM_POOL_FREE(TYPE, P)                                \
(boost::singleton_pool< ::EmldCore::Util::MemPoolTag,                \
     sizeof(TYPE)>::free(P))

#define EMLD_UTIL_MEM_POOL_FREE_N(TYPE,P,N)                             \
(boost::singleton_pool< ::EmldCore::Util::MemPoolTag,                \
     sizeof(TYPE)>::ordered_free((P),(N)))

    

//-*****************************************************************************
// Uses unordered malloc/free from boost singleton pool
#define EMLD_UTIL_MEM_POOL_NEW_AND_DELETE(TYPE)                         \
                                                                        \
static void *operator new( std::size_t s )                              \
{                                                                       \
    if ( s > sizeof(TYPE) )                                             \
    {                                                                   \
        return boost::singleton_pool< ::EmldCore::Util::MemPoolTag,  \
            sizeof(TYPE)>::ordered_malloc( s/sizeof(TYPE) );            \
    }                                                                   \
    else                                                                \
    {                                                                   \
        return boost::singleton_pool< ::EmldCore::Util::MemPoolTag,  \
            sizeof(TYPE)>::malloc();                                    \
    }                                                                   \
}                                                                       \
                                                                        \
static void operator delete( void *p, std::size_t s )                   \
{                                                                       \
    if ( s > sizeof(TYPE) )                                             \
    {                                                                   \
        boost::singleton_pool< ::EmldCore::Util::MemPoolTag,         \
            sizeof(TYPE)>::ordered_free( p, s/sizeof(TYPE) );           \
    }                                                                   \
    else                                                                \
    {                                                                   \
        boost::singleton_pool< ::EmldCore::Util::MemPoolTag,         \
            sizeof(TYPE)>::free( p );                                   \
    }                                                                   \
}

//-*****************************************************************************
// For STL Vectors
#define EMLD_UTIL_MEM_POOL_ALLOC(TYPE)          \
    boost::pool_allocator<TYPE>

// For STL Lists
#define EMLD_UTIL_MEM_POOL_FAST_ALLOC(TYPE)     \
    boost::fast_pool_allocator<TYPE>


} // End namespace Util
} // End namespace EmldCore

#endif
