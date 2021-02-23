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

#ifndef _EmldCore_CompactHashMap_Foundation_h_
#define _EmldCore_CompactHashMap_Foundation_h_

#if __cplusplus > 199711L
#define CHM_USE_CXX11 1
#else
#undef CHM_USE_CXX11
#endif

#include <EmldCore/Noise/All.h>
#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/Util/All.h>

#include <ImathMath.h>
#include <ImathVec.h>
#include <ImathMatrix.h>
#include <ImathBox.h>
#include <ImathQuat.h>
#include <ImathColor.h>
#include <ImathFun.h>
#include <ImathRandom.h>
#include <ImathBoxAlgo.h>

#include <boost/pool/pool.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/format.hpp>
#include <boost/progress.hpp>

#if CHM_USE_CXX11

#include <unordered_map>
#include <regex>
#include <chrono>
#include <ratio>
#include <random>
#include <memory>
#include <type_traits>

#define CHM_SHARED_PTR std::shared_ptr
#define CHM_UNIQUE_PTR std::unique_ptr
#define CHM_STATIC_CONSTEXPR static constexpr

#else

#include <boost/smart_ptr.hpp>
#include <boost/regex.hpp>
#include <boost/random.hpp>
#include <boost/chrono.hpp>
#include <boost/unordered_map.hpp>

#define CHM_SHARED_PTR boost::shared_ptr
#define CHM_UNIQUE_PTR boost::scoped_ptr
#define CHM_STATIC_CONSTEXPR static const

#endif

#include <iostream>
#include <vector>
#include <map>
#include <deque>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <utility>
#include <stdlib.h>

//-*****************************************************************************
// Thread Building Blocks
//#include <tbb/tbb.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_scan.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_sort.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <tbb/spin_rw_mutex.h>
#include <tbb/atomic.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/task.h>
//-*****************************************************************************

#include <sys/types.h>
#include <stdio.h>
#include <limits>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>

//-*****************************************************************************
namespace EmldCore {
namespace CompactHashMap {

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
// Adopt exceptions
#define CHM_THROW( TEXT ) EMLD_THROW( TEXT )
#define CHM_ASSERT( COND, TEXT ) EMLD_ASSERT( COND, TEXT )
#define CHM_DEBUG_ASSERT( COND, TEXT ) EMLD_DEBUG_ASSERT( COND, TEXT )

} // End namespace CompactHashMap
} // End namespace EmldCore


#endif