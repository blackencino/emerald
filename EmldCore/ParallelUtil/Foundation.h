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

#ifndef _EmldCore_ParallelUtil_Foundation_h_
#define _EmldCore_ParallelUtil_Foundation_h_

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

#include <boost/format.hpp>
#include <boost/progress.hpp>
#include <boost/utility/value_init.hpp>
#include <boost/utility.hpp>

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
#include <tbb/blocked_range2d.h>
#include <tbb/blocked_range3d.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_queue.h>
#include <tbb/queuing_rw_mutex.h>
#include <tbb/atomic.h>
#include <tbb/parallel_sort.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/mutex.h>
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
namespace ParallelUtil {

using namespace EmldCore::Util;


//-*****************************************************************************
typedef tbb::atomic<int8_t>  atomic_int8_t;
typedef tbb::atomic<int16_t> atomic_int16_t;
typedef tbb::atomic<int32_t> atomic_int32_t;
typedef tbb::atomic<int64_t> atomic_int64_t;

} // End namespace ParallelUtil

namespace Util {

template <>
struct storage_traits<ParallelUtil::atomic_int8_t>
{
    typedef ParallelUtil::atomic_int8_t       value_type;
    typedef storage_bytes_1     storage_type;
    typedef int8_t              pod_type;
};

template <>
struct storage_traits<ParallelUtil::atomic_int16_t>
{
    typedef ParallelUtil::atomic_int16_t      value_type;
    typedef storage_bytes_2     storage_type;
    typedef int16_t             pod_type;
};
template <>
struct storage_traits<ParallelUtil::atomic_int32_t>
{
    typedef ParallelUtil::atomic_int32_t      value_type;
    typedef storage_bytes_4     storage_type;
    typedef int32_t             pod_type;
};
template <>
struct storage_traits<ParallelUtil::atomic_int64_t>
{
    typedef ParallelUtil::atomic_int64_t      value_type;
    typedef storage_bytes_8     storage_type;
    typedef int64_t             pod_type;
};

} // End namespace Util

namespace ParallelUtil {


} // End namespace ParallelUtil
} // End namespace EmldCore


#endif