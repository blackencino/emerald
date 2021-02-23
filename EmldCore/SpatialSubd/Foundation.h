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

#ifndef _EmldCore_SpatialSubd_Foundation_h_
#define _EmldCore_SpatialSubd_Foundation_h_

#include <ImathMath.h>
#include <ImathVec.h>
#include <ImathBox.h>
#include <ImathFun.h>
#include <ImathBoxAlgo.h>
#include <ImathInterval.h>

#include <boost/static_assert.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/cstdint.hpp>
#include <boost/random.hpp>
#include <boost/format.hpp>

#include <half.h>

#include <map>
#include <utility>
#include <algorithm>
#include <string>
#include <vector>

#include <iostream>
#include <sstream>
#include <exception>
#include <limits>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

namespace EmldCore {
namespace SpatialSubd {

using Imath::V3i;
typedef Imath::Vec3<unsigned int> V3ui;
using Imath::V3f;
using Imath::V3d;

using Imath::Box3i;
using Imath::Box3f;
using Imath::Box3d;

} // End namespace SpatialSubd
} // End namespace EmldCore


#endif