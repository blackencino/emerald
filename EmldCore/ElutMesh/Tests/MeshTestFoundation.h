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

#ifndef _EmldCore_ElutMesh_MeshTestFoundation_h_
#define _EmldCore_ElutMesh_MeshTestFoundation_h_

#include <EmldCore/ElutMesh/All.h>
#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/Util/All.h>
#include <EmldCore/GeepGLFW/All.h>
#include <EmldCore/SimpleSimViewer/All.h>

#include <ImathInterval.h>

#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/algorithm/string/find.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <utility>
#include <vector>
#include <string>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

namespace MeshTest {

//-*****************************************************************************
//using namespace EmldCore::Util;
//using namespace EmldCore::GeepGLFW;
namespace eu = EmldCore::Util;
namespace epu = EmldCore::ParallelUtil;
namespace elm = EmldCore::ElutMesh;
namespace essv = EmldCore::SimpleSimViewer;
namespace egl = EmldCore::GeepGLFW;

using namespace EmldCore::Util;

using Imath::Intervalf;

typedef Imath::Vec3<uint32_t> V3ui;

typedef std::vector<V3f> V3fArray;
typedef std::vector<V3i> V3iArray;
typedef std::vector<Intervalf> IvlfArray;

inline std::ostream& operator<<( std::ostream& ostr, const Intervalf& i )
{
    ostr << "[" << i.min << ":" << i.max << "]";
    return ostr;
}
    
} // End namespace MeshTest

#endif
