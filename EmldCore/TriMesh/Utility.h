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

#ifndef _EmldCore_TriMesh_Utility_h_
#define _EmldCore_TriMesh_Utility_h_

#include "Foundation.h"

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// Handy convenience function used in multiple places
// 
// A function which takes three values and rotates them so that the
// smallest is in the zero spot, but does not change their winding order.
template <typename T>
void rotateMinToZero( T v[] )
{
    T tmp;
    if ( v[0] < v[1] )
    {
        if ( v[0] < v[2] )
        {
            // Nothing
        }
        else
        {
            // V2 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[2];
            v[2] = v[1];
            v[1] = tmp;
        }
    }
    else
    {
        if ( v[1] < v[2] )
        {
            // V1 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[1];
            v[1] = v[2];
            v[2] = tmp;
        }
        else
        {
            // V2 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[2];
            v[2] = v[1];
            v[1] = tmp;
        }
    }
}

} // End namespace TriMesh
} // End namespace EmldCore

#endif
