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

#ifndef _EmldCore_Noise_CellNoise_h_
#define _EmldCore_Noise_CellNoise_h_

#include "Foundation.h"

namespace EmldCore {
namespace Noise {

//-*****************************************************************************
float CellNoise( unsigned long int iSeeds[], int numSeeds );

//-*****************************************************************************
template <class T>
float CellNoise( const Imath::Vec3<T> &iPoint )
{
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + ( long int )std::floor( iPoint[0] );
    seeds[2] = 0L + ( long int )std::floor( iPoint[1] );
    seeds[3] = 0L + ( long int )std::floor( iPoint[2] );
    return CellNoise( seeds, 4 );
}

//-*****************************************************************************
template <class T>
Imath::V2f CellNoise2( const Imath::Vec3<T> &iPoint )
{
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + ( long int )std::floor( iPoint[0] );
    seeds[2] = 0L + ( long int )std::floor( iPoint[1] );
    seeds[3] = 0L + ( long int )std::floor( iPoint[2] );
    Imath::V2f ret;
    ret.x = CellNoise( seeds, 4 );
    seeds[0] = 1;
    ret.y = CellNoise( seeds, 4 );
    return ret;
}

//-*****************************************************************************
template <class T>
Imath::V3f CellNoise3( const Imath::Vec3<T> &iPoint )
{
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + ( long int )std::floor( iPoint[0] );
    seeds[2] = 0L + ( long int )std::floor( iPoint[1] );
    seeds[3] = 0L + ( long int )std::floor( iPoint[2] );
    Imath::V3f ret;
    ret.x = CellNoise( seeds, 4 );
    seeds[0] = 1;
    ret.y = CellNoise( seeds, 4 );
    seeds[0] = 2;
    ret.z = CellNoise( seeds, 4 );
    return ret;
}

} // End namespace Noise
} // End namespace EmldCore

#endif
