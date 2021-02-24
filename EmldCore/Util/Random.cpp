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

#include "Random.h"

namespace EmldCore {
namespace Util {

//-*****************************************************************************
// Tiny Encryption Algorithm for turning hashes into good random numbers.
// Works in-place.
static void Tea8( uint32_t v[2] )
{
    static const uint32_t k[4] = { 0xa341316c,
                                   0xc8013ea4,
                                   0xad90777d,
                                   0x7395761e };

    static const uint32_t delta = 0x9e3779b9;

    uint32_t sum = 0;

    for ( int i = 0; i < 8; ++i )
    {
        sum += delta;
        v[0] += (( v[1] << 4 ) + k[0])^(v[1] + sum)^((v[1] >> 5) + k[1] );
        v[1] += (( v[0] << 4 ) + k[2])^(v[0] + sum)^((v[0] >> 5) + k[3] );
    }
}

//-*****************************************************************************
void HashGrid( uint32_t seed,
               uint32_t x, uint32_t y, uint32_t z,
               unsigned short out[3] )
{
    union
    {
        uint32_t v[2];
        unsigned short x[4];
    } state;
    
    state.v[0] = seed;
    state.v[1] = x;
    Tea8( state.v );

    state.v[0] += y;
    state.v[1] += z;
    Tea8( state.v );

    out[0] = state.x[0];
    out[1] = state.x[1];
    out[2] = state.x[2];
}

} // End namespace Util
} // End namespace EmldCore
