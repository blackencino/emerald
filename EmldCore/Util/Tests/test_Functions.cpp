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

#include <EmldCore/Util/All.h>

#include <iostream>
#include <typeinfo>
#include <cmath>
#include <cstdlib>

namespace EmldCore {
namespace Util {

//-*****************************************************************************
template <typename T>
void testWrapT( T x, T lb, T ub, T expected, T tol )
{
    T k = wrap<T>( x, lb, ub );
    std::cout << "wrap<" << typeid( T ).name() << ">( " << x
            << ", " << lb << ", " << ub << " ) = " << k << ", expecting: "
            << expected << std::endl;
    EMLD_ASSERT( std::abs( k - expected ) <= tol, "test wrap" );
}

//-*****************************************************************************
void testWrap()
{
    testWrapT<int>( 15, 7, 12, 9, 0 );
    testWrapT<int>( 6, -4, 3, -2, 0 );
    testWrapT<int>( -19, 9, 13, 11, 0 );

    // Try floats.
    testWrapT<float>( -741.325f, 1.4151f, 19.7333f, 9.72113f, 0.0001f );

    float e = 32.4f;
    int block = -844;
    float lb = 1.8f;
    float ub = 66.7113f;

    float x = lb + ( ( ub - lb ) * float( block ) ) + ( e - lb );
    testWrapT<float>( x, lb, ub, e, 0.01f );
}

} // End namespace Util
} // End namespace EmldCore

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    EmldCore::Util::testWrap();
    return 0;
}
