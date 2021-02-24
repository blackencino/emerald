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

#ifndef _EmldCore_Util_ImathVecOrdering_h_
#define _EmldCore_Util_ImathVecOrdering_h_

#include "Foundation.h"

#include <OpenEXR/ImathVec.h>

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
#else
namespace Imath {
#endif

//-*****************************************************************************
//-*****************************************************************************
// VEC2 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
inline bool operator<( const Vec2<T>& A, const Vec2<T>& B )
{
    if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x < B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator<=( const Vec2<T>& A, const Vec2<T>& B )
{
    if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x <= B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>( const Vec2<T>& A, const Vec2<T>& B )
{
    if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x > B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>=( const Vec2<T>& A, const Vec2<T>& B )
{
    if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x >= B.x ); }
}

//-*****************************************************************************
//-*****************************************************************************
// VEC3 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
bool operator<( const Vec3<T>& A, const Vec3<T>& B )
{
    if ( A.z < B.z ) { return true; }
    else if ( A.z > B.z ) { return false; }
    else if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x < B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator<=( const Vec3<T>& A, const Vec3<T>& B )
{
    if ( A.z < B.z ) { return true; }
    else if ( A.z > B.z ) { return false; }
    else if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x <= B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>( const Vec3<T>& A, const Vec3<T>& B )
{
    if ( A.z > B.z ) { return true; }
    else if ( A.z < B.z ) { return false; }
    else if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x > B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>=( const Vec3<T>& A, const Vec3<T>& B )
{
    if ( A.z > B.z ) { return true; }
    else if ( A.z < B.z ) { return false; }
    else if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x >= B.x ); }
}

//-*****************************************************************************
//-*****************************************************************************
// VEC4 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
inline bool operator<( const Vec4<T>& A, const Vec4<T>& B )
{
    if ( A[3] < B[3] ) { return true; }
    else if ( A[3] > B[3] ) { return false; }
    else if ( A.z < B.z ) { return true; }
    else if ( A.z > B.z ) { return false; }
    else if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x < B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator<=( const Vec4<T>& A, const Vec4<T>& B )
{
    if ( A[3] < B[3] ) { return true; }
    else if ( A[3] > B[3] ) { return false; }
    else if ( A.z < B.z ) { return true; }
    else if ( A.z > B.z ) { return false; }
    else if ( A.y < B.y ) { return true; }
    else if ( A.y > B.y ) { return false; }
    else { return ( A.x <= B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>( const Vec4<T>& A, const Vec4<T>& B )
{
    if ( A[3] > B[3] ) { return true; }
    else if ( A[3] < B[3] ) { return false; }
    else if ( A.z > B.z ) { return true; }
    else if ( A.z < B.z ) { return false; }
    else if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x > B.x ); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>=( const Vec4<T>& A, const Vec4<T>& B )
{
    if ( A[3] > B[3] ) { return true; }
    else if ( A[3] < B[3] ) { return false; }
    else if ( A.z > B.z ) { return true; }
    else if ( A.z < B.z ) { return false; }
    else if ( A.y > B.y ) { return true; }
    else if ( A.y < B.y ) { return false; }
    else { return ( A.x >= B.x ); }
}

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
#else
} // End namespace Imath
#endif

#endif
