//-*****************************************************************************
// Copyright (c) 2001-2012 Christopher Jon Horvath. All rights reserved.
//-*****************************************************************************

#ifndef _EmldCore_MeshEdit_Foundation_h_
#define _EmldCore_MeshEdit_Foundation_h_

#include <EmldCore/Util/All.h>

#include <ImathMath.h>
#include <ImathVec.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <list>
#include <set>
#include <map>
#include <utility>

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

namespace EmldCore {
namespace MeshEdit {

using Imath::V3f;
using Imath::V3d;

template <class T>
inline Imath::Vec3<T> normalized( const Imath::Vec3<T> &iVec )
{
    return iVec.normalized();
}

template <class T>
inline Imath::Vec3<T> cross( const Imath::Vec3<T> &iA,
                             const Imath::Vec3<T> &iB )
{
    return iA.cross( iB );
}

template <class T>
inline T dot( const Imath::Vec3<T> &iA,
              const Imath::Vec3<T> &iB )
{
    return iA.dot( iB );
}

template <class T>
inline T angleBetween( const Imath::Vec3<T> &iA,
                       const Imath::Vec3<T> &iB )
{
    return std::acos( dot( normalized( iA ),
                                      normalized( iB ) ) );
}

template <class T>
inline T length( const Imath::Vec3<T> &iA )
{
    return iA.length();
}

template <class T>
inline T length2( const Imath::Vec3<T> &iA )
{
    return iA.length2();
}


} // End namespace MeshEdit
} // End namespace EmldCore

#endif
