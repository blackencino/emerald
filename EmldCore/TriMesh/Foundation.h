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

#ifndef _EmldCore_TriMesh_Foundation_h_
#define _EmldCore_TriMesh_Foundation_h_

#include <EmldCore/SpatialSubd/All.h>
#include <EmldCore/Util/All.h>

#include <ImathMath.h>
#include <ImathMatrix.h>
#include <ImathMatrixAlgo.h>
#include <ImathVec.h>
#include <ImathBox.h>
#include <ImathFun.h>
#include <ImathBoxAlgo.h>
#include <ImathInterval.h>
#include <ImathLine.h>
#include <ImathLineAlgo.h>

#include <boost/static_assert.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/cstdint.hpp>
#include <boost/random.hpp>
#include <boost/range/iterator_range.hpp>

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
namespace TriMesh {

//-*****************************************************************************
#define TRIMESH_THROW( TEXT ) EMLD_THROW( TEXT )
#define TRIMESH_ASSERT( COND , TEXT ) EMLD_ASSERT( COND , TEXT )

//-*****************************************************************************
// Bring in Imath types!
using Imath::V3f;
using Imath::Box3f;
using Imath::M44f;
using Imath::M44d;
using Imath::V3d;
using Imath::Box3d;
using Imath::V2i;
using Imath::V2f;
using Imath::M33f;
using Imath::Box2f;
using Imath::Box2d;

//-*****************************************************************************
// Make some standard vectors
typedef std::vector<V3f> V3fVector;
typedef std::vector<int> IntVector;
typedef std::vector<unsigned int> UintVector;
typedef std::vector<float> FloatVector;

//-*****************************************************************************
// We use ranges as a way of passing arrays of data around, without explicitly
// declaring them as vectors or any other storage.
typedef boost::iterator_range<const V3f *> ConstV3fRange;
typedef boost::iterator_range<const int *> ConstIntRange;
typedef boost::iterator_range<const unsigned int *> ConstUintRange;

//-*****************************************************************************
template <class T>
inline boost::iterator_range<const T *>
CreateRange( const std::vector<T> &iVec )
{
    if ( iVec.empty() ) { return boost::iterator_range<const T*>(
                              ( const T * )NULL, ( const T * )NULL ); }
    else { return boost::iterator_range<const T*>(
               &iVec.front(), iVec.size() + &iVec.front() ); }
}

//-*****************************************************************************
inline ConstV3fRange CreateV3fRange( const V3fVector &iVec )
{ return CreateRange<V3f>( iVec ); }

//-*****************************************************************************
inline ConstIntRange CreateIntRange( const IntVector &iVec )
{ return CreateRange<int>( iVec ); }

//-*****************************************************************************
inline ConstUintRange CreateUintRange( const UintVector &iVec )
{ return CreateRange<unsigned int>( iVec ); }

//-*****************************************************************************
template <class T, class RANGE>
inline const T *GetBeginPointer( const RANGE & iRng )
{
    if ( iRng.empty() ) { return ( const T * )NULL; }
    else { return &(iRng.front()); }
}

//-*****************************************************************************
template <class RANGE>
inline const V3f* GetBeginV3fPointer( const RANGE &iRng )
{
    return GetBeginPointer<V3f,RANGE>( iRng );
}

//-*****************************************************************************
template <class RANGE>
inline const int* GetBeginIntPointer( const RANGE &iRng )
{
    return GetBeginPointer<int,RANGE>( iRng );
}

//-*****************************************************************************
template <class RANGE>
inline const unsigned int* GetBeginUintPointer( const RANGE &iRng )
{
    return GetBeginPointer<unsigned int,RANGE>( iRng );
}

} // End namespace TriMesh
} // End namespace EmldCore

#endif
