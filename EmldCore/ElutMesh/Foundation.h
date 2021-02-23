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

#ifndef _EmldCore_ElutMesh_Foundation_h_
#define _EmldCore_ElutMesh_Foundation_h_

//-*****************************************************************************
// This is an implementation of the paper:
// P. Cignoni, F. Ganovelli, C. Montani, and R. Scopigno:
// Reconstruction of Topologically Correct and Adaptive Trilinear Isosurfaces,
// Computer & Graphics, Elsevier Science, 1999
// Which provides an exhaustive LUT to disambiguate isosurface triangulations
// inside a rectangular cell with iso values on the 8 corners of the cell,
// such that meshes will link together without having to explicitly march
// through them, as in Marching Cubes.
//-*****************************************************************************

#include <EmldCore/CompactHashMap/SpatialHash.h>
#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/Util/All.h>

#include <ImathFrustum.h>
#include <ImathEuler.h>
#include <ImathInterval.h>

namespace EmldCore {
namespace ElutMesh {

using namespace EmldCore::Util;

namespace eu = EmldCore::Util;
namespace epu = EmldCore::ParallelUtil;

//-*****************************************************************************
typedef std::vector<V3f> V3fArray;
typedef std::vector<V3d> V3dArray;
typedef std::vector<V3i> V3iArray;
typedef std::vector<Box3i> Box3iArray;

//-*****************************************************************************
// Macros for level set stuff.
//#define QNEGATIVE( Q ) ((Q)<0)
//#define QPOSITIVE( Q ) ((Q)>=0)
template <typename T>
inline bool QNEGATIVE( const T& Q )
{
#ifdef DEBUG
    EMLD_ASSERT( std::isfinite( Q ), "bad q: " << Q );
#endif
    EMLD_LITERAL_CONSTANT T zero = T(0);
    return Q < zero;
}

//-*****************************************************************************
template <typename T>
inline bool QPOSITIVE( const T& Q )
{
#ifdef DEBUG
    EMLD_ASSERT( std::isfinite( Q ), "bad q: " << Q );
#endif
    EMLD_LITERAL_CONSTANT T zero = T(0);
    return Q >= zero;
}


//#define BETWEEN( LOW, VAL, HIGH ) (((VAL)>=(LOW))&&((VAL)<=(HIGH)))

//-*****************************************************************************
// Point stuff

//-*****************************************************************************
// CORNER, EDGE, AND FACE NAMES
//-*****************************************************************************

//-*****************************************************************************
// Weird ordering is to match the paper, which is sorta y-up left handed?
enum FaceName
{
    kNegx = 0,             
    kPosx = 1,             
    kNegy = 4,           
    kPosy = 5,              
    kNegz = 2,
    kPosz = 3
};

//-*********************************************************************
// These are ordered so that the first six represent this sample's
// triangulation edges, and the other six represent edges that
// neighboring samples will handle.
enum EdgeName
{
    kNegxNegy,
    kPosxPosy,
    kNegxNegz,
    kPosxPosz,
    kNegyNegz,
    kPosyPosz,
    
    kPosxNegy,
    kNegxPosy,
    kPosxNegz,
    kNegxPosz,
    kPosyNegz,
    kNegyPosz
};

//-*****************************************************************************
// The corner name. Weird ordering to match paper.
enum CornerName
{
    kNegxNegyNegz = 0,      
    kPosxNegyNegz = 1,      
    kNegxPosyNegz = 4,          
    kPosxPosyNegz = 5,
    kNegxNegyPosz = 3,
    kPosxNegyPosz = 2,
    kNegxPosyPosz = 7,
    kPosxPosyPosz = 6
};


} // End namespace ElutMesh
} // End namespace EmldCore

#endif
