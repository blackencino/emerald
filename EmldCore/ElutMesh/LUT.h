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

#ifndef _EmldCore_ElutMesh_LUT_h_
#define _EmldCore_ElutMesh_LUT_h_

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

#include "Foundation.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
template <class T>
struct SaddlePointsT
{
    T data[7];
};

typedef SaddlePointsT<int> SaddlePointsI;

//-*****************************************************************************
// This is the variations lookup table from the paper.
extern int g_ExhaustiveLUT[];
extern std::size_t g_NumInLUT;

//-*****************************************************************************
// Forwards
class LUTIterator;

//-*****************************************************************************
class Variation
{
public:
    Variation() {}
    
    template <typename T>
    bool match( const SaddlePointsT<T> &iSf ) const
    {
        for ( int i = 0; i < 7; ++i )
        {
            if ( m_saddle.data[i] < 0 )
            {
                if ( !QNEGATIVE( iSf.data[i] ) )
                {
                    return false;
                }
            }
            else if ( m_saddle.data[i] > 0 )
            {
                if ( QNEGATIVE( iSf.data[i] ) )
                {
                    return false;
                }
            }
        }
        return true;
    }
    
    std::size_t size() const { return m_triangles.size(); }
    const V3i &triangle( std::size_t i ) const { return m_triangles[i]; }
    const V3i &operator[]( std::size_t i ) const { return m_triangles[i]; }

protected:
    friend class Variations;

    // Updates LUT iterator in place
    void readFromInts( LUTIterator &i_liter );

protected:
    SaddlePointsI m_saddle;
    std::vector<V3i> m_triangles;
};

//-*****************************************************************************
class Variations
{
public:
    Variations() : m_index( 0 ) {}

    std::size_t index() const { return m_index; }

    std::size_t size() const { return m_variations.size(); }
    const Variation &variation( std::size_t i ) const 
    { return m_variations[i]; }
    const Variation &operator[]( std::size_t i ) const 
    { return m_variations[i]; }

protected:
    friend class LUT;

    void readFromInts( std::size_t i_idx, LUTIterator &i_liter );

protected:
    std::size_t m_index;
    std::vector<Variation> m_variations;
};

//-*****************************************************************************
class LUT
{
public:
    LUT();

    std::size_t size() const { return m_variations.size(); }

    const Variations &variations( std::size_t i ) const
    {
        if ( m_variations.size() != 256 )
        {
            EMLD_THROW( "ElutCell: Uninitialized ELUT" );
        }
        return m_variations[i];
    }

    const Variations& operator[]( std::size_t i ) const
    {
        return variations( i );
    }

protected:
    std::vector<Variations> m_variations;
};

//-*****************************************************************************
// Global instance of LUT.
extern LUT g_LUT;
    
} // End namespace ElutMesh
} // End namespace EmldCore

#endif
