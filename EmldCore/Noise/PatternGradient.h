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

#ifndef _EmldCore_Noise_PatternGradient_h_
#define _EmldCore_Noise_PatternGradient_h_

#include "Foundation.h"
#include "SimplexNoise.h"

namespace EmldCore {
namespace Noise {

//-*****************************************************************************
template <class PATTERN>
class PatternGradient2D
{
public:
    typedef PATTERN                             pattern_type;
    typedef typename PATTERN::value_type        value_type;
    typedef typename PATTERN::value_type        T_t;
    typedef Imath::Vec2<T_t>                    output_type;
    typedef PatternGradient2D<PATTERN>          this_type;

    PatternGradient2D( PATTERN pattern )
      : m_pattern( pattern ) {}

    // 2D Gradient
    output_type operator()( T_t x, T_t y ) const
    {
        static const T_t tiny = (( T_t )0.01);
        
        T_t v0 = m_pattern( x, y );
        T_t vx = m_pattern( x + tiny, y );
        T_t vy = m_pattern( x, y + tiny );
        return Imath::Vec2<T_t>( ( vx - v0 ) / tiny,
                                 ( vy - v0 ) / tiny );
    }

    output_type operator()( const Imath::Vec2<T_t> &p ) const
    {
        return operator()( p.x, p.y );
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef PatternGradient2D<SimplexNoisef> Vnoise2f;
typedef PatternGradient2D<SimplexNoised> Vnoise2d;

//-*****************************************************************************
template <class PATTERN>
class PatternGradient3D
{
public:
    typedef PATTERN                             pattern_type;
    typedef typename PATTERN::value_type        value_type;
    typedef typename PATTERN::value_type        T_t;
    typedef Imath::Vec3<T_t>                    output_type;
    typedef PatternGradient3D<PATTERN>          this_type;

    PatternGradient3D( PATTERN pattern )
      : m_pattern( pattern ) {}

    // 3D Gradient
    output_type operator()( T_t x, T_t y, T_t z ) const
    {
        static const T_t tiny = (( T_t )0.01);
        
        T_t v0 = m_pattern( x, y, z );
        T_t vx = m_pattern( x + tiny, y, z );
        T_t vy = m_pattern( x, y + tiny, z );
        T_t vz = m_pattern( x, y, z + tiny );
        return Imath::Vec3<T_t>( ( vx - v0 ) / tiny,
                                 ( vy - v0 ) / tiny,
                                 ( vz - v0 ) / tiny );
    }
    
    output_type operator()( const Imath::Vec3<T_t> &p ) const
    {
        return operator()( p.x, p.y, p.z );
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef PatternGradient3D<SimplexNoisef> Vnoise3f;
typedef PatternGradient3D<SimplexNoised> Vnoise3d;

//-*****************************************************************************
template <class PATTERN>
class EvolvingPatternGradient3D
{
public:
    typedef PATTERN                             pattern_type;
    typedef typename PATTERN::value_type        value_type;
    typedef typename PATTERN::value_type        T_t;
    typedef Imath::Vec3<T_t>                    output_type;
    typedef PatternGradient3D<PATTERN>          this_type;

    EvolvingPatternGradient3D( PATTERN pattern )
      : m_pattern( pattern ) {}

    // 3D Gradient
    output_type operator()( T_t x, T_t y, T_t z, T_t t ) const
    {
        static const T_t tiny = (( T_t )0.01);
        
        T_t v0 = m_pattern( x, y, z, t );
        T_t vx = m_pattern( x + tiny, y, z, t );
        T_t vy = m_pattern( x, y + tiny, z, t );
        T_t vz = m_pattern( x, y, z + tiny, t );
        return Imath::Vec3<T_t>( ( vx - v0 ) / tiny,
                                 ( vy - v0 ) / tiny,
                                 ( vz - v0 ) / tiny );
    }
    
    output_type operator()( const Imath::Vec3<T_t> &p,
                            T_t t ) const
    {
        return operator()( p.x, p.y, p.z, t );
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef EvolvingPatternGradient3D<SimplexNoisef> Evnoise3f;
typedef EvolvingPatternGradient3D<SimplexNoised> Evnoise3d;


} // End namespace Noise
} // End namespace EmldCore

#endif
