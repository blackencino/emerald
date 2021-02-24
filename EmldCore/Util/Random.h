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

#ifndef _EmldCore_Util_Random_h_
#define _EmldCore_Util_Random_h_

#include "Foundation.h"

#include <random>
#include <cstdint>

namespace EmldCore {
namespace Util {

//-*****************************************************************************
void HashGrid( uint32_t i_seed,
               uint32_t i_x, uint32_t i_y, uint32_t i_z,
               unsigned short o_state[3] );

//-*****************************************************************************
inline int64_t HashGrid( uint32_t i_seed,
                         uint32_t i_x,
                         uint32_t i_y,
                         uint32_t i_z )
{
    unsigned short state[3];
    HashGrid( i_seed, i_x, i_y, i_z, state );

    // Assemble the 48-bit value x[n] from the
    // three 16-bit values stored in state.
    int64_t x = \
        ( int64_t( state[2] ) << 32 ) |
        ( int64_t( state[1] ) << 16 ) |
        ( int64_t( state[0] ) );

    return x;
}

//-*****************************************************************************
typedef std::mt19937_64 BaseRandGenType;
typedef std::uniform_real_distribution<double> UniDistType;

//-*****************************************************************************
class UniformRand
{
public:
    UniformRand( double iMin = 0.0, double iMax = 1.0, int64_t iSeed = 54321 )
      : m_baseGenerator( ( const BaseRandGenType::result_type & )iSeed )
      , m_distribution( iMin, iMax )
    {
        // Nothing
    }

    void reset( int64_t iSeed )
    {
        m_baseGenerator.seed(
            ( const BaseRandGenType::result_type & )iSeed );
        m_distribution.reset();
    }

    void reset( const V3f &iSeed )
    {
        const uint32_t *data = ( const uint32_t * )(&iSeed.x);
        reset( HashGrid( 54321, data[0], data[1], data[2] ) );
    }

    void reset( int64_t i_seed0, const V3f& i_seed1 )
    {
        const uint32_t *data = ( const uint32_t * )(&i_seed1.x);
        reset( HashGrid( i_seed0, data[0], data[1], data[2] ) );
    }

    double operator()( void )
    {
        return m_distribution(m_baseGenerator);
    }

protected:
    BaseRandGenType m_baseGenerator;
    UniDistType m_distribution;
};



//-*****************************************************************************
// Gaussian variation based on floating point input.
typedef std::normal_distribution<double> NormDistType;

//-*****************************************************************************
class GaussRand
{
public:
    GaussRand( double iMean = 0.0, double iDev = 1.0, int64_t iSeed = 54321 )
      : m_baseGenerator( ( const BaseRandGenType::result_type & )iSeed )
      , m_distribution( iMean, iDev  )
    {
        // Nothing
    }

    void reset( int64_t iSeed )
    {
        m_baseGenerator.seed(
            ( const BaseRandGenType::result_type & )iSeed );
        m_distribution.reset();
    }

    void reset( const V3f &iSeed )
    {
        const uint32_t *data = ( const uint32_t * )(&iSeed.x);
        reset( HashGrid( 54321, data[0], data[1], data[2] ) );
    }

    void reset( int64_t i_seed0, const V3f& i_seed1 )
    {
        const uint32_t *data = ( const uint32_t * )(&i_seed1.x);
        reset( HashGrid( i_seed0, data[0], data[1], data[2] ) );
    }

    void reset( int32_t iSeed )
    {
        m_baseGenerator.seed(
            ( const BaseRandGenType::result_type & )iSeed );
        m_distribution.reset();
    }

    double operator()( void )
    {
        return m_distribution(m_baseGenerator);
    }

protected:
    BaseRandGenType m_baseGenerator;
    NormDistType m_distribution;
};

} // End namespace Util
} // End namespace EmldCore

#endif
