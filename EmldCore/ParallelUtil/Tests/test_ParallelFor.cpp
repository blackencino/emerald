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
#include <EmldCore/ParallelUtil/For.h>
#include <ImathVec.h>
#include <vector>
#include <iostream>

using namespace EmldCore::Util;
using namespace EmldCore::ParallelUtil;

//-*****************************************************************************
template <typename VEC>
struct ComputeVectorMagnitude
        : public ZeroForEachFunctorI<ComputeVectorMagnitude<VEC> >
{
    typedef ZeroForEachFunctorI<ComputeVectorMagnitude<VEC> > super_type;
    typedef typename super_type::index_type index_type;

    typedef typename VEC::BaseType vec_base_type;

    vec_base_type* Magnitudes;
    VEC* Vectors;

    void operator()( index_type i ) const
    {
        Magnitudes[i] = Vectors[i].length();
    }
};

//-*****************************************************************************
template <typename VEC>
struct IntegratePositionInPlace
        : public ZeroForEachFunctorI<IntegratePositionInPlace<VEC> >
{
    typedef ZeroForEachFunctorI<IntegratePositionInPlace<VEC> > super_type;
    typedef typename super_type::index_type index_type;
    typedef typename VEC::BaseType vec_base_type;

    VEC* Positions;
    VEC* Velocities;
    vec_base_type Dt;

    void operator()( index_type i ) const
    {
        Positions[i] += Velocities[i] * Dt;
    }
};

//-*****************************************************************************
struct SimState
{
    std::vector<Imath::V3f> positions;
    std::vector<Imath::V3f> velocities;
    std::vector<float> velMags;

    SimState( std::size_t i_n )
        : positions( i_n )
        , velocities( i_n )
        , velMags( i_n )
    {
        UniformRand urand( -1.0, 1.0 );
        for ( std::size_t i = 0; i < i_n; ++i )
        {
            positions[i] = Imath::V3f( 100.0 * urand(),
                                       100.0 * urand(),
                                       100.0 * urand() );
            velocities[i] = Imath::V3f( 5.0 * urand(),
                                        5.0 * urand(),
                                        5.0 * urand() );
        }

        computeVelMags();
    }

    void computeVelMags()
    {
        const std::size_t N = velocities.size();
        velMags.resize( N );

        ComputeVectorMagnitude<Imath::V3f> F;
        F.Magnitudes = vector_data( velMags );
        F.Vectors = vector_data( velocities );
        F.execute( N );
    }

    void integrate( float i_dt )
    {
        const std::size_t N = positions.size();

        IntegratePositionInPlace<Imath::V3f> F;
        F.Positions = vector_data( positions );
        F.Velocities = vector_data( velocities );
        F.Dt = i_dt;
        F.execute( N );
    }
};

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    SimState state( 1024 );
    std::cout << "Created sim state with 1024 members." << std::endl;

    state.integrate( 1.0f / 24.0f );
    std::cout << "Integrated state through one frame." << std::endl;

    return 0;
}
