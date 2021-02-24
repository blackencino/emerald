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

#include "ExampleState.h"

//-*****************************************************************************
namespace UTL = EmldCore::Util;
namespace NSE = EmldCore::Noise;

//-*****************************************************************************
namespace EmldCore {
namespace CompactHashMap {
namespace Example {

EMLD_STATLIT_CONSTANT float CELL_SIZE = 5.5f;
EMLD_STATLIT_CONSTANT float POS_DEV = 25.0f;
EMLD_STATLIT_CONSTANT float MASS_MEAN = 1.0f;
EMLD_STATLIT_CONSTANT float MASS_DEV = 0.1f;
EMLD_STATLIT_CONSTANT int P_NSE_OCTAVES = 4;
EMLD_STATLIT_CONSTANT float P_NSE_LAC = 1.67;
EMLD_STATLIT_CONSTANT float P_NSE_OG = 0.5;
EMLD_STATLIT_CONSTANT float P_NSE_SCALE = 2.0f;
EMLD_STATLIT_CONSTANT float P_NSE_OFFSET = 903.1f;
EMLD_STATLIT_CONSTANT float P_NSE_EVOLVE = 0.25f;
EMLD_STATLIT_CONSTANT float P_NSE_AMP = 1.25f;
EMLD_STATLIT_CONSTANT float LIFESPAN_MEAN = 1.0f;
EMLD_STATLIT_CONSTANT float LIFESPAN_DEV = 0.25f;

//-*****************************************************************************
SimpleTestSim::SimpleTestSim( int i_numParts )
    : m_currentTime( 0.0f )
{
    const int N = i_numParts;

    m_vectorManager.reset( new VectorManager );
    m_transform.reset( new SimpleTransform( CELL_SIZE ) );
    m_state.reset( new TestState( *m_vectorManager,
                                  *m_transform ) );

    // Emit.
    {
        emit_mvh emitted = m_vectorManager->get<EmitInfo>( 0 );
        emit( N, 1.0f / 24.0f, emitted );
        m_state->emitUnsorted( emitted );
    }

    // Sort state.
    m_state->sort();
}

//-*****************************************************************************
void SimpleTestSim::emit( int N, float i_dt,
                          emit_mvh o_emitted )
{
    int seed = reinterpret_cast<const int&>( m_currentTime );

    // Get emitted data.
    UTL::GaussRand posGrand( 0.0, POS_DEV, seed );
    UTL::GaussRand mGrand( MASS_MEAN, MASS_DEV, seed + 54321 );
    UTL::UniformRand aGrand( -i_dt, 0.0f, seed + 98832 );
    UTL::GaussRand lGrand( LIFESPAN_MEAN, LIFESPAN_DEV, seed + 991 );

    o_emitted.resize( 0 );

    for ( int i = 0; i < N; ++i )
    {
        o_emitted.push_back(
            EmitInfo( V3f( posGrand(), posGrand(), posGrand() ),
                      V3f( 0.0f ),
                      std::max( ( float )mGrand(), MASS_DEV ),
                      aGrand(),
                      lGrand() ) );
    }
    EMLD_DEBUG_ASSERT( o_emitted.size() == N, "Array size mismatch" );
}

//-*****************************************************************************
template <typename FORCER>
struct Integrate : public ZeroForEachFunctorI<Integrate<FORCER>, int32_t>
{
    V3f* Pos;
    V3f* Vel;
    const float32_t* Mass;

    const FORCER* Forcer;
    float32_t Dt;
    float32_t CurrentTime;

    void operator()( int32_t i ) const
    {
        V3f pi = Pos[i];
        const V3f fi = ( *Forcer )( pi, CurrentTime );
        const V3f ai = fi / Mass[i];
        const V3f vi = Vel[i] + ( Dt * ai );
        pi += Dt * vi;

        Pos[i] = pi;
        Vel[i] = vi;
    }
};

//-*****************************************************************************
struct Age: public ZeroForEachFunctorI<Age, int32_t>
{
    float32_t* Ages;
    float32_t Dt;

    void operator()( int32_t i ) const
    {
        Ages[i] += Dt;
    }
};

//-*****************************************************************************
void SimpleTestSim::unsortedTimeStep( float i_dt )
{
    const int N = m_state->size();

    typedef NSE::StdFbmf PressureBase;
    typedef NSE::XformEvolveBiasGain<PressureBase> Pressure;
    typedef NSE::EvolvingPatternGradient3D<Pressure> PressureGrad;
    PressureBase b( P_NSE_OCTAVES, P_NSE_LAC, P_NSE_OG );

    M44f scale;
    scale.setScale( V3f( P_NSE_SCALE ) );
    M44f trans;
    trans.setTranslation( V3f( P_NSE_OFFSET ) );
    M44f xform = scale * trans;
    Pressure p( b, xform, P_NSE_EVOLVE, 0.0f, P_NSE_AMP );

    PressureGrad g( p );

    {
        Integrate<PressureGrad> F;
        F.Pos = vector_data( m_state->Positions );
        F.Vel = vector_data( m_state->Velocities );
        F.Mass = vector_cdata( m_state->Masses );
        F.Forcer = &g;
        F.CurrentTime = m_currentTime + i_dt;
        F.Dt = i_dt;
        F.execute( N );
    }

    {
        Age F;
        F.Ages = m_state->Ages.data();
        F.Dt = i_dt;
        F.execute( N );
    }

    m_currentTime += i_dt;
}

//-*****************************************************************************
void SimpleTestSim::timeStep( float i_dt )
{
    unsortedTimeStep( i_dt );
    m_state->sort();
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
HashedTestSim::HashedTestSim( int i_numParts )
    : SimpleTestSim( i_numParts )
{
    m_emitRate = (( float )i_numParts ) / LIFESPAN_MEAN;

    m_stateHashMap.reset( new TestStateCHM( *m_state ) );
    m_stateHashMap->initFromStateElements();
}

//-*****************************************************************************
struct KillOld : public ZeroForEachFunctorI<KillOld, int32_t>
{
    mutable int_mvh::ptr_handle_type KilledIndices;
    const float32_t* Ages;
    const float32_t* Lifespans;

    void operator()( int32_t i ) const
    {
        if ( Ages[i] >= Lifespans[i] ) { KilledIndices.push_back( i ); }
    }
};

//-*****************************************************************************
void HashedTestSim::killAndEmit( float i_dt )
{
    int_mvh killedIndices = vectorManager().get<int>( 0 );
    killedIndices.clear();
    emit_mvh emitted = vectorManager().get<EmitInfo>( 0 );
    emitted.clear();

    // N
    const int N = m_state->size();

    // First, kill the old.
    {
        KillOld F;
        F.KilledIndices = killedIndices.ptrHandle();
        F.Ages = m_state->Ages.cdata();
        F.Lifespans = m_state->Lifespans.cdata();
        F.execute( N );
    }

    // Now emit new.
    int numToEmit = ( int )floorf( 0.5f + ( m_emitRate * i_dt ) );
    this->emit( numToEmit, i_dt, emitted );

    std::cout << "About to try killing: " << killedIndices.size()
              << " and emitting: " << emitted.size() << std::endl;

    // Update the hash map with these things.
    m_stateHashMap->killAndEmitStateElements( killedIndices, emitted );
}

//-*****************************************************************************
void HashedTestSim::timeStep( float i_dt )
{
    unsortedTimeStep( i_dt );
    m_stateHashMap->syncFromStateElementsByChangedSimCoords();
    killAndEmit( i_dt );
}

} // End namespace Example
} // End namespace CompactHashMap
} // End namespace EmldCore


