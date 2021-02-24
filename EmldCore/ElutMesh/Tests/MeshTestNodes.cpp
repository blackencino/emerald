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

#include "MeshTestNodes.h"

namespace MeshTest {

//-*****************************************************************************
// NODE
//-*****************************************************************************

//-*****************************************************************************
void Node::gradientMultipleFiltered(
        std::size_t i_numPoints,
        V3f* o_gradients,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    std::vector<V3f> Paxis( i_numPoints );
    std::vector<float> lsOrig( i_numPoints );
    std::vector<float> lsAxis( i_numPoints );
    std::vector<V3f> Px( i_numPoints );
    std::vector<V3f> Py( i_numPoints );
    std::vector<V3f> Pz( i_numPoints );
    std::vector<float> Dxyz( i_numPoints );

    // Get Dxyzs
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        Dxyz[i] = 0.1f * std::min( std::min( i_dPdus[i].length(),
                                             i_dPdvs[i].length() ),
                                   i_dPdws[i].length() );
        Dxyz[i] = std::max( Dxyz[i], 1.0e-5f );
    }

    // Get level set origs.
    evalMultipleFiltered( i_numPoints, 
                          eu::vector_data( lsOrig ), 1,
                          i_pObjs,
                          i_dPdus,
                          i_dPdvs,
                          i_dPdws,
                          i_time,
                          io_cache );

    //-*************************************************************************
    // Get dx positions
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        Paxis[i] = i_pObjs[i] + V3f( Dxyz[i], 0.0f, 0.0f );
    }

    // Get level set dxs.
    evalMultipleFiltered( i_numPoints, 
                          eu::vector_data( lsAxis ), 1,
                          eu::vector_cdata( Paxis ),
                          i_dPdus,
                          i_dPdvs,
                          i_dPdws,
                          i_time,
                          io_cache );

    // Set x-component of gradient
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        o_gradients[i].x = ( lsAxis[i] - lsOrig[i] ) / Dxyz[i];
    }

    //-*************************************************************************
    // Get dy positions
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        Paxis[i] = i_pObjs[i] + V3f( 0.0f, Dxyz[i], 0.0f );
    }

    // Get level set dys.
    evalMultipleFiltered( i_numPoints, 
                          eu::vector_data( lsAxis ), 1,
                          eu::vector_cdata( Paxis ),
                          i_dPdus,
                          i_dPdvs,
                          i_dPdws,
                          i_time,
                          io_cache );

    // Set y-component of gradient
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        o_gradients[i].y = ( lsAxis[i] - lsOrig[i] ) / Dxyz[i];
    }

    //-*************************************************************************
    // Get dz positions
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        Paxis[i] = i_pObjs[i] + V3f( 0.0f, 0.0f, Dxyz[i] );
    }

    // Get level set dzs.
    evalMultipleFiltered( i_numPoints, 
                          eu::vector_data( lsAxis ), 1,
                          eu::vector_cdata( Paxis ),
                          i_dPdus,
                          i_dPdvs,
                          i_dPdws,
                          i_time,
                          io_cache );

    // Set z-component of gradient
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        o_gradients[i].z = ( lsAxis[i] - lsOrig[i] ) / Dxyz[i];
    }
}

//-*****************************************************************************
// SPHERE NODE
//-*****************************************************************************

//-*****************************************************************************
void SphereNode::evalMultipleFiltered(
        std::size_t i_numPoints,
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    float* outPtr = o_levelSets;
    for ( std::size_t i = 0; i < i_numPoints; ++i, outPtr += i_resultStride )
    {
        (*outPtr) = ( i_pObjs[i] - m_center ).length() - m_radius;
    }
}

//-*****************************************************************************
Intervalf SphereNode::range(
        const V3f i_corners[8],
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    Box3f b;
    b.makeEmpty();
    for ( int i = 0; i < 8; ++i )
    {
        b.extendBy( i_corners[i] );
    }

    Intervalf dx( b.min.x - m_center.x, b.max.x - m_center.x );
    Intervalf dy( b.min.y - m_center.y, b.max.y - m_center.y );
    Intervalf dz( b.min.z - m_center.z, b.max.z - m_center.z );

    float da = dx.min * dx.min;
    float db = dx.max * dx.max;
    Intervalf dx2( std::min( da, db ), std::max( da, db ) );
    if ( dx.intersects( 0.0f ) ) { dx2.min = 0.0f; }

    da = dy.min * dy.min;
    db = dy.max * dy.max;
    Intervalf dy2( std::min( da, db ), std::max( da, db ) );
    if ( dy.intersects( 0.0f ) ) { dy2.min = 0.0f; }

    da = dz.min * dz.min;
    db = dz.max * dz.max;
    Intervalf dz2( std::min( da, db ), std::max( da, db ) );
    if ( dz.intersects( 0.0f ) ) { dz2.min = 0.0f; }

    Intervalf r2( dx2.min + dy2.min + dz2.min,
                  dx2.max + dy2.max + dz2.max );
    Intervalf r( sqrtf( r2.min ), sqrtf( r2.max ) );

    return Intervalf( r.min - m_radius, r.max - m_radius );
}

//-*****************************************************************************
void SphereNode::gradientMultipleFiltered(
        std::size_t i_numPoints,
        V3f* o_gradients,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    V3f* outPtr = o_gradients;
    for ( std::size_t i = 0; i < i_numPoints; ++i, ++outPtr )
    {
        // This will have filtering problems near the center
        (*outPtr) = ( i_pObjs[i] - m_center ).normalized();
    }
}

//-*****************************************************************************
// MIN NODE
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
T SmoothMin( const T& a, const T& b, const T& w )
{
    //const T f = smoothstep( -w/T(2), w/T(2), a-b );
    const T f = smoothstep( -T(2)*w, T(2)*w, a-b );
    return mix( a, b, f );
    //return std::min( a, b );
}

//-*****************************************************************************
// These should have better filtering, really.
void MinNode::evalMultipleFiltered(
        std::size_t i_numPoints,
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    if ( m_nodes.size() < 1 )
    {
        EMLD_THROW( "Can't evaluate an empty Min Node" );
    }

    // Evaluate the first node directly.
    m_nodes.front()->evalMultipleFiltered( 
        i_numPoints, o_levelSets, i_resultStride,
        i_pObjs, i_dPdus, i_dPdvs, i_dPdws, i_time, io_cache );

    if ( m_nodes.size() == 1 ) { return; }

    // Get filter size.  Can do this better in the future.
    std::vector<float> filtWidth( i_numPoints );
    for ( std::size_t i = 0; i < i_numPoints; ++i )
    {
        filtWidth[i] = std::max( std::max( i_dPdus[i].length(),
                                        i_dPdvs[i].length() ),
                                i_dPdws[i].length() );
    }

    // Make temp space.
    std::vector<float> tmpPoints( i_numPoints );
    float* tmpData = eu::vector_data( tmpPoints );

    // Loop over other nodes, eval others, take min.
    for ( Nodes::const_iterator citer = m_nodes.begin() + 1; 
          citer != m_nodes.end(); ++citer )
    {
        (*citer)->evalMultipleFiltered( 
            i_numPoints, tmpData, 1,
            i_pObjs, i_dPdus, i_dPdvs, i_dPdws, i_time, io_cache );

        float *outPtr = o_levelSets;
        float *inPtr = tmpData;
        for ( std::size_t i = 0; i < i_numPoints; 
              ++i, ++inPtr, outPtr += i_resultStride )
        {
            (*outPtr) = SmoothMin( (*inPtr), (*outPtr), filtWidth[i] );
        }
    }


    // Else, check to see if everything is the same.


    // To do better filtering, you'd notice whether the "best" node ever
    // changed within the region, and then you'd do multiple passes
    // based on some filtering configuration.
}

//-*****************************************************************************
Intervalf MinNode::range( const V3f i_corners[8],
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const
{
    Intervalf ret;
    ret.makeEmpty();

    if ( m_nodes.size() < 1 )
    {
        EMLD_THROW( "Can't evaluate an empty Min Node" );
    }

    // Evaluate the first node directly.
    ret = m_nodes.front()->range( i_corners, i_time, io_cache );

    if ( m_nodes.size() == 1 ) { return ret; }

    // Loop over other nodes, eval others, take min.
    for ( Nodes::const_iterator citer = m_nodes.begin() + 1; 
          citer != m_nodes.end(); ++citer )
    {
        Intervalf r2 = (*citer)->range( i_corners, i_time, io_cache );

        ret.min = std::min( ret.min, r2.min );
        ret.max = std::max( ret.max, r2.max );
    }

    return ret;
}

} // End namespace MeshTest

