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

#ifndef _EmldCore_ElutMesh_MeshTestNodes_h_
#define _EmldCore_ElutMesh_MeshTestNodes_h_

#include "MeshTestFoundation.h"

namespace MeshTest {

//-*****************************************************************************
class ThreadSafeCache
{
public:
    ThreadSafeCache() {}
    virtual ~ThreadSafeCache() {}
};

//-*****************************************************************************
struct TimeInterval
{
    TimeInterval() : tbegin( 0.0f ), tend( 0.0f ) {}

    float tbegin;
    float tend;
};

//-*****************************************************************************
class Node
{
public:
    typedef float value_type;

    Node() : m_verbose( false ) {}
    virtual ~Node() {}

    bool verbose() const { return m_verbose; }
    void setVerbose( bool v ) { m_verbose = v; }

    //-*************************************************************************
    virtual void evalMultipleFiltered(

        // Number of points to evaluate
        std::size_t i_numPoints,

        // Where to put the results, and
        // the stride for the output array
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,

        // The positions & filter axes of the
        // points.
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,

        // The time interval of the request
        const TimeInterval& i_time,

        // The cache. Only the cache can
        // be modified, but it can store
        // anything.
        ThreadSafeCache& io_cache ) const = 0;

    //-*************************************************************************
    // Return a range for the values.
    virtual Intervalf range(

        // The corners of the request volume.
        const V3f i_corners[8],

        // The time interval of the request.
        const TimeInterval& i_time,

        // The cache. Only the cache can
        // be modified, but it can store
        // anything.
        ThreadSafeCache& io_cache ) const = 0;

    //-*************************************************************************
    // Default implementation will use the filter axes to create an offset.
    // If the filter axes are degenerate, it will get confused.
    virtual void gradientMultipleFiltered(

        // The number of points to evaluate
        std::size_t i_numPoints,

        // The output gradients.
        V3f* o_gradients,

        // The positions & filter axes of the
        // points.
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,

        // The time interval of the request
        const TimeInterval& i_time,

        // The cache. Only the cache can
        // be modified, but it can store
        // anything.
        ThreadSafeCache& io_cache ) const;

protected:
    bool m_verbose;
};

//-*****************************************************************************
// A node wrapper that encapsulates time and cache, to mimic the 
// expectations of the refinery node concept.
class NodeWrapper
{
public:
    typedef Node::value_type value_type;

    NodeWrapper( const Node& i_node, 
                 const TimeInterval& i_time,
                 ThreadSafeCache& io_cache )
        : m_node( i_node )
        , m_time( i_time )
        , m_cache( io_cache )
        {}

    bool verbose() const { return m_node.verbose(); }

    void evalMultipleFiltered( std::size_t i_numPoints,
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws )
    {
        m_node.evalMultipleFiltered( i_numPoints, o_levelSets, i_resultStride,
                                     i_pObjs, i_dPdus, i_dPdvs, i_dPdws,
                                     m_time,
                                     m_cache );
    }

    Intervalf range( const V3f i_corners[8] )
    {
        return m_node.range( i_corners, m_time, m_cache );
    }

    void gradientMultipleFiltered( std::size_t i_numPoints,
        V3f* o_gradients,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws )
    {
        m_node.gradientMultipleFiltered( i_numPoints, o_gradients,
                                         i_pObjs, i_dPdus, i_dPdvs, i_dPdws,
                                         m_time,
                                         m_cache );
    }

protected:
    const Node& m_node;
    TimeInterval m_time;
    ThreadSafeCache& m_cache;
};

//-*****************************************************************************
class SphereNode : public Node
{
public:
    SphereNode( const V3f& i_center, float i_radius )
        : Node()
        , m_center( i_center )
        , m_radius( i_radius )
    {}

    virtual void evalMultipleFiltered(
        std::size_t i_numPoints,
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const;

    virtual Intervalf range(
        const V3f i_corners[8],
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const;

    virtual void gradientMultipleFiltered(
        std::size_t i_numPoints,
        V3f* o_gradients,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const;

    Box3f bounds() const
    {
        return Box3f( m_center - V3f( m_radius ),
                      m_center + V3f( m_radius ) );
    }

    void moveUp( float i_by = 0.1f )
    {
        m_center += V3f( 0.0f, 0.0f, i_by );
    }

    void moveDown( float i_by = 0.1f )
    {
        m_center -= V3f( 0.0f, 0.0f, i_by );
    }

protected:
    V3f m_center;
    float m_radius;
};

//-*****************************************************************************
class MinNode : public Node
{
public:
    typedef std::vector<const Node*> Nodes;

    MinNode() : Node() {}

    void addNode( const Node* i_node ) { m_nodes.push_back( i_node ); }

    virtual void evalMultipleFiltered(
        std::size_t i_numPoints,
        float* o_levelSets,
        std::ptrdiff_t i_resultStride,
        const V3f* i_pObjs,
        const V3f* i_dPdus,
        const V3f* i_dPdvs,
        const V3f* i_dPdws,
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const;

    virtual Intervalf range(
        const V3f i_corners[8],
        const TimeInterval& i_time,
        ThreadSafeCache& io_cache ) const;

protected:
    Nodes m_nodes;
};

} // End namespace MeshTest

#endif
