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

#ifndef _EmldCore_AbcMeshesScene_Mesh_h_
#define _EmldCore_AbcMeshesScene_Mesh_h_

#include "Foundation.h"
#include "Object.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
class Mesh;

//-*****************************************************************************
template <typename T>
struct MeshPoint
{
    MeshPoint() 
        : simPoint( ((T)0.0) )
        , simVelocity( ((T)0.0) )
        , shapePoint( ((T)0.0) )
        , barycentric( ((T)0.0) )
        , triId( -1 )
        , meshId( -1 )
        {}

    MeshPoint( const Imath::Vec3<T>& i_sp )
        : simPoint( i_sp )
        , simVelocity( ((T)0.0) )
        , shapePoint( ((T)0.0) )
        , barycentric( ((T)0.0) )
        , triId( -1 )
        , meshId( -1 )
        {}

    Imath::Vec3<T> simPoint;
    Imath::Vec3<T> simVelocity;
    Imath::Vec3<T> shapePoint;
    Imath::Vec3<T> barycentric;

    int triId;
    int meshId;
};

template <typename T>
struct BestMeshPoint : public MeshPoint<T>
{
    BestMeshPoint()
        : MeshPoint<T>()
        , bestSquaredDist( FLT_MAX )
        , maxSquaredDist( FLT_MAX )
        , bestTriangle( NULL )
        , bestMesh( NULL )
        {}

    BestMeshPoint( const Imath::Vec3<T>& i_sp ) 
        : MeshPoint<T>( i_sp )
        , bestSquaredDist( FLT_MAX )
        , maxSquaredDist( FLT_MAX )
        , bestTriangle( NULL )
        , bestMesh( NULL )
        {}

    BestMeshPoint( const Imath::Vec3<T>& i_sp, T i_d2 ) 
        : MeshPoint<T>( i_sp )
        , bestSquaredDist( i_d2 )
        , maxSquaredDist( i_d2 )
        , bestTriangle( NULL )
        , bestMesh( NULL )
        {}

    Imath::Vec3<T> bestPoint;
    T bestSquaredDist;
    T maxSquaredDist;
    const Etm::Triangle* bestTriangle;
    const Mesh* bestMesh;
};

typedef MeshPoint<float> MeshPointF;
typedef MeshPoint<double> MeshPointD;

typedef BestMeshPoint<float> BestMeshPointF;
typedef BestMeshPoint<double> BestMeshPointD;

//-*****************************************************************************
class Mesh : public Object::Internal
{
public:
    Mesh( Object& i_enclosingObject,
          AbcG::IPolyMesh& i_abcPolyMesh,
          Scene& i_scene );

    int meshId() const { return m_meshId; }
    void setMeshId( int i_mid ) { m_meshId = i_mid; }

    const TriMesh* triMesh() const { return m_triMesh.get(); }

    virtual bool isMesh() const { return true; }

    //-*************************************************************************
    // OBJECT XFORM STUFF
    //-*************************************************************************

    const M44d& localToSim() const
    { return m_enclosingObject.localToSim(); }
    const M44d& simToLocal() const
    { return m_enclosingObject.simToLocal(); }

    const M44d& localToChildren() const
    { return m_enclosingObject.localToChildren(); }
    const M44d& childrenToLocal() const
    { return m_enclosingObject.childrenToLocal(); }

    const M44d& localToShape() const
    { return m_enclosingObject.localToShape(); }
    const M44d& shapeToLocal() const
    { return m_enclosingObject.shapeToLocal(); }

    const M44d& childrenToSim() const
    { return m_enclosingObject.childrenToSim(); }
    const M44d& simToChildren() const
    { return m_enclosingObject.simToChildren(); }

    const M44d& shapeToSim() const
    { return m_enclosingObject.shapeToSim(); }
    const M44d& simToShape() const
    { return m_enclosingObject.simToShape(); }

    const Box3d& childrenLocalBounds() const
    { return m_enclosingObject.childrenLocalBounds(); }
    const Box3d& shapeLocalBounds() const
    { return m_enclosingObject.shapeLocalBounds(); }
    const Box3d& localBounds() const
    { return m_enclosingObject.localBounds(); }

    const Box3d& simBounds() const
    { return m_enclosingObject.simBounds(); }


    //-*************************************************************************
    // REGION STUFF
    //-*************************************************************************

    // Is the mesh static?
    bool isStatic() const;

    // Is the region rigid?
    bool isRigid() const;

    // Is the region consistent (topologically)? This will return true
    // for rigid or deforming-consistent
    bool isConsistent() const;

    // Definitely NOT rigid!
    bool isConsistentAndNotRigid() const;

    // Transform a shape point to a sim point.
    V3d transformShapePointToSim( const V3d& i_shapePoint ) const;

    V3d transformBarycentricToSim( const V3d& i_barycentric, 
                                   int i_triIndex ) const;

    // Is the space inside a given set of bounds fully inside this region?
    // If we assume the mesh is closed - the triMesh intersection functions
    // return whether any of the triangles are inside the box. If the box
    // is not totally outside our bounds, and the box does not intersect
    // the mesh, then we just test the bounds center against the mesh inside.
    bool areBoundsFullyInside( const Box3d& i_bounds ) const;

    // Is the space inside a given set of bounds fully outside this region?
    bool areBoundsFullyOutside( const Box3d& i_bounds ) const;

    // Does the region intersect the bounds?
    // We treat this as an interior test.
    bool intersects( const Box3d& i_bounds ) const;

    // Does the region intersect the point?
    bool intersects( const V3d& i_point,
                     V3d& o_shapePoint ) const;
    bool intersects( const V3d& i_point ) const
    {
        V3d dummy;
        return intersects( i_point, dummy );
    } 

    // Does the region intersect the point, with closest point info...
    bool intersects( const V3d& i_point,
                     BestMeshPointD& o_bestMeshPoint ) const;

    // Does a narrow interior band of the region intersect the bounds?
    // This is an optimization only, and by default just returns intersects.
    bool intersectsInnerNarrowBand( const Box3d& i_bounds,
                                    double i_narrowBand2 ) const;

    // Does a narrow interior band of the region intersect the point?
    bool intersectsInnerNarrowBand( const V3d& i_point,
                                    double i_narrowBand2,
                                    BestMeshPointD& o_bestMeshPoint ) const;

    // Simple attribute stuff.
    // All default to false.
    bool hasBoolProperty( const std::string& i_propName ) const
    { return m_enclosingObject.hasBoolProperty( i_propName ); }
    bool hasIntProperty( const std::string& i_propName ) const
    { return m_enclosingObject.hasIntProperty( i_propName ); }
    bool hasFloatProperty( const std::string& i_propName ) const
    { return m_enclosingObject.hasFloatProperty( i_propName ); }
    bool hasVecProperty( const std::string& i_propName ) const
    { return m_enclosingObject.hasVecProperty( i_propName ); }
    bool hasStringProperty( const std::string& i_propName ) const
    { return m_enclosingObject.hasStringProperty( i_propName ); }

    // All return 0 or "".
    bool boolProperty( const std::string& i_propName ) const
    { return m_enclosingObject.boolProperty( i_propName ); }
    int intProperty( const std::string& i_propName ) const
    { return m_enclosingObject.intProperty( i_propName ); }
    float floatProperty( const std::string& i_propName ) const
    { return m_enclosingObject.floatProperty( i_propName ); }
    V3f vecProperty( const std::string& i_propName ) const
    { return m_enclosingObject.vecProperty( i_propName ); }
    std::string stringProperty( const std::string& i_propName ) const
    { return m_enclosingObject.stringProperty( i_propName ); }

protected:
    void extractScale( bool i_set );
    virtual void setTime();

    AbcG::IPolyMesh m_abcPolyMesh;
    AbcG::MeshTopologyVariance m_abcPolyMeshVariance;

    int m_meshId;

    // TriMesh points are in scaled local position.
    // That means they're read from the file and have the
    // scale matrix applied to them.
    V3d m_scale;

    // The tri-mesh.
    ABCM_UNIQUE_PTR<TriMesh> m_triMesh;
};

} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
