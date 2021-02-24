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

#ifndef _EmldCore_AbcMeshesScene_Object_h_
#define _EmldCore_AbcMeshesScene_Object_h_

#include "Foundation.h"
#include "FileInfo.h"
#include "PropertyWrapper.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
class Scene;

//-*****************************************************************************
class Object;
typedef ABCM_SHARED_PTR<Object> ObjectSptr;
typedef std::vector<ObjectSptr> ObjectSptrVec;

//-*****************************************************************************
enum TopologyMask
{
    kAnyTopology,
    kStaticOnlyTopology,
    kRigidOnlyTopology,
    kConsistentOnlyTopology,
    kInconsistentOnlyTopology
};

//-*****************************************************************************
// Objects have shared pointers to their children.
// Objects have a non-shared pointer to their parent object.
class Object
{
public:
    Object( AbcG::IObject& i_abcObject,
            const OnOffSet& i_onOffSet,
            TopologyMask i_topologyMask,

            Object* i_parentObj,
            Scene& i_scene,
            chrono_t i_initTime );

    ~Object() {}

    void setTime( chrono_t i_time );

    chrono_t currentTime() const { return m_currentTime; }
    chrono_t minTime() const { return m_minTime; }
    chrono_t maxTime() const { return m_maxTime; }

    const M44d& localToSim() const { return m_localToSim; }
    const M44d& simToLocal() const { return m_simToLocal; }

    const M44d& localToChildren() const { return m_localToChildren; }
    const M44d& childrenToLocal() const { return m_childrenToLocal; }

    const M44d& localToShape() const { return m_localToShape; }
    const M44d& shapeToLocal() const { return m_shapeToLocal; }

    const M44d& childrenToSim() const { return m_childrenToSim; }
    const M44d& simToChildren() const { return m_simToChildren; }

    const M44d& shapeToSim() const { return m_shapeToSim; }
    const M44d& simToShape() const { return m_simToShape; }

    const Box3d& childrenLocalBounds() const { return m_childrenLocalBounds; }
    const Box3d& shapeLocalBounds() const { return m_shapeLocalBounds; }
    const Box3d& localBounds() const { return m_localBounds; }

    const Box3d& simBounds() const { return m_simBounds; }

    bool isXformAnimating() const
    {
        return ( m_internal->isXform() &&
                 m_internal->internalIsAnimating() ) ||
               ( m_parent && m_parent->isXformAnimating() );
    }

    //-*************************************************************************
    // PROPERTY STUFF
    //-*************************************************************************

    bool hasBoolProperty( const std::string& i_name ) const;
    bool hasIntProperty( const std::string& i_name ) const;
    bool hasFloatProperty( const std::string& i_name ) const;
    bool hasVecProperty( const std::string& i_name ) const;
    bool hasStringProperty( const std::string& i_name ) const;

    //-*************************************************************************

    bool boolProperty( const std::string& i_name ) const;
    int intProperty( const std::string& i_name ) const;
    float floatProperty( const std::string& i_name ) const;
    V3f vecProperty( const std::string& i_name ) const;
    std::string stringProperty( const std::string& i_name ) const;

    //-*************************************************************************
    // END PROPERTY STUFF
    //-*************************************************************************


protected:
    M44d parentChildrenToSim() const
    {
        if ( m_parent )
        {
            return m_parent->childrenToSim();
        }
        else { return m_localToSim; }
    }

    M44d simToParentChildren() const
    {
        if ( m_parent )
        {
            return m_parent->simToChildren();
        }
        else { return m_simToLocal; }
    }

    void constructUserProperties( AbcG::ICompoundProperty i_uProp );
    void setMatrices();
    void setBounds();

public:
    class Internal
    {
    public:
        Internal( Object& i_enclosingObject )
            : m_enclosingObject( i_enclosingObject )
        {
            m_internalBounds.makeEmpty();
            m_internalToLocal = Imath::identity44d;
            m_localToInternal = Imath::identity44d;
            m_internalMinTime = FLT_MAX;
            m_internalMaxTime = -FLT_MAX;
        }
        virtual ~Internal() {}

        virtual bool isXform() const { return false; }
        virtual bool isMesh() const { return false; }

        Object& enclosingObject() { return m_enclosingObject; }
        const Object& enclosingObject() const { return m_enclosingObject; }

        const Box3d& internalBounds() const { return m_internalBounds; }
        const M44d& internalToLocal() const { return m_internalToLocal; }
        const M44d& localToInternal() const { return m_localToInternal; }

        chrono_t internalMinTime() const { return m_internalMinTime; }
        chrono_t internalMaxTime() const { return m_internalMaxTime; }
        bool internalIsAnimating() const
        { return m_internalMaxTime > m_internalMinTime; }

    protected:
        friend class Object;
        virtual void setTime()
        {
            m_internalBounds.makeEmpty();
            m_internalToLocal = Imath::identity44d;
            m_localToInternal = Imath::identity44d;
        }

    protected:
        Object& m_enclosingObject;
        Box3d m_internalBounds;
        M44d m_internalToLocal;
        M44d m_localToInternal;
        chrono_t m_internalMinTime;
        chrono_t m_internalMaxTime;
    };

protected:
    typedef std::map<std::string, BasePropertyWrapperSptr> PropMap;

    AbcG::IObject m_abcObject;
    Object* m_parent;
    ABCM_UNIQUE_PTR<Internal> m_internal;
    ObjectSptrVec m_children;

    // Properties
    PropMap m_boolProperties;
    PropMap m_intProperties;
    PropMap m_floatProperties;
    PropMap m_vecProperties;
    PropMap m_stringProperties;

    // Times
    chrono_t m_currentTime;
    chrono_t m_minTime;
    chrono_t m_maxTime;

    // The matrices.
    M44d m_sceneToSim;
    M44d m_simToScene;

    M44d m_localToSim;
    M44d m_simToLocal;

    M44d m_localToChildren;
    M44d m_childrenToLocal;

    M44d m_localToShape;
    M44d m_shapeToLocal;

    M44d m_simToChildren;
    M44d m_childrenToSim;

    M44d m_simToShape;
    M44d m_shapeToSim;

    // The bounds
    Box3d m_childrenLocalBounds;
    Box3d m_shapeLocalBounds;
    Box3d m_localBounds;
    Box3d m_simBounds;
};

} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
