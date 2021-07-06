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

#include "Object.h"
#include "ConsistentMesh.h"
#include "InconsistentMesh.h"
#include "RigidMesh.h"
#include "Scene.h"
#include "Xform.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
Object::Object(AbcG::IObject& i_abcObject,
               const OnOffSet& i_onOffSet,
               TopologyMask i_topologyMask,

               Object* i_parentObj,
               Scene& i_scene,
               chrono_t i_initTime)
  : m_abcObject(i_abcObject)
  , m_parent(i_parentObj)
  , m_currentTime(i_initTime)
  , m_minTime(FLT_MAX)
  , m_maxTime(-FLT_MAX) {
    // If no parent, set up sim to local.
    if (!m_parent) {
        m_simToLocal = i_scene.simToScene();
        m_localToSim = i_scene.sceneToSim();
    }

    // Is xform up to this point animating?
    bool animatingXform = m_parent ? m_parent->isXformAnimating() : false;

    // First, set up our internal.
    {
        const AbcG::ObjectHeader& ohead = m_abcObject.getHeader();
        // std::cout << "Visiting object: " << ohead.getName() << std::endl
        //          << "... is animating? " << animatingXform << std::endl;

        if (AbcG::IPolyMesh::matches(ohead) &&
            i_onOffSet.isFullNameIncluded(m_abcObject.getFullName())) {
            // std::cout << "... poly mesh matches" << std::endl;

            AbcG::IPolyMesh pmesh(m_abcObject, AbcG::kWrapExisting);
            AbcG::IPolyMeshSchema& mSchema = pmesh.getSchema();
            const AbcG::MeshTopologyVariance mvar =
              mSchema.getTopologyVariance();
            // std::cout << "... variance: " << ( int )mvar << std::endl;
            if (mvar == AbcG::kConstantTopology) {
                // std::cout << "... constant, i_topologyMask = "
                //          << i_topologyMask << std::endl;

                if ((i_topologyMask == kAnyTopology) ||

                    ((!animatingXform) &&
                     (i_topologyMask == kStaticOnlyTopology)) ||

                    (animatingXform && (i_topologyMask == kRigidOnlyTopology))

                ) {
                    m_internal.reset(new RigidMesh(*this, pmesh, i_scene));
                    constructUserProperties(mSchema.getUserProperties());
                    constructUserProperties(mSchema.getArbGeomParams());
                }
            } else if (mvar == AbcG::kHomogenousTopology) {
                // std::cout << "... homogenous, i_topologyMask = "
                //          << i_topologyMask << std::endl;

                if (i_topologyMask == kAnyTopology ||
                    i_topologyMask == kConsistentOnlyTopology) {
                    m_internal.reset(new ConsistentMesh(*this, pmesh, i_scene));
                    constructUserProperties(mSchema.getUserProperties());
                    constructUserProperties(mSchema.getArbGeomParams());
                }
            } else {
                if (i_topologyMask == kAnyTopology ||
                    i_topologyMask == kInconsistentOnlyTopology) {
                    m_internal.reset(
                      new InconsistentMesh(*this, pmesh, i_scene));
                    constructUserProperties(mSchema.getUserProperties());
                    constructUserProperties(mSchema.getArbGeomParams());
                }
            }

            if (!m_internal) { m_internal.reset(new Internal(*this)); }
        } else if (AbcG::IXform::matches(ohead)) {
            AbcG::IXform xform(m_abcObject, AbcG::kWrapExisting);
            m_internal.reset(new Xform(*this, xform));
            constructUserProperties(xform.getSchema().getUserProperties());
            constructUserProperties(xform.getSchema().getArbGeomParams());
        } else {
            m_internal.reset(new Internal(*this));
        }
    }

    std::cout << "Internal min time: " << m_internal->internalMinTime()
              << std::endl
              << "Internal max time: " << m_internal->internalMaxTime()
              << std::endl;

    // Set up our min & max times from internals
    m_minTime = m_internal->internalMinTime();
    m_maxTime = m_internal->internalMaxTime();

    // Build our matrices from internals..
    setMatrices();

    // Then, build our children.
    size_t numc = m_abcObject.getNumChildren();
    for (size_t i = 0; i < numc; ++i) {
        const AbcG::ObjectHeader& ohead = m_abcObject.getChildHeader(i);
        AbcG::IObject cobj(m_abcObject, ohead.getName());

        ObjectSptr child(new Object(
          cobj, i_onOffSet, i_topologyMask, this, i_scene, i_initTime));
        m_children.push_back(child);

        m_minTime = std::min(m_minTime, child->minTime());
        m_maxTime = std::max(m_maxTime, child->maxTime());
    }

    std::cout << "Object min time: " << m_minTime << std::endl
              << "Object max time: " << m_maxTime << std::endl
              << "Name: " << m_abcObject.getName() << std::endl;
    if (m_parent) {
        std::cout << "Parent name: " << m_parent->m_abcObject.getName()
                  << std::endl;
    } else {
        std::cout << "NULL parent" << std::endl;
    }

    // Then build our bounds.
    setBounds();
}

//-*****************************************************************************
void Object::constructUserProperties(AbcG::ICompoundProperty i_uProp) {
    if (!i_uProp) {
        std::cout << "No user properties on object: " << m_abcObject.getName()
                  << std::endl;
        return;
    }

    size_t numProps = i_uProp.getNumProperties();
    for (size_t i = 0; i < numProps; ++i) {
        const AbcG::PropertyHeader& phead = i_uProp.getPropertyHeader(i);
        std::cout << "Reading user property: " << phead.getName()
                  << " for object: " << m_abcObject.getName() << std::endl;
        if (phead.isScalar() &&
            boost::starts_with(phead.getName(), std::string("EMLD_"))) {
            //-*****************************************************************
            // BOOL goes to BOOL
            //-*****************************************************************
            if (AbcG::IBoolProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IBoolProperty bprop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new BoolPropertyWrapper(bprop, m_currentTime));
                    m_boolProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            // INT32 and INT64 go to INT
            //-*****************************************************************
            if (AbcG::IInt32Property::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IInt32Property prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new Int32PropertyWrapper(prop, m_currentTime));

                    m_intProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IInt64Property::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IInt64Property prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new Int64PropertyWrapper(prop, m_currentTime));

                    m_intProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            // FLOAT and DOUBLE go to FLOAT
            //-*****************************************************************
            if (AbcG::IFloatProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IFloatProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new FloatPropertyWrapper(prop, m_currentTime));

                    m_floatProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IInt64Property::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IDoubleProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new DoublePropertyWrapper(prop, m_currentTime));

                    m_floatProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            // V3f,V3d,P3f,P3d,N3f,N3d,C3f go to VEC
            //-*****************************************************************
            if (AbcG::IV3fProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IV3fProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new V3fPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            // CJH HUGE HACK.
            if (AbcG::IV3fArrayProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IV3fArrayProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new V3fArrayPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IV3dProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IV3dProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new V3dPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IP3fProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IP3fProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new P3fPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IP3dProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IP3dProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new P3dPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IN3fProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IN3fProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new N3fPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IN3dProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IN3dProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new N3dPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            if (AbcG::IC3fProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IC3fProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new C3fPropertyWrapper(prop, m_currentTime));

                    m_vecProperties[shortName] = bpropW;
                }
            }

            //-*****************************************************************
            // STRING goes to STRING
            //-*****************************************************************

            //-*****************************************************************
            if (AbcG::IStringProperty::matches(phead)) {
                std::string shortName =
                  boost::erase_head_copy(phead.getName(), 8);
                if (shortName != "") {
                    AbcG::IStringProperty prop(i_uProp, phead.getName());
                    BasePropertyWrapperSptr bpropW(
                      new StringPropertyWrapper(prop, m_currentTime));

                    m_stringProperties[shortName] = bpropW;
                }
            }
        }
    }  // End loop over user properties.
}

//-*****************************************************************************
void Object::setMatrices() {
    if (m_parent) {
        m_localToSim = parentChildrenToSim();
        m_simToLocal = simToParentChildren();
    }

    // std::cout << "m_localToSim: " << m_localToSim << std::endl;

    if (m_internal->isXform()) {
        m_localToChildren = m_internal->localToInternal();
        m_childrenToLocal = m_internal->internalToLocal();
        m_localToShape = Imath::identity44d;
        m_shapeToLocal = Imath::identity44d;
    } else {
        m_localToChildren = Imath::identity44d;
        m_childrenToLocal = Imath::identity44d;
        m_localToShape = m_internal->localToInternal();
        m_shapeToLocal = m_internal->internalToLocal();
    }

    // std::cout << "m_localToChildren: " << m_localToChildren << std::endl
    //          << "m_localToShape: " << m_localToShape << std::endl;

    m_simToChildren = m_simToLocal * m_localToChildren;
    m_childrenToSim = m_childrenToLocal * m_localToSim;

    m_simToShape = m_simToLocal * m_localToShape;
    m_shapeToSim = m_shapeToLocal * m_localToSim;

    // std::cout << "m_childrenToSim: " << m_childrenToSim << std::endl
    //          << "m_shapeToSim: " << m_shapeToSim << std::endl;
}

//-*****************************************************************************
void Object::setBounds() {
    m_shapeLocalBounds = m_internal->internalBounds();
    m_childrenLocalBounds.makeEmpty();
    for (ObjectSptrVec::iterator iter = m_children.begin();
         iter != m_children.end();
         ++iter) {
        m_childrenLocalBounds.extendBy((*iter)->localBounds());
    }
    m_localBounds.makeEmpty();
    m_localBounds.extendBy(m_shapeLocalBounds);
    m_localBounds.extendBy(m_childrenLocalBounds);
#if 0
    std::cout << "Shape local bounds: " << m_shapeLocalBounds.min
              << " to " << m_shapeLocalBounds.max << std::endl
              << "Children local bounds: " << m_childrenLocalBounds.min
              << " to " << m_childrenLocalBounds.max << std::endl
              << "Local bounds: " << m_localBounds.min
              << " to " << m_localBounds.max << std::endl;

    std::cout << "Shape to sim: " << m_shapeToSim << std::endl
              << "Children to sim: " << m_childrenToSim << std::endl
              << "Local to sim: " << m_localToSim << std::endl
              << "Scene to sim: " << m_sceneToSim << std::endl;
#endif

    m_simBounds.makeEmpty();
    if (!m_localBounds.isEmpty()) {
        m_simBounds = Imath::transform(m_localBounds, m_localToSim);
    }

#ifdef DEBUG_HARD
    if (m_internal->isMesh() && !m_simBounds.isEmpty()) {
        std::cout << "Mesh: " << m_abcObject.getName()
                  << " sim bounds after transform: " << m_simBounds.min
                  << " to " << m_simBounds.max << std::endl;
    }
#endif
}

//-*****************************************************************************
void Object::setTime(chrono_t i_time) {
    // Set time.
    m_currentTime = i_time;

    // Set internal time.
    m_internal->setTime();

    // Set matrices.
    setMatrices();

    // Set all the children times.
    for (ObjectSptrVec::iterator iter = m_children.begin();
         iter != m_children.end();
         ++iter) {
        (*iter)->setTime(i_time);
    }

    // Set bounds.
    setBounds();

    // Set all the property times.

    // Bools
    for (PropMap::iterator miter = m_boolProperties.begin();
         miter != m_boolProperties.end();
         ++miter) {
        (*miter).second->setTime(i_time);
    }

    // Ints
    for (PropMap::iterator miter = m_intProperties.begin();
         miter != m_intProperties.end();
         ++miter) {
        (*miter).second->setTime(i_time);
    }

    // Floats
    for (PropMap::iterator miter = m_floatProperties.begin();
         miter != m_floatProperties.end();
         ++miter) {
        (*miter).second->setTime(i_time);
    }

    // Vecs
    for (PropMap::iterator miter = m_vecProperties.begin();
         miter != m_vecProperties.end();
         ++miter) {
        (*miter).second->setTime(i_time);
    }

    // Strings
    for (PropMap::iterator miter = m_stringProperties.begin();
         miter != m_stringProperties.end();
         ++miter) {
        (*miter).second->setTime(i_time);
    }
}

//-*****************************************************************************
bool Object::hasBoolProperty(const std::string& i_name) const {
    return (m_boolProperties.count(i_name) > 0);
}

//-*****************************************************************************
bool Object::hasIntProperty(const std::string& i_name) const {
    return (m_intProperties.count(i_name) > 0);
}

//-*****************************************************************************
bool Object::hasFloatProperty(const std::string& i_name) const {
    return (m_floatProperties.count(i_name) > 0);
}

//-*****************************************************************************
bool Object::hasVecProperty(const std::string& i_name) const {
    return (m_vecProperties.count(i_name) > 0);
}

//-*****************************************************************************
bool Object::hasStringProperty(const std::string& i_name) const {
    return (m_stringProperties.count(i_name) > 0);
}

//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
bool Object::boolProperty(const std::string& i_name) const {
    PropMap::const_iterator fiter = m_boolProperties.find(i_name);
    if (fiter != m_boolProperties.end()) {
        return (*fiter).second->getValueBool();
    } else {
        return false;
    }
}

//-*****************************************************************************
int Object::intProperty(const std::string& i_name) const {
    PropMap::const_iterator fiter = m_intProperties.find(i_name);
    if (fiter != m_intProperties.end()) {
        return (*fiter).second->getValueInt();
    } else {
        return 0;
    }
}

//-*****************************************************************************
float Object::floatProperty(const std::string& i_name) const {
    PropMap::const_iterator fiter = m_floatProperties.find(i_name);
    if (fiter != m_floatProperties.end()) {
        return (*fiter).second->getValueFloat();
    } else {
        return 0.0f;
    }
}

//-*****************************************************************************
V3f Object::vecProperty(const std::string& i_name) const {
    PropMap::const_iterator fiter = m_vecProperties.find(i_name);
    if (fiter != m_vecProperties.end()) {
        return (*fiter).second->getValueVec();
    } else {
        return V3f(0.0f);
    }
}

//-*****************************************************************************
std::string Object::stringProperty(const std::string& i_name) const {
    PropMap::const_iterator fiter = m_stringProperties.find(i_name);
    if (fiter != m_stringProperties.end()) {
        return (*fiter).second->getValueString();
    } else {
        return std::string("");
    }
}

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore
