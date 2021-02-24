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

#ifndef _EmldCore_SpatialSubd_DUOctree_h_
#define _EmldCore_SpatialSubd_DUOctree_h_

#include "Foundation.h"

#include <cassert>
#include <memory>

namespace EmldCore {
namespace SpatialSubd {

//-*****************************************************************************
// DUOctree stands for 'Discrete Uniform Octree'
// This class is NOT threadsafe for writing, but is threadsafe for reading.
template <typename _Tp>
class DUOctree {
protected:
    static const unsigned int toUnsigned(int i) {
        return (unsigned int)(i + INT_MIN);
    }
    static const int toSigned(unsigned int i) {
        return (int)(i - INT_MIN);
    }

    static V3ui toUnsigned(const V3i& iPoint) {
        return V3ui(
          toUnsigned(iPoint.x), toUnsigned(iPoint.y), toUnsigned(iPoint.z));
    }

    static V3i toSigned(const V3ui iUPoint) {
        return V3i(
          toSigned(iUPoint.x), toSigned(iUPoint.y), toSigned(iUPoint.z));
    }

public:
    typedef _Tp* pointer;
    typedef const _Tp* const_pointer;
    typedef _Tp& reference;
    typedef const _Tp& const_reference;
    typedef _Tp value_type;

    DUOctree() {
    }

    void set(const V3i& iPoint, const value_type& iValue);
    bool get(const V3i& iPoint, value_type& oValue) const {
        return m_node ? m_node->get(toUnsigned(iPoint), oValue) : false;
    }

    int level() const {
        return m_node ? m_node->m_level : -1;
    }

protected:
    struct _Node;
    typedef std::shared_ptr<_Node> _Node_ptr;

    struct _Node {
        _Node(_Node_ptr iChild);
        _Node(const V3ui& iUPoint, unsigned int iLevel)
          : m_coord(
              iUPoint.x >> iLevel, iUPoint.y >> iLevel, iUPoint.z >> iLevel)
          , m_level(iLevel) {
        }

        bool set(const V3ui& iUPoint, const value_type& iValue);
        bool get(const V3ui& iUPoint, value_type& oValue) const;

        V3ui m_coord;
        unsigned int m_level;
        value_type m_value;
        _Node_ptr m_children[8];
    };

    _Node_ptr m_node;
};

//-*****************************************************************************
// TEMPLATE AND INLINE FUNCTIONS
//-*****************************************************************************

//-*****************************************************************************
// DUOctree
//-*****************************************************************************

//-*****************************************************************************
// When setting a value, we start with the node which is the base of our
// tree. If the node CAN set the value, it means that the coordinate of the
// value lies within the coordinate bounds of the node, and it will get set
// either on the node directly or on one of its children - but at a leaf.
// If the node exists, but cannot set the value because the coordinate is
// outside its bounds... then we create a new node with bigger bounds and
// push it as the root, making the previous root a child of this new node.
// We keep doing that until the new point fits.
template <typename _Tp>
void DUOctree<_Tp>::set(const V3i& iPoint, const value_type& iValue) {
    V3ui iUPoint = toUnsigned(iPoint);

    if (!m_node) {
        m_node.reset(new _Node(iUPoint, 0));
        bool added = m_node->set(iUPoint, iValue);
        assert(added);
    } else {
        bool added = false;
        do {
            added = m_node->set(iUPoint, iValue);
            if (!added) {
                _Node_ptr child = m_node;
                m_node.reset(new _Node(child));
            }
        } while (!added);
    }
}

//-*****************************************************************************
// NODE
//-*****************************************************************************

//-*****************************************************************************
template <typename _Tp>
DUOctree<_Tp>::_Node::_Node(_Node_ptr iChild)
  : m_level(iChild->m_level + 1) {
    const V3ui& cc = iChild->m_coord;

    m_coord = V3ui(cc.x >> 1, cc.y >> 1, cc.z >> 1);

    // xbit = ( point.x >> level ) & 0x1
    // ybit = ( point.y >> level ) & 0x1
    // zbit = ( point.z >> level ) & 0x1
    // index = ( xbit | ( ybit << 1 ) | ( zbit << 2 ) )
    int index =
      ((((cc.x) & 0x1)) | (((cc.y) & 0x1) << 1) | (((cc.z) & 0x1) << 2));
    assert(index >= 0 && index < 8);

    m_children[index] = iChild;
}

//-*****************************************************************************
// Add a point
template <typename _Tp>
bool DUOctree<_Tp>::_Node::set(const V3ui& iPoint, const value_type& iValue) {
    assert(m_level >= 0);

    // If any of these checks fail for the given coordinate,
    // it means that the point lies outside the domain of this node or
    // its children. (Basically, it's outside our bounding box).
    // That means we can't set.
    if (m_level < 32 &&

        ((iPoint.x >> m_level) != m_coord.x ||
         (iPoint.y >> m_level) != m_coord.y ||
         (iPoint.z >> m_level) != m_coord.z)) {
        return false;
    }

    if (m_level == 0) {
        assert(iPoint == m_coord);
        m_value = iValue;
    } else {
        // xbit = ( point.x >> sublevel ) & 0x1
        // ybit = ( point.y >> sublevel ) & 0x1
        // zbit = ( point.z >> sublevel ) & 0x1
        // index = ( xbit | ( ybit << 1 ) | ( zbit << 2 ) )
        unsigned int sublevel = m_level - 1;
        V3ui cc(
          iPoint.x >> sublevel, iPoint.y >> sublevel, iPoint.z >> sublevel);
        unsigned index =
          (((cc.x & 0x1)) | ((cc.y & 0x1) << 1) | ((cc.z & 0x1) << 2));
        assert(index >= 0 && index < 8);

        if (!m_children[index]) {
            m_children[index].reset(new _Node(iPoint, sublevel));
        }

        assert(m_children[index]);
        const _Node* nptr = m_children[index].get();

        assert(nptr->m_level == sublevel);
        assert(nptr->m_coord == cc);

        bool added = m_children[index]->set(iPoint, iValue);
        assert(added);
    }

    return true;
}

//-*****************************************************************************
// Retrieve a point
template <typename _Tp>
bool DUOctree<_Tp>::_Node::get(const V3ui& iPoint, value_type& oValue) const {
    assert(m_level >= 0);

    if (m_level < 32 &&
        (iPoint.x >> m_level != m_coord.x || iPoint.y >> m_level != m_coord.y ||
         iPoint.z >> m_level != m_coord.z)) {
        return false;
    }

    if (m_level == 0) {
        assert(iPoint == m_coord);
        oValue = m_value;
        return true;
    } else {
        // xbit = ( point.x >> sublevel ) & 0x1
        // ybit = ( point.y >> sublevel ) & 0x1
        // zbit = ( point.z >> sublevel ) & 0x1
        // index = ( xbit | ( ybit << 1 ) | ( zbit << 2 ) )
        unsigned int sublevel = m_level - 1;
        unsigned int index = ((((iPoint.x >> sublevel) & 0x1)) |
                              (((iPoint.y >> sublevel) & 0x1) << 1) |
                              (((iPoint.z >> sublevel) & 0x1) << 2));

        return m_children[index] ? m_children[index]->get(iPoint, oValue)
                                 : false;
    }
}

}  // End namespace SpatialSubd
}  // End namespace EmldCore

#endif
