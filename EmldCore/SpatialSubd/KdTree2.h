#ifndef _EmldCore_SpatialSubd_KdTree2_h_
#define _EmldCore_SpatialSubd_KdTree2_h_

#include "Foundation.h"

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathVec.h>

#include <algorithm>
#include <cassert>
#include <vector>

namespace EmldCore {
namespace SpatialSubd {

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// BOUNDERS and SORT_POINTERS
//
// All of the trees in this library rely on functors which, given an iterator,
// will return the bounding box of that iterator and the "sort point" (the
// point which the bounding boxes are sorted into sub-trees by).
//
// The simplest case, a point tree, in which the elements of the tree
// are simply points - the bounds and sort-point of the elements are simply:
// Box<Vec2<T> >( *iter ) and (*iter) respectively.
//
// In a more complex case - suppose that your iterators point to some sort
// of index into a separate array. The bounds and sort_pt functors passed
// into the constructor for the Tree would contain references to the
// array that the indices refer to, and would use these arrays when returning
// the bounds or sort_pt. This is entirely up to the user.
// However, there is rarely a situation in which an array of direct pointers
// to the elements being sorted would not be the most ideal approach.
//
// Suppose you're sorting pointers to "MyThing" in this tree, and that
// your "MyThing" has member functions "bounds()" and "center()". You could
// create the appropriate functors like this:
//
// struct MyThingBounds
// {
//     Imath::Box2f operator()( const MyThing *a ) const
//     { return a->bounds(); }
// };
//
// struct MyThingsSortPt
// {
//     Imath::Vec2f operator()( const MyThing *a ) const
//     { return a->center(); }
// };
//
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// IN-PLACE KD TREE
// Sorts iterators by pointer directly into a KdTree.
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class T, class LEAF, class BOUNDS, class SORT_PT>
class KdTree2 {
public:
    typedef T value_type;
    typedef LEAF leaf_type;
    typedef BOUNDS bounds_type;
    typedef SORT_PT sort_point_type;

    typedef typename Imath::Vec2<T> V2T;
    typedef typename Imath::Box<V2T> B2T;

    typedef LEAF* iterator;
    typedef const LEAF* const_iterator;

    typedef KdTree2<T, LEAF, BOUNDS, SORT_PT> this_type;

    KdTree2(iterator Begin,
            iterator End,
            const BOUNDS& bounder,
            const SORT_PT& sorter,
            int maxPerLeaf = 2,
            int maxSubDivs = 32) {
        init(Begin, End, bounder, sorter, maxPerLeaf, maxSubDivs);
    }

    KdTree2(std::vector<LEAF>& vec,
            const BOUNDS& bounder,
            const SORT_PT& sorter,
            int maxPerLeaf = 2,
            int maxSubDivs = 32) {
        LEAF* begin = NULL;
        LEAF* end = NULL;
        if (vec.size() > 0) {
            begin = &(vec.front());
            end = begin + vec.size();
        }
        init(begin, end, bounder, sorter, maxPerLeaf, maxSubDivs);
    }

    ~KdTree2();

    // Basic Data Access.
    bool isLeaf() const {
        return m_isLeaf;
    }
    iterator begin() {
        assert(m_isLeaf);
        return m_data.leaf.begin;
    }
    const_iterator begin() const {
        assert(m_isLeaf);
        return m_data.leaf.begin;
    }
    iterator end() {
        assert(m_isLeaf);
        return m_data.leaf.end;
    }
    const_iterator end() const {
        assert(m_isLeaf);
        return m_data.leaf.end;
    }

    const B2T& bounds() const {
        return m_bounds;
    }

    // Left and right access. These are named
    // obscurely so that derived classes can use the left_and_right
    // class to provide upcasted pointers.
    this_type* left() {
        assert(!m_isLeaf);
        return m_data.branch.left;
    }
    const this_type* left() const {
        assert(!m_isLeaf);
        return m_data.branch.left;
    }
    this_type* right() {
        assert(!m_isLeaf);
        return m_data.branch.right;
    }
    const this_type* right() const {
        assert(!m_isLeaf);
        return m_data.branch.right;
    }

    //-*************************************************************************
    //-*************************************************************************
    // BASIC GATHER FUNCTIONS
    // These function takes a functor which considers possibly valid members
    // of the tree, as well as a bounds (or a point) for the check region.
    // These functions are recursive.
    //-*************************************************************************
    //-*************************************************************************
    template <class CHECKER>
    void gatherByBounds(const B2T& bnds, CHECKER& check) const;

    template <class CHECKER>
    void gatherByPoint(const V2T& pnt, CHECKER& check) const;

    //-*************************************************************************
    //-*************************************************************************
    // OKAY, BETTER TOOLS! We basically want a functor which can decide
    // whether to consider a tree based on bounds alone, and then
    // can consume each leaf. We can't doubly overload the operator()
    // for bounds and leaves, because if we have a tree of bounding boxes
    // it'll be ambiguous. What are our choices? We'll assert the interface
    // for a Tree Traversal Functor:
    //
    // interface TreeTraversalFunctor
    // {
    //     // Return true if values that are within the bounds are worth
    //     // traversing
    //     bool validBounds( const B2T &bnds );
    //
    //     // Traverse a leaf
    //     // It is expected that the functor will potentially alter itself.
    //     void leaf( LEAF &a );        // For traverse
    //     void leaf( const LEAF &a );  // For constTraverse
    //
    //     // Is the traversal done? Useful for "first hit"-type queries.
    //     // should be fast-ish.
    //     bool unfinished()
    // }
    //
    // There will be a few handy traversals down below.
    //-*************************************************************************
    template <class TRAVERSER>
    void constTraverse(TRAVERSER& trav) const;

    template <class TRAVERSER>
    void traverse(TRAVERSER& trav);

protected:
    // Sorters.
    struct LeafCompX {
        LeafCompX(const SORT_PT& sp)
          : sortPt(sp) {
        }
        bool operator()(const LEAF& a, const LEAF& b) const {
            return sortPt(a).x < sortPt(b).x;
        }
        const SORT_PT& sortPt;
    };

    struct LeafCompY {
        LeafCompY(const SORT_PT& sp)
          : sortPt(sp) {
        }
        bool operator()(const LEAF& a, const LEAF& b) const {
            return sortPt(a).y < sortPt(b).y;
        }
        const SORT_PT& sortPt;
    };

    // Init function
    void init(iterator Begin,
              iterator End,
              const BOUNDS& bnd,
              const SORT_PT& srt,
              int maxPerLeaf,
              int maxSubDivs);

    // Data
    B2T m_bounds;

    union {
        struct {
            iterator begin;
            iterator end;
        } leaf;

        struct {
            this_type* left;
            this_type* right;
        } branch;
    } m_data;

    bool m_isLeaf;
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// TEMPLATE AND INLINE FUNCTIONS
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class T, class LEAF, class BND, class SP>
void KdTree2<T, LEAF, BND, SP>::init(iterator Begin,
                                     iterator End,
                                     const BND& bnd,
                                     const SP& sp,
                                     int maxPerLeaf,
                                     int maxSubDivs) {
    // Fix inputs.
    maxPerLeaf = std::max(maxPerLeaf, 2);

    // Get bounds.
    m_bounds.makeEmpty();
    for (iterator iter = Begin; iter != End; ++iter) {
        m_bounds.extendBy(bnd((*iter)));
    }

    // Decide whether or not to split.
    int N = (int)(End - Begin);
    if (N > maxPerLeaf && maxSubDivs > 0) {
        int halfN = N / 2;
        iterator nthIter = Begin + halfN;

        // Split along the largest axis of the bounds.
        // Do this by median sorting based on the appropriate axis.
        V2T bsze = m_bounds.size();
        if (bsze.x > bsze.y) {
            // X is the biggest axis.
            LeafCompX lc(sp);
            std::nth_element(Begin, nthIter, End, lc);
        } else {
            // Y is the biggest axis.
            LeafCompY lc(sp);
            std::nth_element(Begin, nthIter, End, lc);
        }

        // Okay, now they're median sorted about the split
        // point, based on the axes.
        // Make children.
        m_isLeaf = false;
        m_data.branch.left =
          new this_type(Begin, nthIter, bnd, sp, maxPerLeaf, maxSubDivs - 1);

        m_data.branch.right =
          new this_type(nthIter, End, bnd, sp, maxPerLeaf, maxSubDivs - 1);
    } else {
        // We're a leaf! Yay!
        m_isLeaf = true;
        m_data.leaf.begin = Begin;
        m_data.leaf.end = End;
    }
}

//-*****************************************************************************
template <class T, class LEAF, class BND, class SP>
KdTree2<T, LEAF, BND, SP>::~KdTree2() {
    if (!m_isLeaf) {
        delete m_data.branch.left;
        delete m_data.branch.right;
    }
}

//-*****************************************************************************
template <class T, class LEAF, class BND, class SP>
template <class CHECKER>
void KdTree2<T, LEAF, BND, SP>::gatherByBounds(const B2T& bnds,
                                               CHECKER& chk) const {
    // Check intersection first.
    if (!m_bounds.intersects(bnds)) { return; }

    if (m_isLeaf) {
        for (const_iterator iter = m_data.leaf.begin; iter != m_data.leaf.end;
             ++iter) {
            chk((*iter));
        }
    } else {
        m_data.branch.left->gatherByBounds(bnds, chk);
        m_data.branch.right->gatherByBounds(bnds, chk);
    }
}

//-*****************************************************************************
template <class T, class LEAF, class BND, class SP>
template <class CHECKER>
void KdTree2<T, LEAF, BND, SP>::gatherByPoint(const V2T& pnt,
                                              CHECKER& chk) const {
    // Check intersection first.
    if (!m_bounds.intersects(pnt)) { return; }

    if (m_isLeaf) {
        for (const_iterator iter = m_data.leaf.begin; iter != m_data.leaf.end;
             ++iter) {
            chk((*iter));
        }
    } else {
        m_data.branch.left->gatherByPoint(pnt, chk);
        m_data.branch.right->gatherByPoint(pnt, chk);
    }
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// Most Common Traversals are ones which compare based on bounds or point.
//
// BoundsTraverser is a traverser with bounds for the bound check.
//
// Note that it still requires a leaf function and a bool operator.
template <class T>
class BoundsTraverser2 {
public:
    typedef T value_type;
    typedef Imath::Vec2<T> V2T;
    typedef Imath::Box<V2T> B2T;

    BoundsTraverser2(const B2T& testBnds)
      : m_testBounds(testBnds) {
    }

    const B2T& testBounds() const {
        return m_testBounds;
    }

    bool validBounds(const B2T& bnds) const {
        return m_testBounds.intersects(bnds);
    }

protected:
    B2T m_testBounds;
};

//-*****************************************************************************
template <class T>
class PointTraverser2 {
public:
    typedef T value_type;
    typedef Imath::Vec2<T> V2T;
    typedef Imath::Box<V2T> B2T;

    PointTraverser2(const V2T& testPt)
      : m_testPoint(testPt) {
    }

    const V2T& testPoint() const {
        return m_testPoint;
    }

    bool validBounds(const B2T& bnds) const {
        return bnds.intersects(m_testPoint);
    }

protected:
    V2T m_testPoint;
};

//-*****************************************************************************
// The great thing is that traversal is so easy!!!
template <class T, class LEAF, class BND, class SP>
template <class TRAVERSER>
void KdTree2<T, LEAF, BND, SP>::constTraverse(TRAVERSER& trav) const {
    if (!trav.validBounds(m_bounds)) { return; }

    if (m_isLeaf) {
        for (const_iterator iter = m_data.leaf.begin;
             trav.unfinished() && iter != m_data.leaf.end;
             ++iter) {
            trav.leaf((*iter));
        }
    } else {
        if (trav.unfinished()) { m_data.branch.left->constTraverse(trav); }
        if (trav.unfinished()) { m_data.branch.right->constTraverse(trav); }
    }
}

//-*****************************************************************************
// The great thing is that traversal is so easy!!!
template <class T, class LEAF, class BND, class SP>
template <class TRAVERSER>
void KdTree2<T, LEAF, BND, SP>::traverse(TRAVERSER& trav) {
    if (!trav.validBounds(m_bounds)) { return; }

    if (m_isLeaf) {
        for (iterator iter = m_data.leaf.begin;
             trav.unfinished() && iter != m_data.leaf.end;
             ++iter) {
            trav.leaf((*iter));
        }
    } else {
        if (trav.unfinished()) { m_data.branch.left->traverse(trav); }
        if (trav.unfinished()) { m_data.branch.right->traverse(trav); }
    }
}

}  // End namespace SpatialSubd
}  // End namespace EmldCore

#endif
