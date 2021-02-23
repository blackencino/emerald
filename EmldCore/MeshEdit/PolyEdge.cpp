//-*****************************************************************************
// Copyright (c) 2002 Tweak Inc. 
// All rights reserved.
//-*****************************************************************************

//-*****************************************************************************
// This class is a port of the TwkGeom::PolyEdge class written by
// Jim Hourihan. It has been changed to use Imath vec types instead of TwkMath.
//-*****************************************************************************

#include "PolyEdge.h"

namespace EmldCore {
namespace MeshEdit {

//-*****************************************************************************
// PROPERTY
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::Property::Property(PolyEdge *pe)
  : _mark(false)
{
    _index = pe->_properties.size();
    pe->_properties.push_back(this);
}

//-*****************************************************************************
PolyEdge::Property::~Property() {}

//-*****************************************************************************
void
PolyEdge::Property::print(std::ostream& o) const
{
    o << "(property " << this << ")";
}

//-*****************************************************************************
// VERTEX
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::PolyEdge::Vertex::Vertex(PolyEdge *pe, PolyEdge::Point p)
  : _point(p), _first(0), _userFlags(0), _generation(0)
{
    _index = pe->_vertices.size();
    pe->_vertices.push_back(this);
}

//-*****************************************************************************
PolyEdge::PolyEdge::Vertex::Vertex(PolyEdge *pe, PolyEdge::Point p,
                                   int index)
  : _point(p), _first(0), _userFlags(0), _index(index), _generation(0)
{
    pe->_vertices.push_back(this);
}

//-*****************************************************************************
PolyEdge::PolyEdge::Vertex::Vertex(PolyEdge *pe, PolyEdge::Point p,
                                   int index, int gen)
  : _point(p), _first(0), _userFlags(0), _index(index),
    _generation(gen)
{
    pe->_vertices.push_back(this);
}

//-*****************************************************************************
void PolyEdge::Vertex::detach(EdgePointer edge)
{
    if (_first == edge)
    {
        _first = edge->next(this);
    }
    else
    {
        for (EdgePointer e=_first; e; e = e->next(this))
        {
            if (e->_next_a == edge)
            {
                e->_next_a = edge->next(this);
                return;
            }

            if (e->_next_b == edge)
            {
                e->_next_b = edge->next(this);
                return;
            }
        }
    }
}

//-*****************************************************************************
bool PolyEdge::Vertex::isBorder()
{
    for (EdgeIterator i(this); i; i++) 
        if (i->isBorder()) return true;
    return false;
}

//-*****************************************************************************
bool PolyEdge::Vertex::isBridge()
{
    if (isNaked()) return false;
    int numPolys = 0;
    int numBorder = 0;

    for (EdgeIterator i(this); i; i++)
    {
        if (i->isNaked()) continue;
        if (i->isBorder()) numBorder++;
    }

    if (numBorder > 2)
    {
        return true;
    }
    
    // Do the hard thing here
    return false;
}

//-*****************************************************************************
int
PolyEdge::Vertex::valence()
{
    int count=0;
    for (EdgeIterator i(this); i; i++)
        if (!i->isNaked()) count++;
    return count;
}

//-*****************************************************************************
float
PolyEdge::Vertex::gaussCurvature()
{
    float A = 0.0;
    float phi = 0.0;

    for (PolygonIterator i(this); i; i++)
    {
        V3f a = i->incomingEdge(this)->otherVertex(this)->point();
        V3f b = i->outgoingEdge(this)->otherVertex(this)->point();
        phi += angleBetween(a - point(), b - point());
        A += i->area();
    }

    if (isBorder())
    {
        return (3.1415927 - phi) / (A / 3.0);
    }
    else
    {
        return (6.2831853 - phi) / (A / 3.0);
    }
}

//-*****************************************************************************
float
PolyEdge::Vertex::meanCurvature()
{
    float e = 0.0;
    float A = 0.0;

    for (PolygonIterator i(this); i; i++) A += i->area();

    for (EdgeIterator i(this); i; i++)
    {
        VertexPointer o = i->otherVertex(this);
        V3f outV = o->point() - point();

        PolygonPointer a = Edge::PolygonIterator(i())();
        PolygonPointer b = i->otherPolygon(a);

        V3f Na = a->normal();
        V3f Nb = b->normal();
        float gamma = angleBetween(Na, Nb);
        V3f cn = normalized(cross(Na, Nb));
        bool convex = dot(cn, outV) > 0.0;

        if (!convex) gamma = -gamma;
        e += gamma;
    }

    return e / (A / 3.0);
}

//-*****************************************************************************
PolyEdge::EdgePointer 
PolyEdge::Vertex::sharedEdge(PolyEdge::VertexPointer other)
{
    for (EdgeIterator i(this); i; i++)
        if (i->otherVertex(this) == other) return i();
    return 0;
}

//-*****************************************************************************
PolyEdge::Vertex::PolygonIterator::PolygonIterator(VertexPointer v)
  : _index(0)
{
    for (EdgeIterator i(v); i; i++)
    {
        for (Edge::PolygonIterator ip(i()); ip; ip++)
        {
            PolygonPointer p = ip();
            bool found=false;
            for (int q=_array.size(); q--;)
            {
                if (_array[q] == p) 
                {
                    found = true;
                    break;
                }
            }
	    
            if (!found) _array.push_back(p);
        }
    }
}

//-*****************************************************************************
PolyEdge::Vertex::~Vertex()
{
    //assert(_first == 0);
}

//-*****************************************************************************
// VERTEX ORDERED EDGE ITERATOR
//-*****************************************************************************

//-*****************************************************************************
void
PolyEdge::Vertex::OrderedEdgeIterator::next()
{
    // Find the outgoing edge polygon
    for (Edge::PolygonIterator i(_edge); i; i++)
    {
        PolygonPointer p = i();
        if (p->incomingEdge(_vertex) == _edge)
        {
            _edge = p->outgoingEdge(_vertex);

            if (_edge == _vertex->firstEdge())
            {
                _edge = 0;
            }
            else
            {
                return;
            }
        }
    }

    _edge = 0;
}

//-*****************************************************************************
// LINK
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::Link::~Link()
{
    delete _next;
}

//-*****************************************************************************
// EDGE
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::Edge::Edge(PolyEdgePointer pe, VertexPointer a, VertexPointer b)
  : _a(a), _b(b), _first(0), _userFlags(0)
{
    assert(_a);
    assert(_b);
    _next_a = _a->_first; _a->_first = this;
    _next_b = _b->_first; _b->_first = this;
    pe->_edges.push_back(this);
}

//-*****************************************************************************
PolyEdge::Edge::~Edge()
{
    delete _first;
}

//-*****************************************************************************
PolyEdge::VertexPointer 
PolyEdge::Edge::sharedVertex(EdgePointer e)
{
    VertexPointer v = 0;
    if      (e->_a == _a) v = _a;
    else if (e->_a == _b) v = _b;
    else if (e->_b == _a) v = _a;
    else if (e->_b == _b) v = _b;
    return v;
}

//-*****************************************************************************
PolyEdge::PolygonPointer
PolyEdge::Edge::otherPolygon(PolygonPointer p)
{
    assert( isManifold() );
    LinkPointer l = linkOf(p);
    assert(l);
    PolygonPointer ret =  l->_next ? l->_next->_polygon : _first->_polygon;
    if ( ret == p )
    {
        return NULL;
    }
    else
    {
        return ret;
    }
}

//-*****************************************************************************
PolyEdge::LinkPointer
PolyEdge::Edge::linkOf(PolyEdge::PolygonPointer p) const
{
    for (PolyEdge::LinkPointer l = _first; l; l=l->_next)
        if (l->_polygon == p) return l;
    return 0;
}

//-*****************************************************************************
PolyEdge::EdgePointer 
PolyEdge::Edge::next(PolyEdge::PolygonPointer p)
{
    assert(linkOf(p));
    if (PolyEdge::LinkPointer l = linkOf(p))
        return l->_next_edge;
    return 0;
}

//-*****************************************************************************
PolyEdge::LinkPointer
PolyEdge::Edge::attachFront(PolyEdge::PolygonPointer p)
{
    assert(!linkOf(p));
    LinkPointer l = new Link(p,p->_first,_first);
    p->_first 	  = this;
    _first = l;
    return l;
}

//-*****************************************************************************
PolyEdge::LinkPointer
PolyEdge::Edge::attachAfter(PolyEdge::PolygonPointer p,
                            PolyEdge::LinkPointer a)
{
    assert(!linkOf(p));
    LinkPointer l = new Link(p,a->_next_edge,_first);
    a->_next_edge = this;
    _first = l;
    return l;
}

//-*****************************************************************************
void
PolyEdge::Edge::detach(PolygonPointer p)
{
    // NOTE: This function leaves the PolyEdge is a non-sane state. This
    // function is an interum function.
    LinkPointer l = linkOf(p);
    assert(l);

    // detach from polygon by removing link pointers
    if (p->_first == this)
    {
        p->_first = l->_next_edge;
    }
    else
    {
        LinkPointer lp = p->previous(this)->linkOf(p);
        lp->_next_edge = l->_next_edge;
    }

    // detach link from edge
    if (l == _first)
    {
        _first = l->_next;
    }
    else
    {
        for (LinkPointer q=_first; q; q=q->_next)
        {
            if (q->_next == l)
            {
                q->_next = l->_next;
                break;
            }
        }
    }

    // link is now isolated -- delete it.
    l->_next = 0;
    delete l;
}

//-*****************************************************************************
// POLYGON
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::Polygon::Polygon(PolyEdgePointer pe,
                           VertexPointer v1,
                           VertexPointer v2,
                           VertexPointer v3, ...)
  : _first(0), _userFlags(0)
{
    // Make an array out of the arguments and pass it on to the
    // init() function to do the work.
    VertexArray varray;
    PropertyArray parray;
    va_list ap;
    varray.push_back(v1);
    varray.push_back(v2);
    varray.push_back(v3);
    va_start(ap,v3);

    while (VertexPointer v = va_arg(ap,VertexPointer))
    {
        varray.push_back(v);
    }

    va_end(ap);
    init(pe,varray,parray);
}

//-*****************************************************************************
PolyEdge::Polygon::Polygon(PolyEdgePointer pe,
                           VertexPointer v1, PropertyPointer p1,
                           VertexPointer v2, PropertyPointer p2,
                           VertexPointer v3, PropertyPointer p3, ...)
  : _first(0), _userFlags(0)
{
    // Make an array out of the arguments and pass it on to the
    // init() function to do the work.
    VertexArray varray;
    PropertyArray parray;
    va_list ap;
    varray.push_back(v1); parray.push_back(p1);
    varray.push_back(v2); parray.push_back(p2);
    varray.push_back(v3); parray.push_back(p3);
    va_start(ap,p3);

    while (VertexPointer v = va_arg(ap,VertexPointer))
    {
        PropertyPointer p = va_arg(ap,PropertyPointer);
        varray.push_back(v);
        parray.push_back(p);
    }

    va_end(ap);
    init(pe,varray,parray);
}

//-*****************************************************************************
void
PolyEdge::Polygon::init(PolyEdgePointer pe, 
                        const VertexArrayReference varray,
                        const PropertyArrayReference parray)
{
    bool useParray = parray.size() != 0;
    _userFlags = 0;

    // Add the polygon to the list
    pe->_polygons.push_back(this);

    // Find the first edge or create it
    EdgePointer e = pe->uniqueEdge(varray[0],varray[1]);
    LinkPointer l = e->attachFront(this);
    if (useParray) l->_property = parray.front();

    // Loop over args for next N edges/vertices creating if necessary
    for (int i=1,s=varray.size(); i<s; i++)
    {
        VertexPointer va = varray[i];
        VertexPointer vb = varray[(i+1)%s];
        e = pe->uniqueEdge(va,vb); 
        l = e->attachAfter(this,l);
        if (useParray) l->_property = parray[i];
    }
}

//-*****************************************************************************
PolyEdge::Polygon::Polygon(PolyEdgePointer pe,
                           EdgePointer e1,
                           EdgePointer e2,
                           EdgePointer e3, ...)
  : _first(0), _userFlags(0)
{
    va_list ap;

    // Add the polygon
    pe->_polygons.push_back(this);

    // Add first 3 edges in order
    LinkPointer l;
    l = e1->attachFront(this);
    l = e2->attachAfter(this,l);
    l = e3->attachAfter(this,l);

    // Add remaining edge arguments in order
    va_start(ap,e3);
    while (EdgePointer e = va_arg(ap,EdgePointer))
        l = e->attachAfter(this,l);
    va_end(ap);
}

//-*****************************************************************************
size_t 
PolyEdge::Polygon::size()
{
    int count = 0;
    for (EdgePointer e = _first; e; e = e->next(this)) count++;
    return count;
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::Polygon::next(EdgePointer e)
{
    if (EdgePointer n = e->next(this)) return n;
    return _first;
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::Polygon::previous(EdgePointer e)
{
    if (e == _first)
    {
        EdgePointer q=_first;
        for (EdgePointer n; (n=q->next(this)); q=n);
        return q;
    }
    else
    {
        EdgePointer q,n;
        for (q=_first; (n=q->next(this)) != e; q=n);
        return q;
    }
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::Polygon::otherEdge(VertexPointer v, EdgePointer e)
{
    EdgePointer n = next(e);
    if (n->sharedVertex(e) != v) n = previous(e);
    return n;
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::Polygon::outgoingEdge(VertexPointer v)
{
    for (VertexIterator i(this); i; i++)
    {
        if (i() == v) return i.edge();
    }

    assert(false); // should not get here.
    return 0;
}

//-*****************************************************************************
PolyEdge::PropertyPointer
PolyEdge::Polygon::propertyOf(VertexPointer v)
{
    for (VertexIterator i(this); i; i++)
    {
        if (i() == v) return i.property();
    }

    assert(false); // should not get here.
    return 0;
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::Polygon::incomingEdge(VertexPointer v)
{
    // not terribly efficient, but compact.
    return previous(outgoingEdge(v));
}

//-*****************************************************************************
PolyEdge::Normal
PolyEdge::Polygon::normal()
{
    // This is using the generalized normal computation which works
    // best for 3, 5 or more vertices. For quads, it should use a
    // different algorithm, but what the fuck.
    VertexPointer a=0, b=0, c=0;
    V3f n(0.f,0.f,0.f);

    for (VertexIterator i(this); i; i++)
    {
        a = b;
        b = c;
        c = i();

        if (a && b)
        {
            n += cross(b->point() - a->point(), c->point() - a->point());
        }
    }

    return n.normalized();
}

//-*****************************************************************************
float
PolyEdge::Polygon::area()
{
    // Mostly like the normal computation.
    VertexPointer a=0, b=0, c=0;
    V3f n(0.f,0.f,0.f);

    for (VertexIterator i(this); i; i++)
    {
        a = b;
        b = c;
        c = i();

        if (a && b)
        {
            n += cross(b->point() - a->point(), c->point() - a->point());
        }
    }

    return length(n) / 2.0f;
}

//-*****************************************************************************
bool
PolyEdge::Polygon::isConvex()
{
    // Mostly like the normal computation.
    VertexPointer a=0, b=0, c=0;
    V3f n(0.f,0.f,0.f);

    for (VertexIterator i(this); i; i++)
    {
        a = b;
        b = c;
        c = i();

        if (a && b)
        {
            V3f x = cross(b->point() - a->point(), c->point() - a->point());
            if (length(n) != 0.f && dot(x.normalized(), n) < 0)
                return false;
            n += x;
            n.normalize();
        }
    }

    return true;
}

//-*****************************************************************************
PolyEdge::Point
PolyEdge::Polygon::centroid()
{
    // This is using the generalized normal computation which works
    // best for 3, 5 or more vertices. For quads, it should use a
    // different algorithm, but what the fuck.
    Point c(0.f,0.f,0.f);
    int count=0;

    for (VertexIterator i(this); i; i++)
    {
        count++;
        c += i()->point();
    }

    return c / float(count);
}

//-*****************************************************************************
// BORDER VERTEX ITERATOR
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::BorderVertexIterator::BorderVertexIterator(VertexPointer v)
  : _v(v), _e(0), _e0(0)
{
    for (Vertex::EdgeIterator i(v); i; i++)
    {
        EdgePointer e = i();

        if (e->isBorder())
        {
            Edge::PolygonIterator ip(e);
            if (ip->outgoingEdge(v) == e)
            {
                _e0 = e;
                _e  = e;
                return;
            }
        }
    }
}

//-*****************************************************************************
void
PolyEdge::BorderVertexIterator::next()
{
    if (!_e) return;
    _v = _e->otherVertex(_v);
    
    for (Vertex::EdgeIterator i(_v); i; i++)
    {
        EdgePointer e = i();
        if (e == _e || e->isNaked() || !e->isBorder()) continue;
        _e = e == _e0 ? 0 : e;
        return;
    }
}

//-*****************************************************************************
// POLY EDGE
//-*****************************************************************************

//-*****************************************************************************
PolyEdge::PolyEdge()
{
}

//-*****************************************************************************
PolyEdge::~PolyEdge()
{
    // Just delete everything. This is catastrophic (obviously), but
    // since everything is deleted, no pointers must be munged.
    for (int i=0; i<_vertices.size(); i++) delete _vertices[i];
    for (int i=0; i<_edges.size(); i++) delete _edges[i];
    for (int i=0; i<_polygons.size(); i++) delete _polygons[i];
    for (int i=0; i<_properties.size(); i++) delete _properties[i];
}

//-*****************************************************************************
void
PolyEdge::reindexVertices()
{
    for (size_t i=0; i<_vertices.size(); i++)
    {
        _vertices[i]->_index = i;
    }
}

//-*****************************************************************************
void
PolyEdge::reindexProperties()
{
    for (size_t i=0; i<_properties.size(); i++)
    {
        _properties[i]->_index = i;
    }
}

//-*****************************************************************************
void
PolyEdge::clearVertexFlags(PolyEdge::Flags flags)
{
    flags = ~flags;

    for (int i=0,s=_vertices.size(); i<s; i++)
    {
        _vertices[i]->_userFlags &= flags;
    }
}

//-*****************************************************************************
void
PolyEdge::clearEdgeFlags(PolyEdge::Flags flags)
{
    flags = ~flags;

    for (int i=0,s=_edges.size(); i<s; i++)
    {
        _edges[i]->_userFlags &= flags;
    }
}

//-*****************************************************************************
void
PolyEdge::clearPolygonFlags(PolyEdge::Flags flags)
{
    flags = ~flags;

    for (int i=0,s=_polygons.size(); i<s; i++)
    {
        _polygons[i]->_userFlags &= flags;
    }
}

//-*****************************************************************************
int
PolyEdge::deleteNakedVertices()
{
    int count=0;

    for (int i=0; i<_vertices.size(); i++)
    {
        if (!_vertices[i]->_first)
        {
            delete _vertices[i];
            _vertices[i] = _vertices.back();
            _vertices.pop_back();
            count++;
            i--;
        }
    }

    return count;
}

//-*****************************************************************************
int
PolyEdge::deleteNakedEdges()
{
    int count = 0;

    for (int i=0; i<_edges.size(); i++)
    {
        EdgePointer e = _edges[i];

        if (!e->_first)
        {
            e->a()->detach(e);
            e->b()->detach(e);
            delete e;
            _edges[i] = _edges.back();
            _edges.pop_back();
            i--;
            count++;
        }
    }
    
    return count;
}

//-*****************************************************************************
int
PolyEdge::deleteDeadPolygons()
{
    int count=0;

    for (int i=0; i<_polygons.size(); i++)
    {
        if (!_polygons[i]->_first)
        {
            delete _polygons[i];
            count++;
            _polygons[i] = _polygons.back();
            _polygons.pop_back();
            i--;
        }
    }
    
    return count;
}

//-*****************************************************************************
int
PolyEdge::deleteUnusedProperties()
{
    int count = 0;
    if (!_properties.size()) return 0;

    for (int i=0; i<_properties.size(); i++)
    {
        _properties[i]->_mark = false;
    }

    for (int i=0; i<_polygons.size(); i++)
    {
        for (Polygon::VertexIterator vi(_polygons[i]); vi; vi++)
        {
            if (vi.property()) vi.property()->_mark = true;
        }
    }

    for (int i=0; i<_properties.size(); i++)
    {
        if (!_properties[i] || _properties[i]->_mark == false)
        {
            count++;
            delete _properties[i];
            _properties[i] = _properties.back();
            _properties.pop_back();
            i--;
        }
    }

    return count;
}

//-*****************************************************************************
void
PolyEdge::garbageCollect()
{
    deleteDeadPolygons();
    deleteNakedEdges();
    deleteNakedVertices();
    deleteUnusedProperties();
}

//-*****************************************************************************
// ATOMIC OPERATIONS
//-*****************************************************************************

//-*****************************************************************************
void
PolyEdge::killPolygon(PolygonPointer p)
{
    // Remove all the edges
    while (p->_first)
    {
        p->_first->detach(p);
    }
}

//-*****************************************************************************
void
PolyEdge::killEdge(EdgePointer e)
{
    // kills polygons and makes it naked -- its not really deleted
    // until deleteNakedEdges() is run.
    while (e->_first)
    {
        killPolygon(e->_first->_polygon);
    }
}

//-*****************************************************************************
void
PolyEdge::replace(PolygonPointer p, 
                  VertexPointer vold, 
                  VertexPointer vnew,
                  PropertyPointer pNew)
{
    EdgePointer     en = p->outgoingEdge(vold);
    EdgePointer     ep = p->incomingEdge(vold);
    VertexPointer   vn = en->otherVertex(vold);
    VertexPointer   vp = ep->otherVertex(vold);

    PropertyPointer pp = ep->propertyOf(p);
    PropertyPointer pn = pNew;
    if (pp && !pn)  pn = en->propertyOf(p);

    EdgePointer enNew = uniqueEdge(vnew,vn);
    EdgePointer epNew = uniqueEdge(vnew,vp);
    
    enNew->attachAfter(p,en->linkOf(p));
    en->detach(p);
    
    epNew->attachAfter(p,ep->linkOf(p));
    ep->detach(p);

    epNew->propertyOf(p) = pp;
    enNew->propertyOf(p) = pn;
}

//-*****************************************************************************
void
PolyEdge::remove(PolygonPointer p, VertexPointer v)
{
    EdgePointer     en = p->outgoingEdge(v);
    EdgePointer     ep = p->incomingEdge(v);
    VertexPointer   vn = en->otherVertex(v);
    VertexPointer   vp = ep->otherVertex(v);

    PropertyPointer pp = ep->propertyOf(p);

    EdgePointer eNew = uniqueEdge(vn,vp);
    eNew->attachAfter(p,en->linkOf(p));
    ep->detach(p);
    en->detach(p);
    eNew->propertyOf(p) = pp;
}

//-*****************************************************************************
void
PolyEdge::replaceVertex(PolygonPointer p, 
                        VertexPointer v, 
                        EdgePointer e,
                        VertexPointer nv)
{
    // There has got to be a better way to do this
    EdgePointer o = p->otherEdge(v,e);
    PolygonPointer np;

    do
    {
        np = 0;
        for (Edge::PolygonIterator i(o); i; i++)
        {
            if (i() == p) continue;
            np = i();
            break;
        }

        if (np) replaceVertex(np,v,o,nv);
    } 
    while (np);

    replace(p,v,nv);
}

//-*****************************************************************************
void
PolyEdge::duplicateProperties(VertexPointer v)
{
    for (Vertex::PolygonIterator i(v); i; i++)
    {
        PolygonPointer pp = i();
        EdgePointer e     = pp->outgoingEdge(v);

        if (PropertyPointer p = e->propertyOf(pp))
        {
            e->propertyOf(pp)  = p->copy(this);
        }
    }
}

//-*****************************************************************************
PolyEdge::VertexPointer
PolyEdge::split(VertexPointer v, EdgePointer e)
{
    assert(v->isBorder());

    PolygonPointer skip = Edge::PolygonIterator(e)();
    VertexPointer n = 0;

    for (Edge::PolygonIterator i(e); i;)
    {
        PolygonPointer p = i();

        if (p == skip) 
        {
            i++;
        }
        else
        {
            n = duplicate(v);
            replaceVertex(p,v,e,n);
            duplicateProperties(n);
            i.reset();
        }
    }

    return n;
}

//-*****************************************************************************
void
PolyEdge::splitBridge(VertexPointer v)
{
    if (!v->isBridge()) return;

    for (Vertex::EdgeIterator i(v); i; i++)
    {
        if (i->isBorder())
        {
            Edge::PolygonIterator ip(i());
            VertexPointer n = duplicate(v);
            replaceVertex(ip(),v,i(),n);
            duplicateProperties(n);
            splitBridge(v);
            return;
        }
    }
}

//-*****************************************************************************
void
PolyEdge::insert(PolygonPointer p, 
                 VertexPointer vNew, 
                 VertexPointer vAfter,
                 PropertyPointer pNew)
{
    EdgePointer oe  = p->outgoingEdge(vAfter);
    EdgePointer e0  = uniqueEdge(vAfter,vNew);
    EdgePointer e1  = uniqueEdge(vNew,oe->otherVertex(vAfter));

    PropertyPointer op = oe->propertyOf(p);
    if (!pNew) pNew = op;

    LinkPointer l = e0->attachAfter(p,oe->linkOf(p));
    e1->attachAfter(p,l);
    oe->detach(p);

    e0->propertyOf(p) = op;
    e1->propertyOf(p) = pNew;
}

//-*****************************************************************************
void
PolyEdge::insert(PolygonPointer p, 
                 VertexPointer vNew, 
                 EdgePointer e,
                 PropertyPointer pNew)
{
    insert(p,vNew, p->outgoingEdge(e->a()) == e ? e->a() : e->b(), pNew);
}

//-*****************************************************************************
PolyEdge::VertexPointer
PolyEdge::split(EdgePointer e, float u)
{
    V3f p = e->a()->point() * (1.0f - u) + e->b()->point() * u;
    VertexPointer v = new Vertex(this,p);

    for (Edge::PolygonIterator i(e); i; i.reset())
    {
        PolygonPointer p = i();
        PropertyPointer pn = 0;

        if (e->propertyOf(p))
        {
            PropertyPointer pa = p->propertyOf(e->a());
            PropertyPointer pb = p->propertyOf(e->b());
            pn = pa->copy(this);
            pn->interpolate(pa, pb, u);
        }

        insert(p,v,e,pn);
    }

    return v;
}

//-*****************************************************************************
PolyEdge::EdgePointer
PolyEdge::split(PolygonPointer p, VertexPointer a, VertexPointer b)
{
    if (EdgePointer e = a->sharedEdge(b))
    {
        for (Edge::PolygonIterator i(e); i; i++) 
        {
            // This means that the vertices are already attached to
            // the polygon via a common edge. So the polygon can't be
            // split
            PolygonPointer pp = i();
            assert(pp != p);
        }
    }

    PropertyPointer pa = p->propertyOf(a);

    EdgePointer re = uniqueEdge(a,b);
    VertexArray varray;
    PropertyArray parray;
    varray.push_back(a);
    if (pa) parray.push_back(pa);
    VertexPointer v = a;

    for (EdgePointer e = p->outgoingEdge(v); v != b; e = p->next(e))
    {
        v = e->otherVertex(v);
        assert(v != a);
        varray.push_back(v);
        if (pa) parray.push_back(p->propertyOf(v));
    }

    PolygonPointer np = new Polygon(this, varray, parray);

    for (int i=1; i<varray.size()-1; i++)
    {
        remove(p,varray[i]);
    }

    return re;
}

//-*****************************************************************************
// GLOBALS
//-*****************************************************************************

//-*****************************************************************************
std::ostream&
operator<<(std::ostream &o, PolyEdge::Vertex& v)
{
    o << v.index();
    if (v.generation() > 0) o << ":" << v.generation();
    return o;
}

//-*****************************************************************************
std::ostream&
operator<<(std::ostream &o, PolyEdge::Edge& e)
{
    PolyEdge::VertexPointer a = e.a();
    PolyEdge::VertexPointer b = e.b();
    o << "[" << *a << " " << *b << "]";
    return o;
}

//-*****************************************************************************
std::ostream&
operator<<(std::ostream &o, PolyEdge::Polygon& p)
{
    o << "(";
    int c=0;

    for (PolyEdge::Polygon::VertexIterator i(&p); i; i++)
    {
        if (c++) o << " ";
        PolyEdge::VertexPointer v = i();
        o << *v;
    }

    o << ")";
    return o;
}

//-*****************************************************************************
std::ostream&
operator<<(std::ostream &o, PolyEdge::Property& p)
{
    p.print(o);
    return o;
}

//-*****************************************************************************
void
PolyEdge::sanityCheck(PolyEdge::PolygonPointer p)
{
    VertexArray varray;

    for (Polygon::VertexIterator i(p); i; i++)
    {
        VertexPointer v = i();
        VertexArray::iterator it = find(varray.begin(), varray.end(), v);
        if ( it != varray.end() )
        {
            std::cerr << "Assertion Failed: polygon doubles back on itself.\n";
            std::cerr << "    partial list: (";
            for (int q=0; q<varray.size(); q++)
            {
                std::cerr << *varray[q] << " ";
            }
            std::cerr << *(*it) << " ...)" << std::endl << std::flush;
            abort();
        }
        varray.push_back(v);
    }
}

} // End namespace MeshEdit
} // End namespace EmldCore

