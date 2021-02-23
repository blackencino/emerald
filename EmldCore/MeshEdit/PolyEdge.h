//-*****************************************************************************
// Copyright (c) 2002 Tweak Inc.
// All rights reserved.
//-*****************************************************************************

//-*****************************************************************************
// This class is a port of the TwkGeom::PolyEdge class written by
// Jim Hourihan. It has been changed to throw exceptions instead of
// aborting on error, and to use Imath vec types instead of TwkMath.
//-*****************************************************************************

#ifndef _EmldCore_MeshEdit_PolyEdge_h_
#define _EmldCore_MeshEdit_PolyEdge_h_

#include "Foundation.h"

namespace EmldCore {
namespace MeshEdit {

//-*****************************************************************************
//  class PolyEdge
//
//  A Quad-Edge like structure. You can iterator over Vertices, Edges,
//  or Polygons, from any of the the other structures -- e.g. you can
//  do this:
//
//	VertexPointer v;
//
//	for (Vertex::PolygonIterator i(v); i; i++)
//	{
//	    PolygonPointer p = i();
//	    i->doSomething();
//	}
//
//  or this:
//
//	for (Polygon::VertexIterator i(p); i; i++)
//	{
//	    VertexPointer v = i();
//	    i->doSomething();
//	}
//
//  Some operations are "atomic" in that they are minimal changes to the
//  structure while keeping the configuration "sane". You can build up more
//  complex functions by calling these operations. Functions which make
//  edits to the structure and leave it in a non-sane state are not
//  accessible from the outside.
//-*****************************************************************************
class PolyEdge
{
public:
    class Vertex;
    class Edge;
    class Polygon;
    class Property;
    typedef V3f                         Point;
    typedef V3f                         Normal;
    typedef Point&	    		PointReference;
    typedef Vertex*	    		VertexPointer;
    typedef Edge*	    		EdgePointer;
    typedef Polygon*	    		PolygonPointer;
    typedef Property*	    		PropertyPointer;
    typedef unsigned int    		Flags;
    typedef PolyEdge*	    		PolyEdgePointer;
    typedef std::vector<VertexPointer> 	VertexArray;
    typedef std::vector<EdgePointer> 	EdgeArray;
    typedef std::vector<PolygonPointer> PolygonArray;
    typedef std::vector<PropertyPointer> PropertyArray;
    typedef VertexArray& 		VertexArrayReference;
    typedef EdgeArray& 			EdgeArrayReference;
    typedef PolygonArray& 		PolygonArrayReference;
    typedef PropertyArray& 		PropertyArrayReference;
    typedef std::vector<float>		WeightArray;

private:


    //-*************************************************************************
    // class PolyEdge::Edge::Link
    //
    // This is a link structure between the edge and each of its
    // polygons.  Consequently, it acts as a polygon vertex as
    // well. For this reason, it also holds per vertex per polygon
    // properties.
    //-*****************************************************************************
    struct Link
    {
	EMLD_UTIL_MEM_POOL_NEW_AND_DELETE( Link );

	Link( PolygonPointer p, EdgePointer e, Link *n )
          : _polygon(p), _property(0), _next_edge(e), _next(n) {}
	~Link();
	PolygonPointer  _polygon;
	PropertyPointer	_property;
	EdgePointer	_next_edge;
	Link* 	    	_next;
    };

    typedef Link* LinkPointer;
    friend class PolyEdge::Vertex;
    friend class PolyEdge::Edge;
    friend class PolyEdge::Polygon;

public:

    //-*************************************************************************
    // Property
    // --------
    //
    // This class holds arbitrary data that is interpolated by the atomic
    // operations. The idea is that you make your own sub-class of this
    // class and attach instances to the surface. At the point, any
    // operations done on the surface will automatically handle the
    // property interpolation.
    //
    // You need to supply the copy and interpolate functions. Interpolate
    // should set the values of the property based on the input
    // properties. Note that one of the arguments could be the property
    // itself. There are two interpolate functions because the first
    // (interpolation between two points), is used so frequently that an
    // optimized version is warranted. The second version is the more
    // general interpolation function which requires the class to compute
    // a "weighted sum" or something equivalent.
    //-*************************************************************************
    class Property
    {
    public:
	Property(PolyEdge*);
	virtual ~Property();
	virtual Property*   copy(PolyEdge*) const = 0;
	virtual void	    interpolate(Property *a, Property *b, float u) = 0;
	virtual void	    interpolate(const PolyEdge::PropertyArray&,
					const WeightArray& weights) = 0;
	virtual bool	    equals(const Property*) const = 0;
	virtual void	    print(std::ostream&) const;

	size_t		    index() const { return _index; }

    private:
	bool		    _mark  : 1;
	size_t		    _index : 31;
	friend class PolyEdge;
    };

    //-*************************************************************************
    // template class PropertyVector<> is a very simple
    // implementation of Property that holds a small fixed size array
    // of some type. The 3rd template parameter Ni, indicates that
    // the first Ni elements should be normalized. The default is 0.
    //-*************************************************************************
    template<typename T, size_t N, size_t Ni=0>
    class PropertyVector : public PolyEdge::Property
    {
    public:
	typedef PropertyVector<T,N> ThisType;
	PropertyVector(PolyEdge *p) : PolyEdge::Property(p) {}
	virtual ~PropertyVector();

	T data[N];

	virtual PolyEdge::Property* copy(PolyEdge*) const;
	virtual void	    interpolate(PolyEdge::Property*,
					PolyEdge::Property*,
					float);
	virtual void	    interpolate(const PolyEdge::PropertyArray&,
					const WeightArray& weights);
	virtual bool	    equals(const PolyEdge::Property*) const;
	virtual void	    print(std::ostream&) const;
    };

    typedef PropertyVector<float,2> STProperty;
    typedef PropertyVector<float,3,3> NormalProperty;
    typedef PropertyVector<float,5,3> NormalSTProperty;

    //-*************************************************************************
    // PolyEdge::Vertex
    // ----------------
    //
    // This class is a brick.
    //-*************************************************************************
    class Vertex
    {
    public:
	// Constructor: A vertex is created in the context of a
	// PolyEdge structure and with a point. The index is
	// automatically given if not passed into the constructor.
	// Its ok to have duplicate indices (you can use these to
	// keep track of original vertex indices for example).
	EMLD_UTIL_MEM_POOL_NEW_AND_DELETE( Vertex );
	Vertex(PolyEdgePointer,Point);
	Vertex(PolyEdgePointer,Point,int index);

        //-*********************************************************************
	// class PolyEdge::Vertex::EdgeIterator
	//
	// Iterate over edges connected to a vertex
        //-*********************************************************************
	class EdgeIterator
	{
	public:
	    // Constructor:
	    // A pointer to a vertex whose edges will be iteratored
	    // over.
	    EdgeIterator(VertexPointer v);

	    // operator++(), operator++(int):
	    // Next edge on vertex. Returns void.
	    // reset sets the iterator back to the beginning.
	    void operator++();
	    void operator++(int);
	    void reset();

	    // operator->():
	    // Dereference the iterator, returns pointer to
	    // referenced edge.
	    EdgePointer operator->() const { return _edge; }
            EdgePointer operator*() const { return _edge; }
	    EdgePointer operator()() const { return _edge; }

	    // operator bool():
	    // For use in if and for statements.
	    operator bool () const { return _edge?true:false; }

	private:
	    VertexPointer   _vertex;
	    EdgePointer	    _edge;
	};

        //-*********************************************************************
	// OrderedEdgeIterator
	//
	// Just like edge iterator, but in order around the
	// vertex. The order is determined by the surface
	// orientation. The vertex must be manifold and not a
	// boundary vertex for this to work
        //-*********************************************************************
	class OrderedEdgeIterator
	{
	public:
	    // Constructor:
	    // A pointer to a vertex whose edges will be iteratored
	    // over.
	    OrderedEdgeIterator(VertexPointer v);

	    // operator++(), operator++(int):
	    // Next edge on vertex. Returns void.
	    // reset sets the iterator back to the beginning.
	    void next();
	    void operator++() { next(); }
	    void operator++(int) { next(); }
	    void reset();

	    // operator->():
	    // Dereference the iterator, returns pointer to
	    // referenced edge.
	    EdgePointer operator->() const { return _edge; }
            EdgePointer operator*() const { return _edge; }
	    EdgePointer operator()() const { return _edge; }

	    // operator bool():
	    // For use in if and for statements.
	    operator bool () const { return _edge?true:false; }

	private:
	    VertexPointer   _vertex;
	    EdgePointer	    _edge;
	};

        //-*********************************************************************
	// Polygon iterator
	//
	// This iterator caches all the polygons assocated with the
	// vertex upon construction so if you delete one it will
	// still be iterated over.
        //-*********************************************************************
	class PolygonIterator
	{
	public:
	    PolygonIterator(VertexPointer v);

	    void operator++() { _index++; }
	    void operator++(int) { _index++; }

	    PolygonPointer operator->() const { return _array[_index]; }
            PolygonPointer operator*() const { return _array[_index]; }
	    PolygonPointer operator()() const { return _array[_index]; }

	    operator bool () const { return _index < _array.size(); }

	private:
	    size_t	    _index;
	    PolygonArray    _array;
	};

	// point():
	// Return a reference to the location of the vertex.
	Point point() const { return _point; }
	PointReference point() { return _point; }

	// flag(): Set or get the status of a flag. Note that you
	// can't get a reference to the flags for a Vertex since its
	// a bit field.
	bool flag(Flags f) const { return _userFlags & f; }
	void flag(Flags f, bool s)
        { if (s) _userFlags |= f; else _userFlags = _userFlags & ~f; }
	Flags flags() const { return _userFlags; }

	// isnaked():
	// Returns true if vertex is not connected to any edges.
	bool isNaked() const { return _first ? false : true; }

	// isborder():
	// Returns true if all connected edges are border edges.
	bool isBorder();

	// isBridge(): returns true if the vertex is a single point
	// that joins two or more contiguous surfaces that do not
	// share an edge.
	bool isBridge();

	// index()
	// Returns the vertex number
	int index() const { return _index; }
	int& index() { return _index; }

	// This is useful only for debugging.
	unsigned int generation() const { return _generation; }

	// sharedEdge():
	// Returns edge if there is one between this vertex and another
	EdgePointer sharedEdge(VertexPointer other);

	// First edge connected to this vertex
	EdgePointer firstEdge() const { return _first; }

	// Valence (number of edges shared by vertex) -- ignores
	// naked edges.
	int valence();

        // Gaussian Curvature at vertex
        float gaussCurvature();

        // Mean curvature at vertex
        float meanCurvature();

    private:

	// This constructor is used by duplicate
	Vertex(PolyEdgePointer,Point,int index,int duplicate);

	// Destructor:
	// To destroy a vertex you must use the atomic delete
	// operation on vertices.
	~Vertex();

	friend class PolyEdge;
	friend class Edge;

	void detach(EdgePointer);

    private:
	Point 	    _point;
	EdgePointer _first;
	int	    _index;
	Flags 	    _userFlags  : 20;
	Flags	    _generation	: 12;
    };

    //-*************************************************************************
    // class PolyEdge::Edge
    // --------------------
    //
    // This class holds most of the state and does most of the work.
    //-*************************************************************************
    class Edge
    {
    public:
	class PolygonIterator;
	friend class PolygonIterator;

	//-*********************************************************************
	// class PolyEdge::Edge::PolygonIterator
	//
	// Iterate over polygons connected to an edge.
	//-*********************************************************************
	class PolygonIterator
	{
	public:
	    PolygonIterator(EdgePointer e) : _e(e), _link(e->_first) {}

	    // operator++(), operator++(int):
	    // Next polygon on edge. Returns void.
	    void operator++() { if (_link) _link=_link->_next; }
	    void operator++(int) { if (_link) _link=_link->_next; }

	    void reset() { _link = _e->_first; }

	    // operator->():
	    // Dereference the iterator, returns pointer to
	    // referenced Polygon.
	    PolygonPointer operator->() const
            { return _link?_link->_polygon:0; }
            PolygonPointer operator*() const
            { return _link?_link->_polygon:0; }
	    PolygonPointer operator()() const
            { return _link?_link->_polygon:0; }

	    // operator bool():
	    // For use in if and for statements.
	    operator bool () const
            { return _link?(bool)_link->_polygon:false; }

	private:
	    EdgePointer	    _e;
	    LinkPointer	    _link;
	};

	// Constructor:
	// An edge is created in the context of a PolyEdge and by
	// supplying two vertices.
	EMLD_UTIL_MEM_POOL_NEW_AND_DELETE( Edge );
	Edge(PolyEdgePointer,VertexPointer,VertexPointer);

	// a(), b():
	// Access to the edge vertices.
	VertexPointer a() const { return _a; }
	VertexPointer b() const { return _b; }

	// sharedVertex():
	// Return the vertex common to both edges (if there is one)
	VertexPointer sharedVertex(EdgePointer e);

	// otherVertex():
	// Return the "other" vertex for this edge.
	VertexPointer otherVertex(VertexPointer v) const
        { return v==_a ? _b : _a; }

	// otherPolygon():
	// Return the "other" polygon attached to this edge. Note that
	// this only works like otherVertex() only in the case of a
	// manifold edge -- (an edge with one or two polygons attached).
	PolygonPointer otherPolygon(PolygonPointer);

	// flag():
	// Set or get the status of a flag.
	bool flag(Flags f) const { return _userFlags & f; }
	void flag(Flags f, bool s)
        { if (s) _userFlags |= f; else _userFlags = _userFlags & ~f; }
	Flags& flags() { return _userFlags; }
	Flags flags() const { return _userFlags; }

	// isnaked():
	// Returns true if edge is not connected to any polygons.
	bool isNaked() const { return _first ? false : true; }

	// isborder():
	// Returns true if edge is connected to only a single
	// polygon. Will return false on a naked edge.
	bool isBorder() const
        { return _first ? _first->_next==0 : false; }

	// isManifold(): returns true if the edge has 1 or 2 polygons
	// attached to it. (it should return true if the polygons are
	// similarily oriented as well.)
	bool isManifold()
        {return _first ? (_first->_next?!_first->_next->_next:true):false;}

	// Return the next edge associated with the polygon
	EdgePointer next(PolygonPointer);

	// Returns the next edge associated with the vertex
	EdgePointer next(const VertexPointer v) const
        { return v==_a ? _next_a : _next_b; }

	// length of edge
	float length() const
        { return (a()->point() - b()->point()).length(); }

	// property for polygon
	PropertyPointer& propertyOf(PolygonPointer p)
        { return linkOf(p)->_property; }

    private:
	~Edge();

	LinkPointer linkOf(PolygonPointer) const;
	LinkPointer attachFront(PolygonPointer);
	LinkPointer attachAfter(PolygonPointer,LinkPointer);

	// Non-sane result guaranteed -- removes this edge from the polygon,
	// but does not repair the damage.
	void detach(PolygonPointer);

	friend class Polygon;
	friend class Vertex;
	friend class PolyEdge;

    private:
	VertexPointer 	_a;
	VertexPointer 	_b;
	EdgePointer 	_next_a;
	EdgePointer 	_next_b;
	LinkPointer 	_first;
	Flags 		_userFlags;
    };

    //-*************************************************************************
    // class PolyEdge::Polygon
    // -----------------------
    //
    // This is a lightweight class which represents a polygon.
    //-*************************************************************************
    class Polygon
    {
    public:
	// Constructor:
	// A polygon is created by a number of vertices or edges.
	Polygon(PolyEdgePointer,
		VertexPointer,
		VertexPointer,
		VertexPointer, ...);

	Polygon(PolyEdgePointer,
		VertexPointer, PropertyPointer,
		VertexPointer, PropertyPointer,
		VertexPointer, PropertyPointer, ...);

	Polygon(PolyEdgePointer,
		EdgePointer,
		EdgePointer,
		EdgePointer, ...);

	Polygon(PolyEdgePointer pe,
		VertexArrayReference a,
		PropertyArrayReference pa)
          : _first(0), _userFlags(0) {init(pe,a,pa);}

	EMLD_UTIL_MEM_POOL_NEW_AND_DELETE( Polygon );

	class EdgeIterator;
	class VertexIterator;
	friend class EdgeIterator;
	friend class VertexIterator;

	// The first edge of this polygons
	EdgePointer firstEdge() const { return _first; }

	// Other edge attached to the given vertex.
	EdgePointer otherEdge(VertexPointer,EdgePointer);

	// These are circular -- they never produce a 0
	EdgePointer next(EdgePointer);
	EdgePointer previous(EdgePointer);

	// Edges with respect to vertices
	EdgePointer outgoingEdge(VertexPointer);
	EdgePointer incomingEdge(VertexPointer);

	// Properties for vertices
	PropertyPointer propertyOf(VertexPointer v);

        // Convex test
        bool isConvex();

	// Normal is length 1
	Normal normal();

	// Area computed using cross products.
	float area();

	// Centroid
	Point centroid();

	// Alive
	bool alive() { return _first ? true : false; }

	// Number of points
	size_t size();

	// flag():
	// Set or get the status of a flag.
	bool flag(Flags f) const { return _userFlags & f; }
	void flag(Flags f, bool s)
        { if (s) _userFlags |= f; else _userFlags = _userFlags & ~f; }
	Flags& flags() { return _userFlags; }
	Flags flags() const { return _userFlags; }

        //-*********************************************************************
	// class PolyEdge::Polygon::EdgeIterator
	//
	// Iterate over edges connected to a polgyon.
	//-*********************************************************************
	class EdgeIterator
	{
	public:
	    EdgeIterator(PolygonPointer p) : _p(p), _e(p->_first)  {}

	    // operator++(), operator++(int):
	    // Next edge on polygon. Returns void.
	    void operator++() { if (_e) _e=_e->next(_p); }
	    void operator++(int) { if (_e) _e=_e->next(_p); }

	    // operator->():
	    // Dereference the iterator, returns pointer to
	    // referenced Edge.
	    EdgePointer operator->() const { return _e; }
            EdgePointer operator*() const { return _e; }
	    EdgePointer operator()() const { return _e; }

	    operator bool () const { return _e; }

	    PolygonPointer polygon() const { return _p; }

	private:
	    PolygonPointer  _p;
	    EdgePointer     _e;
	};

        //-*********************************************************************
	// class PolyEdge::Polygon::VertexIterator
	//
	// Iterate over vertices connected to a polgyon.
	//-*********************************************************************
	class VertexIterator
	{
	public:
	    VertexIterator(PolygonPointer p);

	    // Associated edge
	    EdgePointer edge() const { return _i(); }

	    // Associated property
	    PropertyPointer property() const
            { return edge()->propertyOf(_i.polygon()); }

	    // operator++(), operator++(int):
	    // Next Verteex in Polygon. Returns void.
	    void operator++() { _v = _i->otherVertex(_v); _i++; }
	    void operator++(int) { _v = _i->otherVertex(_v); _i++; }

	    // operator->():
	    // Dereference the iterator, returns pointer to
	    // referenced Vertex.
	    VertexPointer operator->() const { return _v; }
            VertexPointer operator*() const { return _v; }
	    VertexPointer operator()() const { return _v; }

	    operator bool () const { return bool(_i); }

	private:
	    EdgeIterator    _i;
	    VertexPointer   _v;
	};

	friend class Edge;
	friend class PolyEdge;

    private:
	void init(PolyEdgePointer,
		  VertexArrayReference,
		  PropertyArrayReference);

    private:
	EdgePointer _first;
	Flags 	    _userFlags;
    };

    //-*************************************************************************
    // Border Vertex Iterator.
    // -----------------------
    //
    // This thing will follow a border loop. The order is determinded by
    // the polygons which are attached to border. The edges and vertices
    // are in an order that would create a polygon to fill the border.
    //-*************************************************************************
    class BorderVertexIterator
    {
    public:
	BorderVertexIterator(VertexPointer v);

	void operator++()	{ next(); }
	void operator++(int)	{ next(); }
	void next();

	VertexPointer operator->() const { return _v; }
        VertexPointer operator*() const { return _v; }
	VertexPointer operator()() const { return _v; }

	EdgePointer edge()	    { return _e; }
	PolygonPointer polygon()    { return Edge::PolygonIterator(_e)(); }

	operator bool () const	    { return bool(_e); }

    private:
	EdgePointer	    _e0;
	EdgePointer	    _e;
	VertexPointer	    _v;
    };

    //-*************************************************************************
    // PolyEdge Constructors
    //-*************************************************************************
    PolyEdge();
    ~PolyEdge();

    // Data
    const VertexArray&	    vertices() const { return _vertices; }
    const EdgeArray&	    edges() const { return _edges; }
    const PolygonArray&	    polygons() const { return _polygons; }
    const PropertyArray&    properties() const { return _properties; }

    // Sort the verices given a comparision function object
    template <class Comparitor>
    void sortVertices(Comparitor comp)
    { std::sort(_vertices.begin(), _vertices.end(), comp); }

    template <class Comparitor>
    void sortPolygons(Comparitor comp)
    { std::sort(_polygons.begin(), _polygons.end(), comp); }

    template <class Comparitor>
    void sortEdges(Comparitor comp)
    { std::sort(_edges.begin(), _edges.end(), comp); }

    template <class Comparitor>
    void sortProperties(Comparitor comp)
    { std::sort(_properties.begin(), _properties.end(), comp); }


    // Flags
    void clearEdgeFlags(Flags=~0);
    void clearVertexFlags(Flags=~0);
    void clearPolygonFlags(Flags=~0);

    // Garbage collection -- functions return the number of
    // naked/dead objects they deleted. Use garbageCollect() to get
    // everything correctly.
    int deleteDeadPolygons();
    int deleteNakedEdges();
    int deleteNakedVertices();
    int deleteUnusedProperties();

    void garbageCollect();

    // uniqueEdge():
    // Returns existing or new edge
    EdgePointer uniqueEdge(VertexPointer,VertexPointer);

    // duplicate a vertex
    VertexPointer duplicate(VertexPointer v)
    { return new Vertex(this,v->point(),v->index(),v->generation()+1); }

    // Reindex vertices assigns unique indices to all the vertices.
    void reindexVertices();

    // Reindex properties
    void reindexProperties();

    //-*************************************************************************
    // Atomic Operations
    // -----------------
    //
    // Functions which maintain the database in a sane state. These
    // functions may produce naked edges (edges with no polygons
    // attached). You can called deleteNakedEdges() to delete these
    // edges, however I recommend leaving them because many nearby
    // operations may pick up a naked edge and use it either
    // permanently or temporarily.
    //-*************************************************************************


    // killPolygon(): the polygon is not actually deleted until
    // deleteDeadPolygons() is called.
    void killPolygon(PolygonPointer);

    // killEdge(): may result in killed polygons if the edge is not
    // naked.
    void killEdge(EdgePointer);

    // replace(): replace a vertex in a polygon with another
    // vertex. If the new vertex is 0, the old vertex is simply
    // removed. If properties exist on the surface and no property is
    // specified, the new vertex will inherit the old vertex's
    // property.
    void replace(PolygonPointer,
		 VertexPointer vOld,
		 VertexPointer vNew,
		 PropertyPointer pNew=0);

    void remove(PolygonPointer p, VertexPointer v);

    // insert(): inserts a vertex into a polygon after an existing
    // vertex or in place of an existing edge.
    void insert(PolygonPointer,
		VertexPointer vNew,
		VertexPointer vAfter,
		PropertyPointer pNew=0);

    void insert(PolygonPointer,
		VertexPointer vNew,
		EdgePointer eBetween,
		PropertyPointer pNew=0);

    // split(): split a vertex along an edge. The vertex must be a
    // border vertex. Returns the duplicated vertex (or one of them
    // if there were multiple polygons attached).
    VertexPointer split(VertexPointer, EdgePointer);

    // split(): split an edge into two edges at a given edge parameter.
    VertexPointer split(EdgePointer, float u=0.5f);

    // split(): split a polygon along two of its vertices. The
    // original polygon is reduced in size.
    EdgePointer split(PolygonPointer, VertexPointer, VertexPointer);

    // splitBridge(): if the vertex is a bridge vertex, the surfaces
    // joinded at the vertex will be seperated.
    void splitBridge(VertexPointer);

    void sanityCheck(PolygonPointer);

private:
    void replaceVertex(PolygonPointer,VertexPointer,EdgePointer,VertexPointer);

    void duplicateProperties(VertexPointer);

private:
    VertexArray     _vertices;
    EdgeArray 	    _edges;
    PolygonArray    _polygons;
    PropertyArray   _properties;
};

//-*****************************************************************************
// INLINE FUNCTIONS
//-*****************************************************************************

//-*****************************************************************************
inline void PolyEdge::Vertex::EdgeIterator::operator++()
{
    if (_edge) _edge=_edge->next(_vertex);
}

//-*****************************************************************************
inline void PolyEdge::Vertex::EdgeIterator::operator++(int)
{
    if (_edge) _edge=_edge->next(_vertex);
}

//-*****************************************************************************
inline PolyEdge::Vertex::EdgeIterator::EdgeIterator(const VertexPointer v)
  : _vertex(v)
{
    _edge = v->firstEdge();
}

//-*****************************************************************************
inline void PolyEdge::Vertex::EdgeIterator::reset()
{
    _edge = _vertex->firstEdge();
}

//-*****************************************************************************
inline PolyEdge::Vertex::OrderedEdgeIterator
::OrderedEdgeIterator(const VertexPointer v)
  : _vertex(v)
{
    _edge = v->firstEdge();
}

//-*****************************************************************************
inline PolyEdge::EdgePointer
PolyEdge::uniqueEdge(PolyEdge::VertexPointer v1,
		     PolyEdge::VertexPointer v2)
{
    EdgePointer e = 0;
    return (e = v1->sharedEdge(v2)) ? e : (new Edge(this,v1,v2));
}

//-*****************************************************************************
inline
PolyEdge::Polygon::VertexIterator::VertexIterator(PolyEdge::PolygonPointer ip)
  : _i(ip)
{
    PolygonPointer p = _i.polygon();
    EdgePointer ne = _i->next(p);
    VertexPointer ov = _i->sharedVertex(ne);
    _v = _i->otherVertex(ov);
    //_v = _i->otherVertex(_i->sharedVertex(_i->next(_i.polygon())));
}

//-*****************************************************************************
std::ostream& operator<<(std::ostream &o, PolyEdge::Vertex& v);
std::ostream& operator<<(std::ostream &o, PolyEdge::Edge& e);
std::ostream& operator<<(std::ostream &o, PolyEdge::Polygon& p);
std::ostream& operator<<(std::ostream &o, PolyEdge::Property& p);

//-*****************************************************************************
// TEMPLATE FUNCTIONS
//-*****************************************************************************

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
PolyEdge::PropertyVector<T,N,Ni>::~PropertyVector() {}

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
PolyEdge::Property*
PolyEdge::PropertyVector<T,N,Ni>::copy(PolyEdge* pe) const
{
    ThisType *np = new ThisType(pe);
    for (int i=0; i<N; i++) np->data[i] = data[i];
    return np;
}

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
void
PolyEdge::PropertyVector<T,N,Ni>::interpolate(PolyEdge::Property *a,
					      PolyEdge::Property *b,
					      float u)
{
    ThisType *pa = static_cast<ThisType*>(a);
    ThisType *pb = static_cast<ThisType*>(b);

    for (int i=0; i<N; i++)
    {
	data[i] = pa->data[i] * (1.f - u) + pb->data[i] * u;
    }

    if (Ni)
    {
	T sum = T(0);
	for (int i=0; i < Ni; i++) sum += data[i] * data[i];
	if (sum != T(0))
	{
	    sum = std::sqrt(sum);
	    for (int i=0; i < Ni; i++) data[i] /= sum;
	}
    }
}

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
void
PolyEdge::PropertyVector<T,N,Ni>::interpolate(const PolyEdge::PropertyArray &pa,
					      const PolyEdge::WeightArray &wa)
{
    for (int i=0; i < N; i++)
    {
	T temp = T(0);

	for (int q=0, s=pa.size(); q < s; q++)
	{
	    ThisType *p = static_cast<ThisType*>(pa[q]);
	    temp += p->data[i] * wa[q];
	}

	data[i] = temp;
    }

    if (Ni)
    {
	T sum = T(0);
	for (int i=0; i < Ni; i++) sum += data[i] * data[i];
	if (sum != T(0))
	{
	    sum = std::sqrt(sum);
	    for (int i=0; i < Ni; i++) data[i] /= sum;
	}
    }
}

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
bool
PolyEdge::PropertyVector<T,N,Ni>::equals(const PolyEdge::Property *p) const
{
    if (p != this)
    {
	const ThisType *pa = static_cast<const ThisType*>(p);
	for (int i=0; i<N; i++)
	    if (data[i] != pa->data[i]) return false;
    }

    return true;
}

//-*****************************************************************************
template<typename T, size_t N, size_t Ni>
void
PolyEdge::PropertyVector<T,N,Ni>::print(std::ostream &o) const
{
    o << "(";
    o << this << ":";
    for (int i=0; i<N; i++)
    {
	if (i) o << " ";
	o << data[i];
    }
    o << ")";
}

} // End namespace MeshEdit
} // End namespace EmldCore

#endif
