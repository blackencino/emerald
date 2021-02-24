//-*****************************************************************************
// Copyright (c) 2001-2003 Tweak Inc. All rights reserved.
//-*****************************************************************************

//-*****************************************************************************
// Modified in 2010 by Christopher Jon Horvath to use Imath math types instead
// of TwkMath
//-*****************************************************************************

#include "FixOrientation.h"

namespace EmldCore {
namespace MeshEdit {

#include "PolyEdgeTypedefs.h"

//-*****************************************************************************
#define FIRST_WAY_FLAG (1<<9)
#define SECOND_WAY_FLAG (1<<10)
#define VISITED_FLAG (1<<11)
#define NUKE_FLAG (1<<12)

//-*****************************************************************************
static inline PolyEdge::Flags otherFlag( PolyEdge::Flags f )
{
    if ( f == FIRST_WAY_FLAG )
    {
        return SECOND_WAY_FLAG;
    }
    else
    {
        return FIRST_WAY_FLAG;
    }
}

//-*****************************************************************************
static inline PolyEdge::Flags flag( PolygonPointer p )
{
    if ( p->flag( FIRST_WAY_FLAG ) == true )
    {
        return FIRST_WAY_FLAG;
    }
    else
    {
        assert( p->flag( SECOND_WAY_FLAG ) == true );
        return SECOND_WAY_FLAG;
    }
}

//-*****************************************************************************
static void PropagatePolyFlag( PolygonPointer pp,
                               PolyEdge::Flags ppFlag,
                               int &firstWayCount,
                               int &secondWayCount )
{
    // Make sure we haven't been visited
    assert( pp->flag( VISITED_FLAG ) == false );
    assert( pp->alive() );

    // Sanity check.
    assert( flag( pp ) == ppFlag );

    // Set our flag to visited so we don't get revisited.
    pp->flag( VISITED_FLAG, true );

    // Loop over our vertices
    for ( Polygon::VertexIterator vei( pp ); vei; ++vei )
    {
        // The OUTGOING edge:
        EdgePointer oep = vei.edge();
        assert( oep != NULL );
        assert( pp->outgoingEdge( vei() ) == oep );

        // The other polygon
        PolygonPointer opp = oep->otherPolygon( pp );
        if ( opp != NULL &&
             opp->alive() &&
             opp->flag( VISITED_FLAG ) == false )
        {
            // Check orientation
            PolyEdge::Flags oppFlag;
            if ( opp->outgoingEdge( vei() ) == oep )
            {
                // Orientation mismatch
                oppFlag = otherFlag( ppFlag );
            }
            else
            {
                // Orientations same
                oppFlag = ppFlag;
            }

            // Set the flag and increment counts
            if ( oppFlag == FIRST_WAY_FLAG )
            {
                opp->flag( FIRST_WAY_FLAG, true );
                opp->flag( SECOND_WAY_FLAG, false );
                ++firstWayCount;
            }
            else
            {
                opp->flag( FIRST_WAY_FLAG, false );
                opp->flag( SECOND_WAY_FLAG, true );
                ++secondWayCount;
            }

            // Propagate
            PropagatePolyFlag( opp, oppFlag,
                               firstWayCount, secondWayCount );
        }
    }
}

//-*****************************************************************************
void FixOrientation( PolyEdge &pe )
{ 
    // Clear it up.
    pe.clearPolygonFlags( FIRST_WAY_FLAG | SECOND_WAY_FLAG |
                          VISITED_FLAG | NUKE_FLAG );

    // Make some counts.
    int firstWayCount = 0;
    int secondWayCount = 0;
    int deadCount = 0;
    const PolygonArray &polys = pe.polygons();
    const PropertyArray &properties = pe.properties();
    bool hasProperties = properties.size() != 0;

    // Find each polygon that has not been visited.
    // Assign it the "first way" flag and propagate.
    // If the model is a single surface (which it should be!)
    // this will probably only have one loop.
    for ( PolygonArray::const_iterator pi = polys.begin();
          pi != polys.end(); ++pi )
    {
        PolygonPointer pp = ( PolygonPointer )(*pi);
        if ( pp->alive() )
        {
            if ( pp->flag( VISITED_FLAG ) == false )
            {
                pp->flag( FIRST_WAY_FLAG, true );
                ++firstWayCount;
                
                // This flood fills. Only polygons that cannot be
                // walked to from this polygon will remain untouched
                // after this call.
                PropagatePolyFlag( pp, FIRST_WAY_FLAG,
                                   firstWayCount, secondWayCount );
            }
        }
        else
        {
            ++deadCount;
        }
    }

    // Check it.
    assert( firstWayCount + secondWayCount + deadCount == polys.size() );

    // Decide if we need to do anything at all
    if ( firstWayCount == 0 || secondWayCount == 0 )
    {
        // Clear flags
        pe.clearPolygonFlags( FIRST_WAY_FLAG | SECOND_WAY_FLAG |
                              VISITED_FLAG | NUKE_FLAG );
        return;
    }

    // Decide which orientation we like
    PolyEdge::Flags badFlag;
    if ( firstWayCount > secondWayCount )
    {
        badFlag = SECOND_WAY_FLAG;
    }
    else
    {
        badFlag = FIRST_WAY_FLAG;
    }

    // Make new polygons from bad old ones.
    size_t numOriginalPolygons = polys.size();
    for ( size_t i = 0; i < numOriginalPolygons; ++i )
    {
        PolygonPointer pp = polys[i];

        if ( pp->flag( badFlag ) == true )
        {
            VertexArray varray;
            PropertyArray parray;

            for ( Polygon::VertexIterator vi( pp ); vi; ++vi )
            {
                VertexPointer v = vi();
                varray.push_back( v );
                if ( hasProperties )
                {
                    parray.push_back( vi.property() );
                }
            }

            std::reverse( varray.begin(), varray.end() );
            std::reverse( parray.begin(), parray.end() );

            Polygon *np = new Polygon( &pe, varray, parray );
            np->flags() = pp->flags();

            pp->flag( NUKE_FLAG, true );
        }
    }

    // Nuke polys
    for ( size_t i = 0; i < numOriginalPolygons; ++i )
    {
        if ( polys[i]->flag( NUKE_FLAG ) )
        {
            pe.killPolygon( polys[i] );
        }
    }
    
    // Delete stuff that we don't need
    pe.deleteDeadPolygons(); 
    pe.deleteNakedEdges();
    pe.deleteNakedVertices();
    pe.deleteUnusedProperties();
    pe.garbageCollect();

    // Clear flags
    pe.clearPolygonFlags( FIRST_WAY_FLAG | SECOND_WAY_FLAG |
                          VISITED_FLAG | NUKE_FLAG );
}

} // End namespace MeshEdit
} // End namespace EmldCore

