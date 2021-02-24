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

#ifndef _EmldCore_CompactHashMap_StateCompactHashMap_h_
#define _EmldCore_CompactHashMap_StateCompactHashMap_h_

#include "Foundation.h"
#include "VectorManager.h"
#include "IndexBlock.h"
#include "StateReorder.h"
#include "BaseCompactHashMap.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
template <typename STATE>
class StateCompactHashMap
{
public:
    typedef STATE state_type;

    typedef typename STATE::index_type              index_type;

    typedef typename STATE::sort_tuple_type         tuple_type;

    typedef typename STATE::transform_type          transform_type;

    typedef typename STATE::vector_manager_type     vector_manager_type;

    typedef typename STATE::cell_type               cell_type;
    typedef typename STATE::cell_coord_type         cell_coord_type;
    typedef typename cell_coord_type::BaseType      base_value_type;

    typedef typename STATE::sim_coord_type          sim_coord_type;
    typedef typename STATE::emit_type               emit_type;

    typedef typename STATE::index_mvh_type          index_mvh_type;
    typedef typename STATE::sort_tuple_mvh_type     tuple_mvh_type;
    typedef typename STATE::cell_mvh_type           cell_mvh_type;
    typedef typename STATE::emit_mvh_type           emit_mvh_type;

    typedef typename tuple_mvh_type::ptr_handle_type
                                                    tuple_mvph_type;

    typedef BaseCompactHashMap < cell_type,
            cell_mvh_type >       base_chm_type;

    typedef StateReorder<state_type>                reorder_type;
    typedef StateCompactHashMap<STATE>              this_type;

    StateCompactHashMap( STATE& i_state );

    STATE& state() { return m_state; }
    const STATE& state() const { return m_state; }

    vector_manager_type& vectorManager() { return state().vectorManager(); }
    const transform_type& transform() const { return state().transform(); }

    base_value_type cellSize() const { return transform().cellSize(); }

    //-*************************************************************************
    // THE NON-STRUCTURALLY-MODIFYING STUFF
    //-*************************************************************************

    // Returns the number of spatial cells.
    index_type size() const { return m_hashMap->size(); }

    // Return a spatial cell by index.
    const cell_type& cell( index_type i_cellIndex ) const
    { return m_hashMap->cell( i_cellIndex );  }

    // CAREFULLY.
    const cell_type* cellsData() const 
    { return m_hashMap->cellsData(); }

    // Find a cell, and return NULL if not found.
    const cell_type* find( const cell_coord_type& i_cellCoord ) const
    { return m_hashMap->find( i_cellCoord ); }

    //-*************************************************************************
    // THE STRUCTURALLY MODIFYING STUFF.
    //-*************************************************************************

    void initFromStateElements();

    // Returns whether data was reordered. If so, and the supplied o_wentTo
    // array is valid, the new indices will be supplied in the o_wentTo
    // array. If not reordered, the o_wentTo array will be left alone.
    bool syncFromStateElementsByChangedSimCoords(
        index_mvh_type o_wentTo = index_mvh_type(),
        bool i_forceSort = false );

    // Returns whether data was reordered. If so, and the supplied o_wentTo
    // array is valid, the new indices will be supplied in the o_wentTo
    // array. If not reordered, the o_wentTo array will be left alone.
    // Only when nothing is killed is there no reordering.
    bool killAndEmitStateElements(
        index_mvh_type i_killedIndices,
        emit_mvh_type i_emitted,
        index_mvh_type o_wentTo = index_mvh_type() );

    // Returns whether data was reordered. If so, and the supplied o_wentTo
    // array is valid, the new indices will be supplied in the o_wentTo
    // array. If not reordered, the o_wentTo array will be left alone.
    // Only when nothing is killed is there no reordering.
    // This one takes in an externally provided list of sorted emitted
    // indices. 
    bool killAndEmitStateElementsSorted(
        index_mvh_type i_killedIndices,
        emit_mvh_type i_emitted,
        index_mvh_type i_emittedIndices,
        index_mvh_type o_wentTo = index_mvh_type() );

protected:
    STATE& m_state;

    EMLD_UNIQUE_PTR<base_chm_type> m_hashMap;
    int m_hashMapIters;
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename STATE>
StateCompactHashMap<STATE>::StateCompactHashMap( STATE& i_state )
    : m_state( i_state )
    , m_hashMapIters( 10000 )
{
    // Create the base compact hash map.
    cell_mvh_type cells = vectorManager().template get<cell_type>( 0 );
    m_hashMap.reset( new base_chm_type( cells ) );
}

//-*****************************************************************************
template <typename STATE>
void StateCompactHashMap<STATE>::initFromStateElements()
{
    // Just sync with a forced rebuild.
    syncFromStateElementsByChangedSimCoords( index_mvh_type(), true );
}

//-*****************************************************************************
template <typename TUPLE, typename TUPLE_MVPH, typename TRANSFORM>
struct FindMovingElements
        : public ZeroForEachFunctorI < FindMovingElements < TUPLE,
          TUPLE_MVPH, TRANSFORM > >
{
    typedef typename TUPLE::sim_coord_type sim_coord_type;
    typedef typename TUPLE::cell_coord_type cell_coord_type;
    typedef typename TUPLE::index_type index_type;

    mutable TUPLE_MVPH Tuples;
    const sim_coord_type* SimCoords;
    cell_coord_type* CellCoords;
    const TRANSFORM* Transform;

    void operator()( index_type i ) const
    {
        const sim_coord_type Si = SimCoords[i];
        const cell_coord_type oldCi = CellCoords[i];
        const cell_coord_type newCi = Transform->simToCell( Si );

        if ( newCi != oldCi )
        {
            // cell coord has changed, this is a moving element.
            Tuples.push_back( TUPLE( Si, oldCi, i ) );

            // Set the new cell coord.
            CellCoords[i] = newCi;
        }
    }
};

//-*****************************************************************************
// Returns true if elements were reordered, in which case their new indices
// are in the went-to array, if requested.
template <typename STATE>
bool StateCompactHashMap<STATE>::syncFromStateElementsByChangedSimCoords(
    index_mvh_type o_wentTo,
    bool i_force )
{
    // Get n.
    const index_type N = state().size();

    // Save whether we were rebuilt.
    bool reordered = false;

    // Update the compact hash maps.
    ++m_hashMapIters;
    if ( i_force || m_hashMapIters >= 100 )
    {
        // Reorder
        {
            reorder_type r( state() );
            if ( o_wentTo )
            {
                o_wentTo.resize( N );
                r.wentTo( o_wentTo );
            }
        }

        // Rebuild compact hash map.
        index_mvh_type tmp_indices =
            vectorManager().template get<index_type>( N );
        m_hashMap->rebuild( state().cellCoords(), tmp_indices );

        // Reset the counter...
        m_hashMapIters = 0;

        // Reordered is TRUE.
        reordered = true;
    }
    else
    {
        // Get moving elements
        tuple_mvh_type movingElements =
            vectorManager().template get<tuple_type>( 0 );
        movingElements.clear();
        {
            FindMovingElements<tuple_type, tuple_mvph_type, transform_type> F;
            F.Tuples = movingElements.ptrHandle();
            F.SimCoords = state().simCoords().cdata();
            F.CellCoords = state().cellCoords().data();
            F.Transform = &( transform() );
            F.execute( N );
        }

        // Sort the moving elements.
        typename tuple_type::less_than_comp ltc;
        ParallelUtil::VectorSort( movingElements, ltc );

        // Move elements in the hash map. This does not change their
        // indices, but does change the cells that contain the indices.
        const index_type numMoving = movingElements.size();
        if ( numMoving > 0 )
        {
            m_hashMap->moveElements( movingElements.data(),
                                     movingElements.data() + numMoving,
                                     state().cellCoords() );
        }

        // Reordered is FALSE. Data at indices stays at those indices.
        reordered = false;
    }

#ifdef DEBUG_HARD
    m_hashMap->debugValidateElementIndexRange( 0, N, state().cellCoords() );
    m_hashMap->debugPrintOccupancyRatio();
#endif

    return reordered;
}

//-*****************************************************************************
template <typename EMIT, typename EMIT_COMP, typename INDEX>
struct EmitIndexComp
{
    const EMIT* emitted;
    const EMIT_COMP& emitComp;

    EmitIndexComp( const EMIT* i_emitted, const EMIT_COMP& i_ec )
        : emitted( i_emitted ), emitComp( i_ec ) {}

    bool operator()( INDEX i_a, INDEX i_b ) const
    {
        return emitComp( emitted[i_a], emitted[i_b] );
    }
};

//-*****************************************************************************
// If the emitted stuff does not have an externally provided set of sorted
// emitted indices, we make one and pass it down below.
template <typename STATE>
bool StateCompactHashMap<STATE>
::killAndEmitStateElements( index_mvh_type i_killedIndices,
                            emit_mvh_type i_emitted,
                            index_mvh_type o_wentTo )
{
    // Check inputs.
    EMLD_ASSERT( i_emitted,
                    "Must have emitted mvh, even if empty." );

    const index_type numEmitted = i_emitted.size();

    // Sort indices to the emitted. This is so that the emission is
    // deterministic regardless of parallelism. We sort indices to the
    // emitted, rather that the emitted themselves, because we assume the
    // emitted structures are potentially heavy and expensive to swap during
    // sorting.
    index_mvh_type emittedIndices =
            vectorManager().template get<index_type>( numEmitted );
    if ( numEmitted > 0 )
    {
        ParallelUtil::VectorSetOrderedIndices( emittedIndices );
        {
            typedef typename emit_type::less_than_comp emit_comp_type;
            typedef EmitIndexComp < emit_type,
                    emit_comp_type, index_type > eic_type;
            emit_comp_type ec;
            eic_type eic( i_emitted.cdata(), ec );
            ParallelUtil::VectorSort( emittedIndices, eic );
        }
    }

    bool ret = this->killAndEmitStateElementsSorted( i_killedIndices,
                                                     i_emitted,
                                                     emittedIndices,
                                                     o_wentTo );

    emittedIndices.reset();
    return ret;
}

//-*****************************************************************************
// This is a mostly serial function which finalizes the killing and emitting
// that is represented by the killedIndices array and the emittedParticles
// array.
// It first sorts the killed indices and an index array into the emitted
// elements.
// It then copies some number of emitted particle data into the killed
// particle locations, the minimum of the num killed and num emitted.
// If there's still some left to emit, it tacks them onto the end.
// If there's still some left to kill, it relocates the kill locations from
// the end of the array, taking care not to move anything twice.
// The emitted elements which have been MOVED have to communicate this
// move to the fine scale elements, so that their parent indices
// can be adjusted.
// This is the single most complex part of the WHOLE DAMNED SOLVER.
template <typename STATE>
bool StateCompactHashMap<STATE>
::killAndEmitStateElementsSorted( index_mvh_type i_killedIndices,
                                  emit_mvh_type i_emitted,
                                  index_mvh_type i_emittedIndices,
                                  index_mvh_type o_wentTo )
{
    // Check inputs.
    EMLD_ASSERT( i_killedIndices,
                    "Must have killed indices mvh, even if empty." );
    EMLD_ASSERT( i_emitted,
                    "Must have emitted mvh, even if empty." );
    EMLD_ASSERT( i_emitted.size() == i_emittedIndices.size(),
                    "Sorted emitted indices must be same size as emitted." );

    // Start with the number of elements before.
    const index_type numElemsBefore = state().size();

    // Don't need to re-sort if no elements were killed or deleted.
    const index_type numKilled = i_killedIndices.size();
    const index_type numEmitted = i_emitted.size();
    if ( numKilled == 0 && numEmitted == 0 ) { return false; }

    // Init the went-to, if requested.
    const bool reordered = numKilled > 0;
    if ( reordered && o_wentTo )
    {
        o_wentTo.resize( numElemsBefore );
        ParallelUtil::VectorSetOrderedIndices( o_wentTo );
    }

    // Figure out how many new elements there will be.
    const index_type numNew = std::max( numEmitted - numKilled, 0 );
    const index_type numLess = std::max( numKilled - numEmitted, 0 );

    // This is tricky, but since only one of numNew & numLess can
    // be non-zero, it works out correctly.
    const index_type numElemsAfter = numElemsBefore + numNew - numLess;

    // The number of emitted elements that go into existing locations
    // which were killed
    const index_type numKillEmit = std::min( numKilled, numEmitted );

    // The number of killed elements that don't get filled with
    // newly emitted elements.
    const index_type numKillUnemit = numKilled - numKillEmit;
    EMLD_DEBUG_ASSERT( numKillUnemit == numLess, "sanity check" );

    // Sort the killed indices.
    index_type* killedIndicesData = NULL;
    if ( numKilled > 0 )
    {
#ifdef DEBUG_HARD
        for ( index_type i = 0; i < numKilled; ++i )
        {
            for ( index_type j = 0; j < numKilled; ++j )
            {
                if ( i == j ) { continue; }
                EMLD_DEBUG_ASSERT( i_killedIndices[i] !=
                                      i_killedIndices[j],
                                      "DUPLICATE KILLED INDEX!!! EEEK!" );
            }
        }
#endif

        // Sort the killed indices in place. This is so that
        // we fill the killed spots in order from left to right.
        ParallelUtil::VectorSort( i_killedIndices );

        // Set the data.
        killedIndicesData = i_killedIndices.data();
    }

    // Use provided sorted emitter indices.
    index_type* emittedIndicesData = NULL;
    if ( numEmitted > 0 )
    {
        emittedIndicesData = i_emittedIndices.data();
    }

    // Create an array of sort tuples that can be used for communicating
    // the new and deleted element index & coords to the compact hash map.
    tuple_mvh_type movingElems =
        vectorManager().template get<tuple_type>( 0 );
    movingElems.clear();

    // Get the sim & cell coordinates.
    sim_coord_type* simCoordsData = state().simCoords().data();
    cell_coord_type* cellCoordsData = state().cellCoords().data();

    // If we killed more than we emitted, we remove these unemitted
    // kill locations backwards, and decrease the finalNumElems.
    // This is complicated and looks wrong at first. The underlying
    // compact hash map fundamentally stores indices in cells. If we
    // know we're decreasing the total amount of elements by a certain
    // amount - numKillUnemit is this amount - those indices are definitely
    // being removed from the hash map, and they're being removed from the
    // cell coordinates that they were at previously. So - the first thing
    // we do is just remove the whole index range at the end.
    // We will then move the element data from these removed locations into
    // killed locations in the state, and we'll adjust accordingly.
    // THIS TOOK FOREVER TO GET RIGHT, CHANGE IT AT YOUR PERIL.
    if ( numKillUnemit > 0 )
    {
        // Kill the elements at the end first, to try to make
        // as much room as possible.
        EMLD_DEBUG_ASSERT( numElemsAfter < numElemsBefore,
                              "more sanity checks on kill stuff" );

        for ( index_type removedIndex = numElemsAfter;
              removedIndex < numElemsBefore; ++removedIndex )
        {
            m_hashMap->removeElement( removedIndex,
                                      cellCoordsData[removedIndex] );
        }

        index_type numFinalElems = numElemsBefore;
        movingElems.clear();
        index_type takeFrom = numFinalElems - 1;
        for ( index_type ki = numKilled - 1; ki >= numEmitted; --ki )
        {
            index_type killedIndex = killedIndicesData[ki];

            if ( killedIndex >= numElemsAfter )
            {
                // We just leave this one in place, it's already
                // in the tail.
                --numFinalElems;
                continue;
            }

            // Okay, we need to make sure our takeFrom location
            // does not house a killed particle.
            for ( index_type ki2 = numKilled - 1; ki2 >= 0; --ki2 )
            {
                index_type killedIndex2 = killedIndicesData[ki2];
                if ( killedIndex2 == takeFrom )
                {
                    // Darn, the take from location is no good,
                    // keep looking for one by subtracting one
                    // and starting the comparison over.
                    // We know that we don't need to look at
                    // the ones above us, because they're all
                    // guaranteed to be greater than takeFrom.
                    --takeFrom;
                }
                else if ( killedIndex2 < takeFrom )
                {
                    // The remaining killed indices are all smaller
                    // than killedIndex2, since we're going down
                    // backwards, and therefore takeFrom is okay.
                    break;
                }
            }

            // Add it to the moved element list.
            // Moving element
            // NEW POSITION
            // OLD GRID
            // NEW INDEX
            movingElems.push_back( tuple_type( simCoordsData[takeFrom],
                                               cellCoordsData[killedIndex],
                                               killedIndex ) );

            // Move the element data in the state...
            //std::cout << "Moving state element data from: "
            //          << takeFrom << " to " << killedIndex
            //          << std::endl;
            state().copyFromIndex( takeFrom, killedIndex );

            // Mark the went-to locations correctly.
            if ( o_wentTo )
            {
                o_wentTo[takeFrom] = killedIndex;
                o_wentTo[killedIndex] = -1;
            }

            // Decrease the num final elems, and the take from.
            --numFinalElems;
            --takeFrom;
        }

        EMLD_DEBUG_ASSERT( numFinalElems == numElemsAfter,
                              "spatial resort sanity check on final elems" );

        // Sort the moving elements
        typename tuple_type::less_than_comp ltc;
        ParallelUtil::VectorSort( movingElems, ltc );

        // Move the elements that were moved. This fixes
        // the locations represented by the killedIndices
        // that we've moved into, but does not fix the locations
        // at the end.
        m_hashMap->moveElements( movingElems.begin(),
                                 movingElems.end(),
                                 state().cellCoords() );
    }

    // Fill the killed indices with the emitted element data.
    if ( numKillEmit > 0 )
    {
        movingElems.clear();
        for ( index_type ki = 0; ki < numKillEmit; ++ki )
        {
            // Use the sorted emitted indices.
            const index_type emittedIndex = emittedIndicesData[ki];
            const emit_type& emit = i_emitted[emittedIndex];

            // Killed indices have been sorted as well.
            index_type killedIndex = killedIndicesData[ki];

            // The 'moved' (in this case changed) element is at
            // the index killedIndex.
            // We want to fill it with the OLD grid value.
            movingElems.push_back(
                tuple_type( simCoordsData[killedIndex],
                            cellCoordsData[killedIndex],
                            killedIndex ) );

            // Now emit the new data into the fluid. This sets the new
            // grid values.
            state().emitIntoIndex( emit, killedIndex );

            // Mark each of the went-to locations as -1.
            if ( o_wentTo )
            {
                o_wentTo[killedIndex] = -1;
            }
        }

        // Sort the moved elements
        typename tuple_type::less_than_comp ltc;
        ParallelUtil::VectorSort( movingElems, ltc );

        // Move the compact hash map elements
        m_hashMap->moveElements( movingElems.begin(),
                                 movingElems.end(),
                                 state().cellCoords() );
    }

    // We're done with moving the elements.
    movingElems.reset();

    // These various data arrays are now invalid (or will be shortly)
    // make sure we don't make mistakes.
    killedIndicesData = NULL;
    simCoordsData = NULL;
    cellCoordsData = NULL;

    // Fill the end of the array with the new elems, if we
    // emitted more than we killed.
    if ( numNew > 0 )
    {
        // We call this ranged emit function so that hopefully emission
        // can be reasonably efficient.
        state().emitSortedByIndex( i_emitted.cdata(),
                                   emittedIndicesData + numKillEmit, // beg
                                   emittedIndicesData + numEmitted ); // end

        // Re-get cell coords data.
        cellCoordsData = state().cellCoords().data();

        // Tell the compact hash map to add a bunch of new
        // elements.
        EMLD_DEBUG_ASSERT( numElemsAfter == numElemsBefore + numNew,
                              "more sanity checks" );
        for ( index_type addedIndex = numElemsBefore;
              addedIndex < numElemsAfter;
              ++addedIndex )
        {
            m_hashMap->addElement( addedIndex, cellCoordsData[addedIndex] );
        }
    }

    // Finally, resize the fluid down.
    if ( numElemsAfter < numElemsBefore )
    {
        state().resize( numElemsAfter );
    }

    // Did we reorder?
    return reordered;
}

} // End namespace CompactHashMap
} // End namespace EmldCore
#endif