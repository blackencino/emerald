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

#ifndef _EmldCore_CompactHashMap_StateReorder_h_
#define _EmldCore_CompactHashMap_StateReorder_h_

#include "Foundation.h"
#include "Zindex.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
#pragma pack( push, 1 )
template <typename SIM_COORD, typename CELL_COORD, typename INDEX>
struct RT_SizeHelpStruct
{
    SIM_COORD s;
    CELL_COORD c;
    INDEX i;
    INDEX z;
};
#pragma pack( pop )

//-*****************************************************************************
// This Sorting Tuple is what we use to order our points. It orders by
// morton (z) index first, then by cell coordinate, then by positional
// coordinate, and finally by positional index, so that a set of points
// has a precise ordering. This is important for ensuring that parallel
// operations are reproducable.
#pragma pack( push, 1 )
template < typename SIM_COORD, typename CELL_COORD, typename INDEX,
         int BYTES_PER_TUPLE =
         sizeof( SIM_COORD ) + sizeof( CELL_COORD ) + ( 2 * sizeof( INDEX ) ) >
struct ReorderTuple
{
    typedef SIM_COORD sim_coord_type;
    typedef CELL_COORD cell_coord_type;
    typedef INDEX index_type;
    typedef ReorderTuple<SIM_COORD, CELL_COORD, INDEX, BYTES_PER_TUPLE>
    this_type;

    sim_coord_type sim;
    cell_coord_type cell;
    index_type index;
    index_type z;

private:
    EMLD_STATLIT_CONSTANT int pad_bytes = 
        BYTES_PER_TUPLE -
            sizeof( RT_SizeHelpStruct<SIM_COORD, CELL_COORD, INDEX> );

public:
    byte_t __pad[pad_bytes];

    ReorderTuple() : sim( 0 ), cell( 0 ), index( 0 ), z( Zindex( cell ) ) {}
    ReorderTuple( const sim_coord_type& i_sim,
                  const cell_coord_type& i_cell,
                  index_type i_index )
        : sim( i_sim )
        , cell( i_cell )
        , index( i_index )
        , z( Zindex( cell ) ) {}

    // Having trouble with template inference.
    bool less_than( const this_type& i_other ) const
    {
        if ( this->z < i_other.z ) { return true; }
        else if ( this->z > i_other.z ) { return false; }
        else if ( this->cell < i_other.cell ) { return true; }
        else if ( this->cell > i_other.cell ) { return false; }
        else if ( this->sim < i_other.sim ) { return true; }
        else if ( this->sim > i_other.sim ) { return false; }
        else { return ( this->index < i_other.index ); }
    }

    struct less_than_comp
    {
        bool operator()( const this_type& i_a, const this_type& i_b ) const
        {
            return i_a.less_than( i_b );
        }
    };
};
#pragma pack( pop )


//-*****************************************************************************
template <typename TUPLE, typename TRANSFORM>
struct ComputeTuples
        : public ZeroForEachFunctorI<ComputeTuples<TUPLE, TRANSFORM> >
{
    typedef typename TUPLE::sim_coord_type sim_coord_type;
    typedef typename TUPLE::index_type index_type;

    const sim_coord_type* SimCoords;
    TUPLE* Tuples;

    const TRANSFORM* Transform;

    void operator()( index_type i ) const
    {
        const sim_coord_type p = SimCoords[i];
        Tuples[i] = TUPLE( p, Transform->simToCell( p ), i );
    }
};

//-*****************************************************************************
template <typename TUPLE>
struct SetFromTuples
        : public ZeroForEachFunctorI<SetFromTuples<TUPLE> >
{
    typedef typename TUPLE::sim_coord_type sim_coord_type;
    typedef typename TUPLE::cell_coord_type cell_coord_type;
    typedef typename TUPLE::index_type index_type;

    const TUPLE* Tuples;
    sim_coord_type* SimCoords;
    cell_coord_type* CellCoords;

    void operator()( index_type i ) const
    {
        const TUPLE tuple = Tuples[i];
        SimCoords[i] = tuple.sim;
        CellCoords[i] = tuple.cell;
    }
};

//-*****************************************************************************
template <typename TUPLE, typename T>
struct ReorderFromTuples
        : public ZeroForEachFunctorI<ReorderFromTuples<TUPLE, T> >
{
    typedef typename TUPLE::index_type index_type;
    const TUPLE* Tuples;
    const T* Src;
    T* Dst;

    void operator()( index_type i ) const
    {
        const TUPLE tuple = Tuples[i];
        Dst[i] = Src[ tuple.index ];
    }
};


//-*****************************************************************************
template <typename TUPLE, typename T>
struct ReverseReorderFromTuples
        : public ZeroForEachFunctorI<ReverseReorderFromTuples<TUPLE, T> >
{
    typedef typename TUPLE::index_type index_type;
    const TUPLE* Tuples;
    const T* Src;
    T* Dst;

    void operator()( index_type i ) const
    {
        const TUPLE tuple = Tuples[i];
        Dst[ tuple.index ] = Src[ i ];
    }
};


//-*****************************************************************************
template <typename TUPLE>
struct WentToFromTuples
        : public ZeroForEachFunctorI<WentToFromTuples<TUPLE> >
{
    typedef typename TUPLE::index_type index_type;
    const TUPLE* Tuples;
    index_type* WentTo;

    void operator()( index_type i ) const
    {
        const TUPLE tuple = Tuples[i];
        WentTo[ tuple.index ] = i;
    }
};

//-*****************************************************************************
template <typename TUPLE>
struct CameFromFromTuples
        : public ZeroForEachFunctorI<CameFromFromTuples<TUPLE> >
{
    typedef typename TUPLE::index_type index_type;
    const TUPLE* Tuples;
    index_type* CameFrom;

    void operator()( index_type i ) const
    {
        const TUPLE tuple = Tuples[i];
        CameFrom[i] = tuple.index;
    }
};

//-*****************************************************************************
template <typename STATE>
class StateReorder
{
public:
    typedef STATE                                   state_type;

    typedef typename STATE::index_type              index_type;

    typedef typename STATE::sort_tuple_type         tuple_type;

    typedef typename STATE::transform_type          transform_type;

    typedef typename STATE::vector_manager_type     vector_manager_type;

    typedef typename STATE::index_mvh_type          index_mvh_type;
    typedef typename STATE::sort_tuple_mvh_type     tuple_mvh_type;

    StateReorder( state_type& i_state )
    {
        const index_type N = i_state.size();

        vector_manager_type& vmgr = i_state.vectorManager();

        m_tuples = vmgr.template get<tuple_type>( N );
        {
            ComputeTuples<tuple_type, transform_type> F;
            F.Tuples = m_tuples.data();
            F.SimCoords = i_state.simCoords().cdata();
            F.Transform = &( i_state.transform() );
            F.execute( N );
        }

        typename tuple_type::less_than_comp ltc;
        ParallelUtil::VectorSort( m_tuples, ltc );

        // Reorder the simCoords and cellCoords in the state.
        {
            SetFromTuples<tuple_type> F;
            F.Tuples = m_tuples.cdata();
            F.SimCoords = i_state.simCoords().data();
            F.CellCoords = i_state.cellCoords().data();
            F.execute( N );
        }

        // Reorder the rest of the state's data.
        i_state.reorderExtraData( *this );
    }

    //-*************************************************************************
    // This function is called by the state to reorder its data.
    template <typename DATA_VECTOR>
    void reorder( DATA_VECTOR& io_data, DATA_VECTOR& tmp_data ) const
    {
        const index_type N = m_tuples.size();
        EMLD_ASSERT( io_data.size() == N, "Mismatched data size." );
        tmp_data.resize( N );

        {
            ReorderFromTuples<tuple_type, typename DATA_VECTOR::value_type> F;
            F.Tuples = m_tuples.cdata();
            F.Src = io_data.cdata();
            F.Dst = tmp_data.data();
            F.execute( N );
        }

        io_data.swap( tmp_data );
    }

    //-*************************************************************************
    // The state may request a "went-to" array.
    void wentTo( index_mvh_type o_wentTo ) const
    {
        const index_type N = m_tuples.size();
        o_wentTo.resize( N );
        {
            WentToFromTuples<tuple_type> F;
            F.Tuples = m_tuples.cdata();
            F.WentTo = o_wentTo.data();
            F.execute( N );
        }
    }

    //-*************************************************************************
    // The state may request a "came-from" array.
    void cameFrom( index_mvh_type o_cameFrom ) const
    {
        const index_type N = m_tuples.size();
        o_cameFrom.resize( N );
        {
            CameFromFromTuples<tuple_type> F;
            F.Tuples = m_tuples.cdata();
            F.CameFrom = o_cameFrom.data();
            F.execute( N );
        }
    }

protected:
    tuple_mvh_type m_tuples;
};

} // End namespace CompactHashMap
} // End namespace EmldCore

#endif
