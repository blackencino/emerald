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

#ifndef _EmldCore_ParallelUtil_ContiguousBlocks_h_
#define _EmldCore_ParallelUtil_ContiguousBlocks_h_

#include "Foundation.h"
#include "For.h"
#include "SimpleArrayFunctors.h"
#include "Scan.h"

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
// This functor will emit a '1' at any position with index > 0 which has
// a different value, as defined by the equality predicate, at the previous
// index.
template < typename FUNCTOR,
         typename VALUE_TYPE = typename FUNCTOR::index_type,
         typename EQUAL_PRED = std::equal_to<typename FUNCTOR::value_type> >
struct ContiguousBlockBoundaryValuesFunctor
{
    typedef FUNCTOR functor_type;
    typedef VALUE_TYPE value_type;
    typedef EQUAL_PRED equal_pred_type;
    typedef typename FUNCTOR::index_type index_type;
    typedef typename FUNCTOR::value_type input_value_type;

    FUNCTOR& Functor;
    EQUAL_PRED EqualPred;

    ContiguousBlockBoundaryValuesFunctor(
        functor_type& i_functor,
        equal_pred_type const& i_equalPred = equal_pred_type() )
        : Functor( i_functor )
        , EqualPred( i_equalPred )
    {}

    value_type operator[]( index_type i ) const
    {
        if ( i == 0 ||
             EqualPred( Functor[ i - 1 ], Functor[ i ] ) )
        {
            return value_type( 0 );
        }
        else
        {
            return value_type( 1 );
        }
    }
};

//-*****************************************************************************
// So just taking the prefix sum of the above will create a list of indices
// which represents the index of the contiguous block that a value type
// occupied.
template < typename VECTOR,
         typename BLOCK_INDEX_VECTOR,
         typename EQUAL_PRED >
typename BLOCK_INDEX_VECTOR::value_type
VectorContiguousBlockIndices( const VECTOR& i_vector,
                              BLOCK_INDEX_VECTOR& o_blockIndices,
                              EQUAL_PRED const& i_equalP = EQUAL_PRED() )
{
    typedef typename VECTOR::value_type input_value_type;
    typedef typename BLOCK_INDEX_VECTOR::value_type value_type;
    typedef value_type index_type;

    typedef DirectConstValuesFunctor<input_value_type, index_type> VF_type;
    typedef ContiguousBlockBoundaryValuesFunctor < VF_type,
            value_type, EQUAL_PRED >
            CBBVF_type;

    EMLD_ASSERT( i_vector.size() == o_blockIndices.size(),
                 "Vector and BlockIndices must be the same size." );

    VF_type VF;
    VF.Values = vector_cdata( i_vector );

    CBBVF_type CBBVF( VF, i_equalP );

    return FunctorVectorPrefixSum( CBBVF, o_blockIndices );
}

//-*****************************************************************************
// Without default template argument.
template < typename VECTOR,
         typename BLOCK_INDEX_VECTOR >
typename BLOCK_INDEX_VECTOR::value_type
VectorContiguousBlockIndicesEqual
(
    const VECTOR& i_vector,
    BLOCK_INDEX_VECTOR& o_blockIndices
)
{
    typedef typename VECTOR::value_type input_value_type;
    typedef typename BLOCK_INDEX_VECTOR::value_type value_type;
    typedef value_type index_type;
    typedef std::equal_to<typename VECTOR::value_type> ep_type;

    ep_type ep;
    return VectorContiguousBlockIndices<VECTOR, BLOCK_INDEX_VECTOR, ep_type>(
               i_vector, o_blockIndices, ep );
}

//-*****************************************************************************
//-*****************************************************************************
// CREATING BLOCK BEGIN/END
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename INDEX>
struct SimpleBlock
{
    typedef INDEX index_type;
    SimpleBlock() : Begin( 0 ), End( 0 ) {}
    SimpleBlock( index_type i_begin, index_type i_end )
        : Begin( i_begin ), End( i_end ) {}

    INDEX Begin;
    INDEX End;

    INDEX size() const { return End - Begin; }
};

//-*****************************************************************************
template <typename BLOCK, typename INDEX>
struct SetSimpleBlockBeginEnd
{
    typedef BLOCK block_type;
    typedef INDEX index_type;

    void set_begin( BLOCK& o_block, INDEX i_index ) const
    {
        o_block.Begin = i_index;
    }

    void set_end( BLOCK& o_block, INDEX i_index ) const
    {
        o_block.End = i_index;
    }
};

//-*****************************************************************************
template < typename BLOCK, typename INDEX,
         typename BLOCK_SET = SetSimpleBlockBeginEnd<BLOCK, INDEX> >
struct SetBlockBeginEndFunctor
        : public ZeroForEachFunctorI <
        SetBlockBeginEndFunctor<BLOCK, INDEX, BLOCK_SET>,
        INDEX >
{
    BLOCK* Blocks;
    const INDEX* Indices;
    const BLOCK_SET* BlockSet;

    void operator()( INDEX i ) const
    {
        const INDEX thisBlockIndex = Indices[i];

        if ( i == 0 )
        {
            BlockSet->set_begin( const_cast<BLOCK*>( Blocks )[thisBlockIndex],
                                 0 );
        }
        else
        {
            const INDEX prevBlockIndex = Indices[i - 1];

            if ( prevBlockIndex != thisBlockIndex )
            {
                BlockSet->set_end( const_cast<BLOCK*>( Blocks )
                                   [prevBlockIndex], i );
                BlockSet->set_begin( const_cast<BLOCK*>( Blocks )
                                     [thisBlockIndex], i );
            }
        }

        if ( i == ( ( this->N ) - 1 ) )
        {
            BlockSet->set_end( const_cast<BLOCK*>( Blocks )[thisBlockIndex],
                               ( this->N ) );
        }
    }
};

//-*****************************************************************************
// This will resize the number of blocks.
template < typename BLOCK_INDEX_VECTOR,
         typename BLOCK_VECTOR,
         typename BLOCK_SET >
void
VectorContiguousBlockSetBeginEnd( const BLOCK_INDEX_VECTOR& i_blockIndices,
                                  BLOCK_VECTOR& o_blocks,
                                  BLOCK_SET const& i_blockSet = BLOCK_SET() )
{
    typedef typename BLOCK_VECTOR::value_type block_type;
    typedef typename BLOCK_INDEX_VECTOR::value_type index_type;

    const index_type N = i_blockIndices.size();
    if ( N < 1 )
    {
        o_blocks.clear();
        return;
    }
    const index_type numBlocks = 1 + i_blockIndices[N - 1];

    // Coarse check to make sure block indices aren't corrupt.
    EMLD_ASSERT( numBlocks <= N,
                 "Can't have more contiguous blocks than inputs. N = "
                 << N << ", numBlocks = " << numBlocks );

    o_blocks.resize( numBlocks );

    typedef SetBlockBeginEndFunctor<block_type, index_type, BLOCK_SET> F_type;
    {
        F_type F;
        F.Blocks = vector_data( o_blocks );
        F.Indices = vector_cdata( i_blockIndices );
        F.BlockSet = &i_blockSet;
        F.execute( N );
    }
}

//-*****************************************************************************
// This will resize the number of blocks.
template < typename BLOCK_INDEX_VECTOR,
         typename BLOCK_VECTOR >
void
VectorContiguousBlockSetBeginEndSimple
(
    const BLOCK_INDEX_VECTOR& i_blockIndices,
    BLOCK_VECTOR& o_blocks
)
{
    typedef typename BLOCK_VECTOR::value_type block_type;
    typedef typename BLOCK_INDEX_VECTOR::value_type index_type;
    typedef SetSimpleBlockBeginEnd<block_type, index_type> sbe_type;

    sbe_type sbe;
    return VectorContiguousBlockSetBeginEnd <
           BLOCK_INDEX_VECTOR, BLOCK_VECTOR, sbe_type > ( i_blockIndices,
                                                          o_blocks,
                                                          sbe );
}

} // End namespace ParallelUtil
} // End namespace EmldCore

#endif
