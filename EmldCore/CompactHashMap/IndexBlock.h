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

#ifndef _EmldCore_CompactHashMap_IndexBlock_h_
#define _EmldCore_CompactHashMap_IndexBlock_h_

#include "Foundation.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
#pragma pack( push, 1 )
template <typename COORD>
struct TempIndexBlock
{
    TempIndexBlock* blockPointer;
    COORD coord;
    int numValues;
};
#pragma pack( pop )

//-*****************************************************************************
template <typename COORD_TYPE, typename INDEX_TYPE, int BYTES_PER_BLOCK>
struct IB_SizeHelpStruct
{
    EMLD_STATLIT_CONSTANT int num_base_bytes =
        sizeof( TempIndexBlock<COORD_TYPE> );

    EMLD_STATLIT_CONSTANT int num_indices =
        ( BYTES_PER_BLOCK - num_base_bytes ) / sizeof( INDEX_TYPE );

    EMLD_STATLIT_CONSTANT int num_pad_bytes =
        ( BYTES_PER_BLOCK -
          ( num_base_bytes + num_indices* sizeof( INDEX_TYPE ) ) );
};

//-*****************************************************************************
#pragma pack( push, 1 )
template <typename COORD_TYPE, typename INDEX_TYPE, int BYTES_PER_BLOCK>
class IndexBlock
{
public:
    //-*************************************************************************
    // TYPEDEFS AND CONSTS
    //-*************************************************************************
    typedef COORD_TYPE coord_type;
    typedef INDEX_TYPE index_type;
    typedef INDEX_TYPE value_type;
    typedef IndexBlock<COORD_TYPE, INDEX_TYPE , BYTES_PER_BLOCK> this_type;

    typedef IndexBlock<COORD_TYPE, INDEX_TYPE , BYTES_PER_BLOCK> block_type;
    typedef block_type*        block_pointer;
    typedef const block_type*  const_block_pointer;
    typedef block_type&        block_reference;
    typedef const block_type&  const_block_reference;

    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;

    typedef std::size_t size_type;

    EMLD_STATLIT_CONSTANT int max_block_size =
        IB_SizeHelpStruct<COORD_TYPE, INDEX_TYPE, BYTES_PER_BLOCK>::num_indices;
    EMLD_STATLIT_CONSTANT int pad_bytes =
        IB_SizeHelpStruct<COORD_TYPE, INDEX_TYPE, BYTES_PER_BLOCK>::num_pad_bytes;


    //protected:
public:
    //-*************************************************************************
    // PROTECTED DATA
    //-*************************************************************************
    block_pointer m_nextBlock;
    value_type m_indices[max_block_size];
    coord_type m_coord;
    int m_sizeThisBlock;
    byte_t __pad[pad_bytes];

public:
    //-*************************************************************************
    // PUBLIC FUNCTIONS
    //-*************************************************************************

    //-*************************************************************************
    IndexBlock()
        : m_nextBlock( NULL )
        , m_coord( 0 )
        , m_sizeThisBlock( 0 ) {}

    //-*************************************************************************
    // Initialize to one index.
    void init( const coord_type& i_coord,
               const index_type& i_index )
    {
        m_nextBlock = NULL;
        m_coord = i_coord;
        m_indices[0] = i_index;
        m_sizeThisBlock = 1;
    }

    //-*************************************************************************
    // COPY AND ASSIGNMENT MUST BE ALLOWED
    // Otherwise we can't put these in vectors.
    // however, it's scary - must be super careful with allocated
    // overflow blocks.
    //-*************************************************************************

    //-*************************************************************************
    // Just return the size of this sub block.
public:
    int sizeThisBlock() const
    {
        return m_sizeThisBlock;
    };

    //-*************************************************************************
    // This can be expensive, because it walks the chain of blocks,
    // adding up each of their sizes.
    size_type size() const
    {
        if ( !m_nextBlock )
        {
            return static_cast<size_type>( m_sizeThisBlock );
        }
        else
        {
            EMLD_DEBUG_ASSERT( m_sizeThisBlock == max_block_size,
                               "corrupt data block" );
            return static_cast<size_type>( max_block_size ) +
                   m_nextBlock->size();
        }
    }

    //-*************************************************************************
    // Returns whether the block is empty
    bool empty() const
    {
#ifdef DEBUG
        if ( m_sizeThisBlock == 0 )
        {
            EMLD_DEBUG_ASSERT( m_nextBlock == NULL,
                               "corrupt data block" );
            return true;
        }
        else
        {
            return false;
        }
#else
        return m_sizeThisBlock == 0;
#endif
    }

    //-*************************************************************************
    // Returns whether or not a value exists in the block.
    bool contains( const_reference i_value ) const
    {
        for ( int i = 0; i < m_sizeThisBlock; ++i )
        {
            if ( m_indices[i] == i_value )
            {
                return true;
            }
        }

        if ( m_nextBlock )
        {
            EMLD_DEBUG_ASSERT( m_sizeThisBlock == max_block_size,
                               "corrupt data block" );
            return m_nextBlock->contains( i_value );
        }

        return false;
    }

    //-*************************************************************************
    index_type first() const
    {
        EMLD_DEBUG_ASSERT( m_sizeThisBlock > 0,
                           "Called first on an empty block." );
        return m_indices[0];
    }

    //-*************************************************************************
    const coord_type& coord() const
    {
#ifdef DEBUG
        if ( m_nextBlock )
        {
            EMLD_DEBUG_ASSERT( m_nextBlock->coord() == m_coord,
                               "Mismatched coordinates in blocks." );
        }
#endif
        return m_coord;
    }

    //-*************************************************************************
    void setCoord( const coord_type& i_coord )
    {
        m_coord = i_coord;
        if ( m_nextBlock )
        {
            m_nextBlock->setCoord( i_coord );
        }
    }

    //-*************************************************************************
    void setBeginIndex( index_type i_index )
    {
        EMLD_DEBUG_ASSERT( m_nextBlock == NULL &&
                           m_sizeThisBlock == 0,
                           "Cannot set begin index on a non-empty block." );
        m_indices[0] = i_index;
    }

    //-*************************************************************************
    void setEndIndex( index_type i_index )
    {
        EMLD_DEBUG_ASSERT( m_nextBlock == NULL &&
                           m_sizeThisBlock == 0,
                           "Cannot set end index on a non-empty block." );
        m_indices[1] = i_index;
    }

    //-*************************************************************************
    template <typename BLOCK_ALLOCATOR>
    void completeFromBeginEndIndices( BLOCK_ALLOCATOR& i_alloc )
    {
        EMLD_DEBUG_ASSERT( m_nextBlock == NULL &&
                           m_sizeThisBlock == 0,
                           "Cannot complete begin-end indices on a "
                           "non-empty block." );
        index_type i_begin = m_indices[0];
        index_type i_end = m_indices[1];
        EMLD_DEBUG_ASSERT( i_begin <= i_end, "corrupt begin/end" );
        index_type n = i_end - i_begin;

        // The int cast is necessary for weird, weird reasons having
        // to do with static const expressions.
        index_type toAddThisBlock = std::min( n, int( max_block_size ) );
        for ( index_type i = 0; i < toAddThisBlock; ++i )
        {
            m_indices[i] = i_begin + i;
        }
        m_sizeThisBlock = toAddThisBlock;

        if ( n > max_block_size )
        {
            EMLD_DEBUG_ASSERT( m_sizeThisBlock == max_block_size,
                               "completion error" );

            m_nextBlock = i_alloc();
            m_nextBlock->setBeginIndex( i_begin + max_block_size );
            m_nextBlock->setEndIndex( i_end );
            m_nextBlock->completeFromBeginEndIndices( i_alloc );
        }
    }

    //-*************************************************************************
    // Push an index. Needs to be able to allocate blocks. Not thread safe.
    template <typename BLOCK_ALLOCATOR>
    void addIndex( index_type i_index, BLOCK_ALLOCATOR& i_alloc )
    {
        EMLD_DEBUG_ASSERT( !contains( i_index ),
                           "Cannot doubly insert indices in a block" );

        if ( m_sizeThisBlock < max_block_size )
        {
            EMLD_DEBUG_ASSERT( m_nextBlock == NULL,
                               "corrupt data block" );
            m_indices[m_sizeThisBlock] = i_index;
            ++m_sizeThisBlock;
        }
        else
        {
            EMLD_DEBUG_ASSERT( m_sizeThisBlock == max_block_size,
                               "corrupt data block" );
            if ( m_nextBlock )
            {
                m_nextBlock->addIndex( i_index, i_alloc );
            }
            else
            {
                m_nextBlock = i_alloc();
                m_nextBlock->init( m_coord, i_index );
                EMLD_DEBUG_ASSERT( m_nextBlock->size() == 1,
                                   "corrupt data block" );
            }
        }
    }

    //-*************************************************************************
    // Pop the last element off the list, which may destroy a block.
    template <typename BLOCK_DEALLOCATOR>
    index_type pop_back( BLOCK_DEALLOCATOR& i_dealloc )
    {
        EMLD_DEBUG_ASSERT( m_sizeThisBlock != 0,
                           "Cannot remove from an empty block" );

        if ( m_nextBlock )
        {
            index_type iback = m_nextBlock->pop_back( i_dealloc );
            if ( m_nextBlock->empty() )
            {
                i_dealloc( m_nextBlock );
                m_nextBlock = NULL;
            }
            return iback;
        }
        else
        {
            index_type iback = m_indices[m_sizeThisBlock - 1];
            --m_sizeThisBlock;
            return iback;
        }
    }

    //-*************************************************************************
    // Remove an index. Needs to be able to destroy blocks. Not thread safe
    template <typename BLOCK_DEALLOCATOR>
    void removeIndex( index_type i_index, BLOCK_DEALLOCATOR& i_dealloc )
    {
        EMLD_DEBUG_ASSERT( m_sizeThisBlock != 0,
                           "Cannot remove from an empty block" );

        for ( size_type i = 0; i < m_sizeThisBlock; ++i )
        {
            if ( m_indices[i] == i_index )
            {
                // Replace this element with the last element, and decrease
                // the count, while also cleaning up.
                // The popping back will decrease the count
                // as it does the popping.
                m_indices[i] = pop_back( i_dealloc );

                // Index removed, return.
                return;
            }
        }

        // If we get here, it's not in this block. If there's a next block
        // try to remove it from there, and clean up if necessary.
        if ( m_nextBlock )
        {
            EMLD_DEBUG_ASSERT( m_sizeThisBlock == max_block_size,
                               "corrupt data block" );
            m_nextBlock->removeIndex( i_index, i_dealloc );
            if ( m_nextBlock->empty() )
            {
                i_dealloc( m_nextBlock );
                m_nextBlock = NULL;
            }

            return;
        }

        // If we get here, it means there was no next block and we didn't
        // find the element, which is an error.
        EMLD_THROW( "Index: " << i_index
                    << " not found in block for removal" );
    }

    //-*************************************************************************
    void print( int i_linkNum = 0 ) const
    {
        for ( index_type i = 0; i < m_sizeThisBlock; ++i )
        {
            std::cout << coord() << ", " << i_linkNum << ", "
                      << m_indices[i] << std::endl;
        }

        if ( m_nextBlock )
        {
            m_nextBlock->print( i_linkNum + 1 );
        }
    }

    //-*************************************************************************
protected:
    const_block_pointer nextBlock() const
    {
        return m_nextBlock;
    }

    //-*************************************************************************
    // Clear.
    //-*************************************************************************
public:
    template <typename BLOCK_DEALLOCATOR>
    void clear( BLOCK_DEALLOCATOR& i_dealloc )
    {
        if ( m_nextBlock )
        {
            m_nextBlock->clear( i_dealloc );
            i_dealloc( m_nextBlock );
            m_nextBlock = NULL;
        }
        m_sizeThisBlock = 0;
    }

    //-*************************************************************************
    // ACCESS - read only.
    //-*************************************************************************
public:
    const_reference at( index_type i_index ) const
    {
        if ( i_index >= max_block_size )
        {
            EMLD_DEBUG_ASSERT( m_nextBlock != NULL,
                               "out of bound index request" );
            return m_nextBlock->at( i_index - max_block_size );
        }
        else
        {
            EMLD_DEBUG_ASSERT( i_index >= 0 &&
                               i_index < m_sizeThisBlock,
                               "out of bound index request" );
            return m_indices[i_index];
        }
    }

    //-*************************************************************************
    // CONST FORWARD ITERATION
    //-*************************************************************************
public:
    class const_iterator
    {
    protected:
        const_block_pointer m_blockPtr;
        size_type m_localIndex;
        size_type m_overallIndex;

    public:
        const_iterator()
            : m_blockPtr( NULL )
            , m_localIndex( 0 )
            , m_overallIndex( 0 )
        {}

        explicit const_iterator( const_block_reference i_block )
            : m_blockPtr( &i_block )
            , m_localIndex( 0 )
            , m_overallIndex( 0 )
        {}

        bool unfinished() const
        {
            return m_blockPtr && ( m_localIndex < m_blockPtr->sizeThisBlock() );
        }

        operator bool() const { return unfinished(); }

        bool operator!() const { return !unfinished(); }

        void increment()
        {
            if ( unfinished() )
            {
                ++m_localIndex;
                ++m_overallIndex;
                if ( m_localIndex >= m_blockPtr->sizeThisBlock() &&
                     m_blockPtr->nextBlock() )
                {
                    m_localIndex = 0;
                    m_blockPtr = m_blockPtr->nextBlock();
                }
            }
        }

        // ++iter
        typename block_type::const_iterator&
        operator++()
        {
            increment();
            return *this;
        }

        // iter++
        typename block_type::const_iterator
        operator++( int )
        {
            typename block_type::const_iterator ret( *this );
            this->increment();
            return ret;
        }

        // Dereference
        const_reference value() const
        { return m_blockPtr->at( m_localIndex ); }

        const_reference operator*() const { return value(); }

        // Get the local and overall index in the block
        size_type localIndex() const { return m_localIndex; }
        size_type overallIndex() const { return m_overallIndex; }
    };
};
#pragma pack( pop )

} // End namespace CompactHashMap
} // End namespace EmldCore

#endif