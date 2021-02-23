//-*****************************************************************************
// Copyright (c) 2010-2013, Disney-Pixar. All rights reserved.
//
// This file is part of the source code package for the paper:
//
// Mass Preserving Multi-Scale SPH
// by Christopher Jon Horvath, Barbara Solenthaler
// Pixar Technical Memo #13-04
//
// The original paper, source, and data are available here:
// 
// http://graphics.pixar.com/library/MultiScaleSPH/
// 
// Permission is hereby granted to use this software solely for 
// non-commercial applications and purposes including academic or 
// industrial research, evaluation and not-for-profit media
// production.  All other rights are retained by Pixar.  For use 
// for or in connection with commercial applications and
// purposes, including without limitation in or in connection 
// with software products offered for sale or for-profit media
// production, please contact Pixar at tech-licensing@pixar.com.
//-*****************************************************************************

#ifndef _EmldCore_ParallelUtil_LRUCache_h_
#define _EmldCore_ParallelUtil_LRUCache_h_

#include "Foundation.h"

namespace EmldCore {
namespace ParallelUtil {

#ifdef DEBUG

//-*****************************************************************************
extern tbb::atomic<std::size_t> g_totalEntriesCreated;
extern tbb::atomic<std::size_t> g_totalEntriesDeleted;

#endif

//-*****************************************************************************
template <typename DATA>
struct StdDataSizer
{
    typedef DATA data_type;
    typedef std::size_t size_type;

    size_type operator()( const DATA& i_data ) const 
    {
        return i_data.size();
    }
}; 

//-*****************************************************************************
template <typename KEY,
          typename DATA,
          typename KHASHER,
          typename DSIZER = StdDataSizer<DATA> >
class LRUCache : boost::noncopyable
{
public:
    typedef KEY key_type;
    typedef DATA data_type;
    typedef KHASHER key_hasher_type;
    typedef DSIZER data_sizer_type;

    typedef std::size_t size_type;

    typedef LRUCache<KEY,DATA,KHASHER,DSIZER> this_type;

    typedef EMLD_SHARED_PTR<DATA> data_sptr;

    //-*************************************************************************
    // Not really a public type. Here for map definition.
    struct Entry
    {
        key_type key;
        data_sptr data;
        size_type size;
        tbb::atomic<int> count;
        Entry( const key_type& i_key,
               data_sptr i_data,
               size_type i_size )
        : key( i_key ), data( i_data ), size( i_size ) 
        {
            static bool init = false;
            if ( !init )
            {
#ifdef DEBUG
                g_totalEntriesCreated = 0;
                g_totalEntriesDeleted = 0;
#endif
                init = true;
            }

            count = 0;
#ifdef DEBUG
            ++g_totalEntriesCreated;
#endif
        }

        ~Entry()
        {
#ifdef DEBUG
            std::cout << "Entry destructor. Key = " << key
                      << ", count = " << count << std::endl;

            ++g_totalEntriesDeleted;
#endif
        }
    };

    typedef EMLD_SHARED_PTR<Entry> entry_sptr;

    typedef tbb::concurrent_queue<Entry*> queue_type;

    typedef tbb::concurrent_hash_map<KEY,entry_sptr,KHASHER> map_type;
    typedef typename map_type::accessor map_accessor;
    typedef typename map_type::const_accessor map_const_accessor;

    typedef tbb::queuing_rw_mutex rw_mutex_type;
    typedef typename rw_mutex_type::scoped_lock rw_mutex_scoped_lock_type;

    //-*************************************************************************
    struct Parameters
    {
        // Size of the cache, in bytes, before it begins removing data.
        size_type evictThreshold;

        // When the cache removes data, it will remove at least this much
        // data before stopping.
        size_type evictMargin;

        // The number of buckets in the internal hash map. Usually good to
        // be around 2x the average number of entries in the cache.
        size_type mapNumBuckets;

        // When cleaning up queues, if they're smaller than this amount in
        // number of entries, don't bother cleaning up.
        size_type minQueueCleanupSize;

        // The ratio of queue size to map size which will trigger a queue
        // cleanup.
        double maxQueueCleanupRatio;

        // An absolute maximum amount of number of entries waste which will
        // be allowed in the queue.
        size_type maxQueueCleanupWaste;

        Parameters()
        : evictThreshold( 134217728 ) // 128 megabytes
        , evictMargin( 4194304 ) // 4 megabytes
        , mapNumBuckets( 1024 )  // 2x the avg expected num queue entries
        , minQueueCleanupSize( 4096 )
        , maxQueueCleanupRatio( 8.0 )
        , maxQueueCleanupWaste( 65536 )
        {}
    };

    //-*************************************************************************
    explicit LRUCache( const Parameters& i_params,
                       const DSIZER& i_dataSizer = DSIZER() );

    // Destructor just clears.
    ~LRUCache()
    {
#ifdef DEBUG
        std::cout << "Total entries created: " << g_totalEntriesCreated
                  << std::endl
                  << "Total entries deleted: " << g_totalEntriesDeleted
                  << std::endl
                  << "Map size: " << m_map->size() << std::endl;
#endif

        clear(); 
    }

    const Parameters& parameters() const { return m_params; }
    void setParameters( const Parameters& i_params )
    {
        rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, true );
        m_params = i_params;
        if ( m_map ) { m_map->rehash( m_params.mapNumBuckets ); }
    }

    //-*************************************************************************
    // GET!
    // This is basically the entire operation. You pass an object which is
    // capable of producing the data, if it does not exist. The cache
    // looks for the data by the key, and if it finds it, returns the data,
    // while updating the usage stats internally. Otherwise, the supplied
    // compute function is executed, and the data is stored and marked as
    // recently used. When the cache gets full or overly messy, it will block
    // and clean itself up. Any data that is externally held by shared pointers
    // will stay valid.
    // This function can be called by threads in parallel, safely. There is
    // a very slight chance that multiple threads will be asked to produce
    // the same data at the same time, but the design of this cache tries 
    // to minimize that occurrance. If that does happen, each thread will
    // receive a pointer to the data it produced, though only the most recently
    // produced will live in the cache itself.
    template <typename PRODUCER>
    data_sptr get( const KEY& i_key, PRODUCER& i_prod );

#if 0
    //-*************************************************************************
    // FIND!
    // Find is just get, but if it can't find it, returns a null_ptr.
    // If it does find it, the usage is updated.
    data_sptr find( const KEY& i_key );

    //-*************************************************************************
    // CONTAINS!
    // Returns whether the map contains the key, without adjusting usage.
    bool contains( const KEY& i_key );

    //-*************************************************************************
    // STORE!
    // Store is just get, with data already provided. It is assumed that
    // ownership of the data is passed to the cache, and we will delete the
    // data when it is not used. The shared pointer returned refers either
    // to the data, or to previously cached data that has the same key
    // (and should be interchangeable)
    data_sptr store( const KEY& i_key, DATA* i_data, size_type i_dataSize );
#endif

    //-*************************************************************************
    // CLEAR - not thread safe.
    void clear();

    //-*************************************************************************
    // QUERY FUNCTIONS. 
    // The unsafe ones don't use a read mutex, so may be inaccurate if called
    // from multiple threads.
    size_type unsafeDataSize() const
    {
        return m_dataSize;
    }

    size_type unsafeSize() const
    {
        if ( ( bool )m_map ) { return m_map->size(); }
        else { return 0; }
    }

    size_type dataSize()
    {
        rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );
        return unsafeDataSize();
    }

    size_type size()
    {
        rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );
        return unsafeSize();
    }


private:
#if 0
    // This functor can be used to turn find into get.
    struct ExternallySuppliedDcomp : boost::noncopyable
    {
        ExternallySuppliedDcomp( DATA* i_data, size_type i_size )
        : data( i_data )
        , size( i_size )
        {}

        ~ExternallySuppliedDcomp()
        {
            if ( data && size )
            {
                delete data;
                data = NULL;
                size = 0;
            }
        }

        DATA* operator()( size_type& o_size )
        {
            o_size = size;
            DATA* ret = data;
            size = 0;
            data = NULL;
            return ret;
        }

        DATA* data;
        size_type size;
    };
#endif

    void tryPurge();
    void tryEvict();

    Parameters m_params;
    DSIZER m_dataSizer;

    EMLD_UNIQUE_PTR<map_type> m_map;
    EMLD_UNIQUE_PTR<queue_type> m_queue;

    rw_mutex_type m_purgeEvictMutex;

    tbb::atomic<size_type> m_dataSize;

};

//-*****************************************************************************
//-*****************************************************************************
// IMPLEMENTATION
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename KEY, 
          typename DATA, 
          typename KHASHER,
          typename DSIZER>
LRUCache<KEY,DATA,KHASHER,DSIZER>::LRUCache(
    const Parameters& i_params,
    const DSIZER& i_dsizer )
    : m_params( i_params )
    , m_dataSizer( i_dsizer )
{
    // This will make a new queue and a new map, both empty.
    clear();
}

//-*****************************************************************************
// get
template <typename KEY, 
          typename DATA, 
          typename KHASHER,
          typename DSIZER>
template <typename PRODUCER>
typename LRUCache<KEY,DATA,KHASHER,DSIZER>::data_sptr
LRUCache<KEY,DATA,KHASHER,DSIZER>::get( const KEY& i_key, PRODUCER& i_prod )
{
    // Get read lock on the purge/evict.
    // This is non-blocking so long as nothing is trying to purge or
    // evict.
    rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );

    // lookup entry by key k
    map_const_accessor cacc;
    if ( m_map->find( cacc, i_key ) )
    {
        // Got it.
        entry_sptr e = (*cacc).second;
        EMLD_ASSERT( ( bool )e, "shouldn't have empty ptr in map." );

        // put it on the front of the queue, and increment its count
        ++(e->count);
        m_queue->push( e.get() );

        // Return data sptr from entry.
        data_sptr ret = e->data;

        // Release accessor.
        cacc.release();

        // Release the purge-emit read lock, try to purge.
        peLock.release();
        tryPurge();

        // return data!
        return ret;
    }
    else
    {
        // Didn't find the data. Gotta create it.
        map_accessor acc;
        if ( m_map->insert( acc, i_key ) )
        {
            // We have exclusive write access to this entry.
            // It's okay to have blocking access to this entry
            // while we have read access to the purge lock.

            // Create the data. Catch any exception and rethrow it,
            // taking care to release locks
            peLock.release();
            data_sptr d;
            try
            {
                d.reset( i_prod() );
            }
            catch ( ... )
            {
#ifdef DEBUG
                std::cout << "EXCEPTION CAUGHT IN LRU CACHE DATA PROD"
                          << std::endl;
#endif
                peLock.release();
                acc.release();
                m_map->erase( acc );
                throw;
            }

            // Get the data's size.
            size_type dsize = m_dataSizer( *d );

            // Get the old entry sptr, and remove its data size.
            entry_sptr oldE = (*acc).second;
            if ( ( bool )oldE )
            {
                m_dataSize -= oldE->size;
            }

            // Create an entry sptr
            entry_sptr e( new Entry( i_key, d, dsize ) );

            // Insert it into the map via the accessor.
            (*acc).second = e;
            acc.release();

            // Re-get peLock.
            peLock.acquire( m_purgeEvictMutex, false );

            // Put it on the front of the queue and increment its count
            ++(e->count);
            m_queue->push( e.get() );

            // Increase our data size.
            m_dataSize += dsize;

            // Release the purge-emit read lock, try to evict.
            peLock.release();
            tryEvict();

            // Return the data.
            return d;
        }
        else
        {
            // Return the data!
            entry_sptr e = (*acc).second;
            EMLD_ASSERT( ( bool )e, "shouldn't have empty ptr in map." );

            ++(e->count);
            m_queue->push( e.get() );

            acc.release();

            return e->data;
        }
    }
}

#if 0
//-*****************************************************************************
// find
template <typename KEY, 
          typename DATA, 
          typename KHASH>
typename LRUCache<KEY,DATA,KHASH>::data_sptr
LRUCache<KEY,DATA,KHASH>::find( const KEY& i_key )
{
    // Get read lock on the purge/evict.
    // This is non-blocking so long as nothing is trying to purge or
    // evict.
    rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );

    // lookup entry by key k
    map_const_accessor cacc;
    if ( m_map->find( cacc, i_key ) )
    {
        // Got it. get entry from weak ptr, release accessor.
        entry_sptr e = (*cacc).lock();
        EMLD_ASSERT( ( bool )e, "shouldn't have empty ptr in map." );
        cacc.release();

        // put it on the front of the queue.
        m_queue->push( e );

        // Return data sptr from entry.
        data_sptr ret = e->data;

        // Release the purge-emit read lock, try to purge.
        peLock.release();
        tryPurge();

        // return data!
        return ret;
    }
    else
    {
        // Didn't find it, return empty pointer.
        data_sptr nret;
        return nret;
    }
}

//-*****************************************************************************
// contains
template <typename KEY, 
          typename DATA, 
          typename KHASH>
bool
LRUCache<KEY,DATA,KHASH>::contains( const KEY& i_key )
{
    // Get read lock on the purge/evict.
    // This is non-blocking so long as nothing is trying to purge or
    // evict.
    rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );

    return ( m_map->count( i_key ) > 0 );
}

//-*****************************************************************************
// Store.
template <typename KEY, 
          typename DATA, 
          typename KHASH>
typename LRUCache<KEY,DATA,KHASH>::data_sptr
LRUCache<KEY,DATA,KHASH>::store( const KEY& i_key, 
                                 DATA* i_data, size_type i_size )
{
    ExternallySuppliedDcomp dcomp( i_data, i_size );
    return get( i_key, dcomp );
}
#endif

//-*****************************************************************************
template <typename KEY,
          typename DATA,
          typename KHASHER,
          typename DSIZER>
void
LRUCache<KEY,DATA,KHASHER,DSIZER>::clear()
{
    // Get write lock
    rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, true );

    // Delete the map. This clears it out, and acts as a signal to the
    // internal unmap functions to not do anything.
    m_map.reset();

    // This causes the old queue to be deleted, which in turn will delete
    // all its entries, which will call "unmap" a bunch of times, which
    // will do nothing because map sptr is null.
    m_queue.reset( new queue_type );

    // And then we replace the map.
    m_map.reset( new map_type( m_params.mapNumBuckets ) );

    // Set the data size to zero.
    m_dataSize = 0;
}

//-*****************************************************************************
template <typename KEY, 
          typename DATA, 
          typename KHASHER,
          typename DSIZER>
void
LRUCache<KEY,DATA,KHASHER,DSIZER>::tryPurge()
{
    // Try to get a read-lock on purge-evict mutex. If we can't, just
    // exit. Other threads will purge.
    rw_mutex_scoped_lock_type peLock;
    if ( !peLock.try_acquire( m_purgeEvictMutex, false ) )
    { return; }

    // Check the queue size, to see if it's a problem at all.
    size_type qsize = m_queue->unsafe_size();
    if ( qsize < m_params.minQueueCleanupSize ) { return; }

    // Get the map size.
    size_type msize = m_map->size();
    if ( msize < 1 ) { msize = 1; }

    double ratio = double( qsize ) / double( msize );

    // If the ratio is not large, and the difference is not larger than
    // the max queue waste, we can leave.
    if ( ratio < m_params.maxQueueCleanupRatio &&
         ( qsize - msize ) < m_params.maxQueueCleanupWaste )
    { return; }

    // Things are bloated, we must clean up.  Get a write lock.
    peLock.upgrade_to_writer();

    // Make sure it didn't get fixed while we were converting to a writer.
    qsize = m_queue->unsafe_size();
    msize = m_map->size();
    if ( msize < 1 ) { msize = 1; }
    ratio = double( qsize ) / double( msize );
    if ( qsize < m_params.minQueueCleanupSize || 
         ( ratio < m_params.maxQueueCleanupRatio &&
           ( qsize - msize ) < m_params.maxQueueCleanupWaste ) )
    {
        // Somebody got to it before we did!
        return;
    }

    // Okay, we're cleaning now. that means that nobody can modify
    // the queue because we're locking it.
    EMLD_UNIQUE_PTR<queue_type> nq( new queue_type );

    // Pop each shared pointer off the existing queue. Check its ref
    // count. If it is unique, push it onto the new q. Otherwise, just carry
    // on.
    std::cout << "Queue size before purge: " << qsize 
              << ", compared to map size: " << msize << std::endl
              << "Data size: " << m_dataSize << std::endl;
    for ( int i = 0; i < qsize; ++i )
    {
        Entry* e;
        if ( !m_queue->try_pop( e ) ) { break; }

        EMLD_ASSERT( e != NULL, "Null entry from queue" );

        // CJH HACK
        //if ( e.unique() )
        int count = e->count.fetch_and_decrement();
        if ( count == 1 )
        {
            std::cout << "Unique ptr with key: " << e->key << std::endl;

            // put e in the new queue
            ++(e->count);
            nq->push( e );
        }
        else
        {
            //std::cout << "Ptr with key: " << e->key << " used: " 
            //          << e->count << " times" << std::endl;
        }
    }

    // Old queue should be empty
    EMLD_DEBUG_ASSERT( m_queue->unsafe_size() == 0, 
                       "Old queue should be empty" );

    // New queue should be same size as map.
    EMLD_DEBUG_ASSERT( nq->unsafe_size() == m_map->size(),
                       "Map and queue should be same size after purge"
                       << ", map size: " << m_map->size()
                       << ", queue size: " << nq->unsafe_size() );

    // Swap the new queue with the old.
    m_queue.swap( nq );
}

//-*****************************************************************************
template <typename KEY, 
          typename DATA, 
          typename KHASHER,
          typename DSIZER>
void
LRUCache<KEY,DATA,KHASHER,DSIZER>::tryEvict()
{
    // Get read-lock on purge-evict mutex.
    rw_mutex_scoped_lock_type peLock( m_purgeEvictMutex, false );

    // If data size is lower than the evict threshold, just exit.
    if ( m_dataSize < m_params.evictThreshold ) 
    {
#ifdef DEBUG
        std::cout << "Not evicting because data size: " << m_dataSize
                  << " is less than evict thresh: " << m_params.evictThreshold
                  << std::endl;
#endif
        return; 
    }

    // Upgrade to writer lock. CJH HACK NOTE - not sure if I need to
    peLock.upgrade_to_writer();

#ifdef DEBUG
    std::cout << "TRY EVICT" << std::endl
              << " data size: " << m_dataSize
              << ", evict threshold: " << m_params.evictThreshold
              << std::endl;
#endif

    // Check to make sure somebody hasn't gotten to it before us.
    if ( m_dataSize < m_params.evictThreshold ) { return; }

    // Okay, start evicting.
    size_type evictTarget = m_params.evictThreshold - m_params.evictMargin;
    while ( m_dataSize > evictTarget )
    {
        // Just start popping. This will trigger the deleter if the
        // entry_sptr is unique
        Entry* e;
        if ( !m_queue->try_pop( e ) )
        {
            return;
        }

        EMLD_ASSERT( e != NULL, "Null entry in queue" );

        int count = e->count.fetch_and_decrement();
        if ( count == 1 )
        {
            map_accessor acc;
            if ( m_map->find( acc, e->key ) )
            {
#ifdef DEBUG
                std::cout << "Deleting entry with key: " << e->key << std::endl;
#endif
                m_map->erase( acc );
                m_dataSize -= e->size;
            }
        }
    }
}

} // End namespace ParallelUtil
} // End namespace EmldCore

#endif
