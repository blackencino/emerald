


# #-******************************************************************************
# # Bucketed Concurrent Vector Test
# ADD_EXECUTABLE( emld_test_BucketedConcurrentVector 
#                 test_BucketedConcurrentVector.cpp )
# TARGET_LINK_LIBRARIES( emld_test_BucketedConcurrentVector ${THIS_LIBS} )
# ADD_TEST( emld_TEST_BucketedConcurrentVector 
#           emld_test_BucketedConcurrentVector )
emerald_define_test(EmldCore_ParallelUtil_BucketedConcurrentVectorTest
    SOURCES test_BucketedConcurrentVector.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

# #-******************************************************************************
# # LRUCache Test
# ADD_EXECUTABLE( emld_test_LRUCache test_LRUCache.cpp )
# TARGET_LINK_LIBRARIES( emld_test_LRUCache ${THIS_LIBS} )
# ADD_TEST( emld_TEST_LRUCache emld_test_LRUCache )
emerald_define_test(EmldCore_ParallelUtil_LRUCacheTest
    SOURCES test_LRUCache.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel For Test
emerald_define_test(EmldCore_ParallelUtil_ForTest
    SOURCES test_ParallelFor.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel Reduce Test
emerald_define_test(EmldCore_ParallelUtil_ReduceTest
    SOURCES test_ParallelReduce.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel Scan Test
emerald_define_test(EmldCore_ParallelUtil_ScanTest
    SOURCES test_ParallelScan.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel Sort Test
emerald_define_test(EmldCore_ParallelUtil_SortTest
    SOURCES test_ParallelSort.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel Contiguous Blocks Test
emerald_define_test(EmldCore_ParallelUtil_ContiguousBlocksTest
    SOURCES test_ParallelContiguousBlocks.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Parallel CopyFillSet Test
emerald_define_test(EmldCore_ParallelUtil_CopyFillSetTest
    SOURCES test_ParallelCopyFillSet.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

#-******************************************************************************
# Remove Duplicates Test
emerald_define_test(EmldCore_ParallelUtil_RemoveDuplicatesTest
    SOURCES test_RemoveDuplicates.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )
