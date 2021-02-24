


# #-******************************************************************************
# # Bucketed Concurrent Vector Test
# ADD_EXECUTABLE( emld_test_BucketedConcurrentVector 
#                 test_BucketedConcurrentVector.cpp )
# TARGET_LINK_LIBRARIES( emld_test_BucketedConcurrentVector ${THIS_LIBS} )
# ADD_TEST( emld_TEST_BucketedConcurrentVector 
#           emld_test_BucketedConcurrentVector )

# #-******************************************************************************
# # LRUCache Test
# ADD_EXECUTABLE( emld_test_LRUCache test_LRUCache.cpp )
# TARGET_LINK_LIBRARIES( emld_test_LRUCache ${THIS_LIBS} )
# ADD_TEST( emld_TEST_LRUCache emld_test_LRUCache )

# #-******************************************************************************
# # Parallel For Test
# ADD_EXECUTABLE( emld_test_ParallelFor test_ParallelFor.cpp )
# TARGET_LINK_LIBRARIES( emld_test_ParallelFor ${THIS_LIBS} )
# ADD_TEST( emld_TEST_ParallelFor emld_test_ParallelFor )
emerald_define_test(EmldCore_ParallelUtil_ForTest
    SOURCES test_ParallelFor.cpp
    DEPENDENCIES 
        emerald::EmldCore_ParallelUtil
        emerald::EmldCore_Util 
    )

# #-******************************************************************************
# # Parallel Reduce Test
# ADD_EXECUTABLE( emld_test_ParallelReduce test_ParallelReduce.cpp )
# TARGET_LINK_LIBRARIES( emld_test_ParallelReduce ${THIS_LIBS} )
# ADD_TEST( emld_TEST_ParallelReduce emld_test_ParallelReduce )

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

# #-******************************************************************************
# # Parallel Contiguous Blocks Test
# ADD_EXECUTABLE( emld_test_ParallelContiguousBlocks 
#                 test_ParallelContiguousBlocks.cpp )
# TARGET_LINK_LIBRARIES( emld_test_ParallelContiguousBlocks ${THIS_LIBS} )
# ADD_TEST( emld_TEST_ParallelContiguousBlocks 
#           emld_test_ParallelContiguousBlocks )

# #-******************************************************************************
# # Parallel CopyFillSet Test
# ADD_EXECUTABLE( emld_test_ParallelCopyFillSet test_ParallelCopyFillSet.cpp )
# TARGET_LINK_LIBRARIES( emld_test_ParallelCopyFillSet ${THIS_LIBS} )
# ADD_TEST( emld_TEST_ParallelCopyFillSet emld_test_ParallelCopyFillSet )

# #-******************************************************************************
# # Remove Duplicates Test
# ADD_EXECUTABLE( emld_test_RemoveDuplicates 
#                 test_RemoveDuplicates.cpp )
# TARGET_LINK_LIBRARIES( emld_test_RemoveDuplicates ${THIS_LIBS} )
# ADD_TEST( emld_TEST_RemoveDuplicates 
#           emld_test_RemoveDuplicates )
