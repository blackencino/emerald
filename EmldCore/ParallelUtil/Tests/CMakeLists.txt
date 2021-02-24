#-******************************************************************************
# Bucketed Concurrent Vector Test
emerald_define_test(EmldCore_ParallelUtil_BucketedConcurrentVectorTest
    SOURCES test_BucketedConcurrentVector.cpp
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
