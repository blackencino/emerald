emerald_define_test(EmldCore_SpatialSubd_DUOctreeTest
    SOURCES DUOctreeTest.cpp
    DEPENDENCIES 
        emerald::EmldCore_SpatialSubd
    )

emerald_define_test(EmldCore_SpatialSubd_Test1
    SOURCES test1.cpp
    DEPENDENCIES 
        emerald::EmldCore_SpatialSubd
    )
