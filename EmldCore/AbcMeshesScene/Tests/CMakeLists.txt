if (EMERALD_BUILD_GUIS)
emerald_define_executable(AbcMeshesView
    SOURCES 
        TestFoundation.h
        testViewAbc.cpp
        TestViewScene.h
        TestViewScene.cpp
    DEPENDENCIES 
        emerald::EmldCore_AbcMeshesScene
        emerald::simple_sim_viewer
        emerald::geep_glfw
        Alembic::Alembic
        OpenEXR::OpenEXR
        Boost::Boost
    )
endif()


