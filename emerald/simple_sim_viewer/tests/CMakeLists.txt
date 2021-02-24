if (EMERALD_BUILD_GUIS)
emerald_define_executable(test_simple_mesh_sim
    SOURCES 
        test_simple_mesh_sim.cpp
    DEPENDENCIES 
        emerald::simple_sim_viewer
    )

emerald_define_executable(test_simple_points_sim
    SOURCES 
        test_simple_points_sim.cpp
    DEPENDENCIES 
        emerald::simple_sim_viewer
    )

emerald_define_executable(test_simple_slab_sim
    SOURCES 
        test_simple_slab_sim.cpp
    DEPENDENCIES 
        emerald::simple_sim_viewer
    )

emerald_define_library(multi_scale_draw
  SOURCES
    multi_scale_draw.cpp
  HEADERS
    multi_scale_draw.h
  DEPENDENCIES
    emerald::simple_sim_viewer
    TBB::TBB
  )

emerald_define_executable(test_multi_particles_sim
    SOURCES 
        test_multi_particles_sim.cpp
    DEPENDENCIES 
        emerald::multi_scale_draw
    )
endif()

