add_library(emerald_shallow_weaver
    foundation.h
    simulation.h
    simulation_old.h
    slab.h
    slab_ops.h
    state.h

    simulation.cpp
    simulation_old.cpp
    slab_ops.cpp)
target_include_directories(emerald_shallow_weaver PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_shallow_weaver PUBLIC
                      emerald_util
                      emerald_noise
                      gsl-lite::gsl-lite
                      TBB::TBB)

add_subdirectory(bin)
