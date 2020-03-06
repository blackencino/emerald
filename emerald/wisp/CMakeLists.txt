add_library(emerald_wisp
    foundation.h
    simulation.h
    simulation_old.h
    state.h

    simulation.cpp
    simulation_old.cpp)
target_include_directories(emerald_wisp PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_wisp PUBLIC
                      emerald_shallow_weaver
                      emerald_util
                      emerald_noise
                      gsl-lite::gsl-lite
                      TBB::TBB)

#add_subdirectory(bin)
