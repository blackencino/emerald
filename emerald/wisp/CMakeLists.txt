emerald_define_library(wisp
  HEADERS
    foundation.h
    simulation.h
    simulation_old.h
    state.h
  SOURCES
    simulation.cpp
    simulation_old.cpp
  DEPENDENCIES
    emerald::shallow_weaver
    emerald::util 
    emerald::noise 
    gsl-lite::gsl-lite 
    TBB::tbb
  )

add_subdirectory(bin)
