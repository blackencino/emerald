add_executable(wisp_headless
               parse_parameters.h
               parse_parameters.cpp
               headless_main.cpp)
target_link_libraries(wisp_headless
                      PUBLIC
                      emerald_wisp
                      cxxopts::cxxopts)

if (${EMERALD_BUILD_GUIS})
    add_executable(wisp
                   parse_parameters.h
                   parse_parameters.cpp
                   main.cpp)
    target_link_libraries(wisp
                          PUBLIC
                          emerald_wisp
                          emerald_simple_sim_viewer
                          cxxopts::cxxopts)
endif()

