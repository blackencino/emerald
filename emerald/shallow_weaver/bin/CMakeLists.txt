add_executable(shallow_weaver_headless
               parse_parameters.h
               parse_parameters.cpp
               headless_main.cpp)
target_link_libraries(shallow_weaver_headless
                      PUBLIC
                      emerald_shallow_weaver
                      cxxopts::cxxopts)

add_executable(shallow_weaver
               parse_parameters.h
               parse_parameters.cpp
               main.cpp)
target_link_libraries(shallow_weaver
                      PUBLIC
                      emerald_shallow_weaver
                      emerald_simple_sim_viewer
                      cxxopts::cxxopts)

