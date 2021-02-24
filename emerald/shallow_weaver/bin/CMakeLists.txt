emerald_define_executable(shallow_weaver_headless
    SOURCES 
        parse_parameters.h
        parse_parameters.cpp
        headless_main.cpp
    DEPENDENCIES 
        emerald::shallow_weaver
        cxxopts::cxxopts
    )

if (EMERALD_BUILD_GUIS)
    emerald_define_executable(shallow_weaver_gui
        SOURCES 
            parse_parameters.h
            parse_parameters.cpp
            main.cpp
        DEPENDENCIES 
            emerald::shallow_weaver
            emerald::simple_sim_viewer 
            emerald::geep_glfw 
            cxxopts::cxxopts
        )
endif()

