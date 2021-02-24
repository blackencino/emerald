emerald_define_executable(wisp_headless
    SOURCES 
        parse_parameters.h
        parse_parameters.cpp
        headless_main.cpp
    DEPENDENCIES 
        emerald::wisp
        cxxopts::cxxopts
    )


if (EMERALD_BUILD_GUIS)
    emerald_define_executable(wisp_gui
        SOURCES 
            parse_parameters.h
            parse_parameters.cpp
            main.cpp
        DEPENDENCIES 
            emerald::wisp
            emerald::simple_sim_viewer 
            emerald::geep_glfw 
            cxxopts::cxxopts
        )
endif()

