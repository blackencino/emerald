add_subdirectory(util)
add_subdirectory(z_index)
add_subdirectory(noise)
if (${EMERALD_BUILD_GUIS})
    add_subdirectory(geep_glfw)
    add_subdirectory(simple_sim_viewer)
endif()
add_subdirectory(shallow_weaver)
#add_subdirectory(wisp)
#add_subdirectory(sph_common)
#add_subdirectory(sph2d_box)



