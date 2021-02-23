emerald_define_library(simple_sim_viewer
  SOURCES
    gl_camera.cpp
    lines_draw_helper.cpp
    mesh_draw_helper.cpp
    particle_sphere_shaders.cpp
    points_draw_helper.cpp
    sim.cpp
    slab_draw_helper.cpp
    std_shaders.cpp
    viewer.cpp
  HEADERS
    all.h
    foundation.h
    gl_camera.h
    lines_draw_helper.h
    mesh_draw_helper.h
    particle_sphere_shaders.h
    points_draw_helper.h
    sim.h
    slab_draw_helper.h
    std_shaders.h
    viewer.h
  DEPENDENCIES
    emerald::geep_glfw 
    emerald::util 
    OpenEXR::OpenEXR
    Alembic::Alembic
    fmt::fmt
  )

add_subdirectory(tests)


