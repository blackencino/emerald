add_library(emerald_simple_sim_viewer
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

    gl_camera.cpp
lines_draw_helper.cpp
mesh_draw_helper.cpp
particle_sphere_shaders.cpp
points_draw_helper.cpp
sim.cpp
slab_draw_helper.cpp
std_shaders.cpp
viewer.cpp
)

target_include_directories(emerald_simple_sim_viewer PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_simple_sim_viewer PUBLIC
                      emerald_geep_glfw
                      emerald_util
                      Alembic::Alembic
                      OpenEXR::OpenEXR)

add_subdirectory(tests)


