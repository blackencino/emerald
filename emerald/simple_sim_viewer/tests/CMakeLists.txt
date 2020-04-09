add_executable(test_simple_mesh_sim test_simple_mesh_sim.cpp)
target_link_libraries(test_simple_mesh_sim PUBLIC
    emerald_simple_sim_viewer)

add_executable(test_simple_points_sim test_simple_points_sim.cpp)
target_link_libraries(test_simple_points_sim PUBLIC
    emerald_simple_sim_viewer)

add_executable(test_simple_slab_sim test_simple_slab_sim.cpp)
target_link_libraries(test_simple_slab_sim PUBLIC
    emerald_simple_sim_viewer)

add_library(multi_scale_draw multi_scale_draw.h multi_scale_draw.cpp)
target_link_libraries(multi_scale_draw PUBLIC
                      emerald_simple_sim_viewer
                      TBB::TBB)

add_executable(test_multi_particles_sim test_multi_particles_sim.cpp)
target_link_libraries(test_multi_particles_sim PUBLIC
    multi_scale_draw)
