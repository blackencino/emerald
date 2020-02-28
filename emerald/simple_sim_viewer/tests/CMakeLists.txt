add_executable(test_simple_mesh_sim test_simple_mesh_sim.cpp)
target_link_libraries(test_simple_mesh_sim PUBLIC
    emerald_simple_sim_viewer)

add_executable(test_simple_points_sim test_simple_points_sim.cpp)
target_link_libraries(test_simple_points_sim PUBLIC
    emerald_simple_sim_viewer)

add_executable(test_simple_slab_sim test_simple_slab_sim.cpp)
target_link_libraries(test_simple_slab_sim PUBLIC
    emerald_simple_sim_viewer)
