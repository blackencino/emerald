add_library(emerald_util
    all.h
    assert.h
    flicks.h
    format.h
    foundation.h
    functions.h
    imath_vec_ordering.h
    murmur3.h
    random.h
    storage_traits.h
    timer.h
    vector_util.h

    assert.cpp
    murmur3.cpp
    random.cpp
    vector_util.cpp
)

target_include_directories(emerald_util PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_util
    openexr::openexr
    Alembic::Alembic
    fmt::fmt)

add_subdirectory(tests)
