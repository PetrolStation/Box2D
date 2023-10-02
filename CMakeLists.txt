cmake_minimum_required(VERSION 3.16)

PA_NEW(Box2D)

PA_INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/box2d/include
)

PA_LINK_LIBRARIES( box2d )

File(GLOB_RECURSE SOURCE_FILES src/*.cpp )
File(GLOB_RECURSE HEADER_FILES src/*.h   )

pa_add_source_files(
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

add_subdirectory(deps)
