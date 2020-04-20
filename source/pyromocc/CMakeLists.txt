cmake_minimum_required(VERSION 3.10)

project(pyromocc)

set(DEPENDENCIES romocc)

add_subdirectory(third_party/pybind11)

pybind11_add_module(pyromocc source/pyromocc.cpp)
target_link_libraries(pyromocc PRIVATE ${DEPENDENCIES})
set_target_properties(pyromocc PROPERTIES VERSION ${VERSION}
        SOVERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set_target_properties(pyromocc PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pyromocc)

install(TARGETS pyromocc EXPORT romocc DESTINATION pyromocc)

message("-- Setup python module pyromocc")