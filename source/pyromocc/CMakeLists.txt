cmake_minimum_required(VERSION 3.10)

project(pyromocc)

set(DEPENDENCIES romocc)

configure_file(${CMAKE_SOURCE_DIR}/cmake/ExternalPybind.cmake ${CMAKE_BINARY_DIR}/external/pybind11/CMakeLists.txt)

execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/pybind11"
        )

execute_process(COMMAND "${CMAKE_COMMAND}" --build .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/external/pybind11"
        )

# Add pybind11 makefile
add_subdirectory(${CMAKE_BINARY_DIR}/third-party/pybind11 ${CMAKE_BINARY_DIR}/third-party/pybind11 EXCLUDE_FROM_ALL)

include_directories(${CMAKE_BINARY_DIR}/third-party/pybind11/include)

pybind11_add_module(pyromocc source/pyromocc.cpp)
target_link_libraries(pyromocc PRIVATE pybind11::module ${DEPENDENCIES})
set_target_properties(pyromocc PROPERTIES VERSION ${VERSION}
        SOVERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set_target_properties(pyromocc PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pyromocc)

install(TARGETS pyromocc EXPORT romocc DESTINATION pyromocc)

message("-- Setup python module pyromocc")