cmake_minimum_required(VERSION 3.10)

project(pyromocc)

message("-- Setup python module pyromocc")

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

add_custom_target(copy_python_package
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/pyromocc ${ROMOCC_CMAKE_BINARY_DIR}/pyromocc/pyromocc
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/setup.py ${ROMOCC_CMAKE_BINARY_DIR}/pyromocc
        COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/README.md ${ROMOCC_CMAKE_BINARY_DIR}/pyromocc
        COMMENT "Copy python files")

add_custom_target(create_python_wheel
        COMMAND ${PYTHON_EXECUTABLE} setup.py bdist_wheel WORKING_DIRECTORY "${ROMOCC_CMAKE_BINARY_DIR}/pyromocc"
        COMMENT "Create python wheel"
)

add_dependencies(create_python_wheel copy_python_package pyromocc)