# Download and set up ZeroMQ

include(cmake/Externals.cmake)

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/pybind11)

FetchContent_Declare(pybind11
        GIT_REPOSITORY "https://github.com/pybind/pybind11.git"
        GIT_TAG "v2.4.3"
        )

FetchContent_GetProperties(pybind11)
message(${pybind11_SOURCE_DIR} " " ${pybind11_BINARY_DIR})

if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
endif()

romocc_add_subdirectories(${pybind11_SOURCE_DIR})
list(APPEND ROMOCC_INCLUDE_DIRS ${pybind11_SOURCE_DIR}/include/)
