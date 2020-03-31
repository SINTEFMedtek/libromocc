# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)

list(APPEND SYSTEM_LIBRARIES pthread)

## External depedencies
include(cmake/ExternalEigen.cmake)
include(cmake/ExternalOrocos.cmake)
include(cmake/ExternalZeroMQ.cmake)

if(ROMOCC_BUILD_URSIMULATOR)
    include(cmake/ExternalURSimulator.cmake)
endif()

if(ROMOCC_BUILD_PYTHON)
    find_package(Python 3 REQUIRED)
    list(APPEND ROMOCC_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

    include(cmake/ExternalPybind.cmake)
endif()

# Make sure project can find external includes and libaries
link_directories(${ROMOCC_EXTERNAL_INSTALL_DIR}/lib/)
list(APPEND INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include)
