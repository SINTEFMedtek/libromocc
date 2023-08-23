# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    list(APPEND SYSTEM_LIBRARIES pthread)
endif()

## External depedencies
include(cmake/ExternalEigen.cmake)
include(cmake/ExternalOrocos.cmake)
include(cmake/ExternalZeroMQ.cmake)
include(cmake/ExternalCxxOpts.cmake)

if(ROMOCC_BUILD_URSIMULATOR)
    include(cmake/ExternalURSimulator.cmake)
endif()

if(ROMOCC_BUILD_PYTHON_BINDINGS)
    find_package(PythonLibs 3 REQUIRED)
endif()

# Make sure project can find external includes and libaries
link_directories(${ROMOCC_EXTERNAL_INSTALL_DIR}/lib/)
list(APPEND INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include)
