# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_definitions("-fPIC") # Get rid of Qt error with position independent code
endif()

## External depedencies
include(cmake/ExternalEigen.cmake)
include(cmake/ExternalOrocos.cmake)
include(cmake/ExternalZeroMQ.cmake)

# Make sure project can find external includes and libaries
link_directories(${CORAH_EXTERNAL_INSTALL_DIR}/lib/)
list(APPEND INCLUDE_DIRS ${CORAH_EXTERNAL_INSTALL_DIR}/include)
