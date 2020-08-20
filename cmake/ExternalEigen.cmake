# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(eigen
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/eigen
        BINARY_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/eigen/build
        URL "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz"
        INSTALL_DIR ${ROMOCC_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
        -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
        -DCMAKE_INSTALL_PREFIX:STRING=${ROMOCC_EXTERNAL_INSTALL_DIR}
        -DBUILD_TESTING:BOOL=OFF
        )

list(APPEND ROMOCC_INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include/eigen3/)
list(APPEND ROMOCC_EXTERNAL_DEPENDENCIES eigen)