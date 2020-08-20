# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(eigen
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/eigen
        BINARY_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/eigen/build
        URL "https://gitlab.com/libeigen/eigen/-/archive/603e213d13311af286c8c1abd4ea14a8bd3d204e/eigen-603e213d13311af286c8c1abd4ea14a8bd3d204e.tar.gz"
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