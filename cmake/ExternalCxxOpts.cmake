# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(cxxopts
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/cxxopts
        BINARY_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/cxxopts/build
        URL "https://github.com/jarro2783/cxxopts/archive/refs/tags/v3.1.1.tar.gz"
        INSTALL_DIR ${ROMOCC_EXTERNAL_INSTALL_DIR}
        CMAKE_CACHE_ARGS
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
        -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
        -DCMAKE_INSTALL_PREFIX:STRING=${ROMOCC_EXTERNAL_INSTALL_DIR}
        -DBUILD_TESTING:BOOL=OFF
        )

list(APPEND ROMOCC_INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include/cxxopts/)
list(APPEND ROMOCC_EXTERNAL_DEPENDENCIES cxxopts)