# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(orocos
        PREFIX ${CORAH_EXTERNAL_BUILD_DIR}/orocos
        BINARY_DIR ${CORAH_EXTERNAL_BUILD_DIR}/orocos/build
        GIT_REPOSITORY "https://github.com/orocos/orocos_kinematics_dynamics.git"
        GIT_TAG "master"
        INSTALL_DIR ${CORAH_EXTERNAL_INSTALL_DIR}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/orocos_kdl
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${CORAH_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
        )

list(APPEND CORAH_INCLUDE_DIRS ${CORAH_EXTERNAL_INSTALL_DIR}/include/)
list(APPEND CORAH_EXTERNAL_DEPENDENCIES orocos)