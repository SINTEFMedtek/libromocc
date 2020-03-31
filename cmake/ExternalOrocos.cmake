# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(orocos
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/orocos
        BINARY_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/orocos/build
        GIT_REPOSITORY "https://github.com/orocos/orocos_kinematics_dynamics.git"
        GIT_TAG "1ae45bb2e0821586e74047b01f7cb48d913204f4"
        INSTALL_DIR ${ROMOCC_EXTERNAL_INSTALL_DIR}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/orocos_kdl
        CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF
            -DCMAKE_INSTALL_MESSAGE:BOOL=LAZY
            -DCMAKE_INSTALL_PREFIX:PATH=${ROMOCC_EXTERNAL_INSTALL_DIR}
            -DBUILD_TESTING:BOOL=OFF
        )

add_dependencies(orocos eigen)

list(APPEND ROMOCC_INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include/)
list(APPEND LIBRARIES orocos-kdl)
list(APPEND ROMOCC_EXTERNAL_DEPENDENCIES orocos)