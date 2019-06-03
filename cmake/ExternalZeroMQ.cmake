# Download and set up Eigen

include(cmake/Externals.cmake)

ExternalProject_Add(ZMQ
        PREFIX ${CORAH_EXTERNAL_BUILD_DIR}/ZMQ
        GIT_REPOSITORY "https://github.com/zeromq/libzmq.git"
        GIT_TAG "latest_release"
        UPDATE_COMMAND ""
        INSTALL_DIR ${CORAH_EXTERNAL_INSTALL_DIR}
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_INSTALL_PREFIX:PATH=${CORAH_EXTERNAL_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_STATIC_LIBS=ON
            -DBUILD_PACKAGING=OFF
            -DBUILD_TESTING=OFF
            -DBUILD_NC_TESTS=OFF
            -BUILD_CONFIG_TESTS=OFF
            -DINSTALL_HEADERS=ON
        )

list(APPEND CORAH_INCLUDE_DIRS ${CORAH_EXTERNAL_INSTALL_DIR}/include/)
list(APPEND LIBRARIES zmq)
list(APPEND CORAH_EXTERNAL_DEPENDENCIES ZMQ)