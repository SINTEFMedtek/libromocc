# Download and set up ZeroMQ

include(cmake/Externals.cmake)

ExternalProject_Add(zeromq
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/zeromq
        BINARY_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/zeromq
        GIT_REPOSITORY "https://github.com/zeromq/libzmq.git"
        GIT_TAG "v4.3.2"
        UPDATE_COMMAND ""
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DCMAKE_INSTALL_PREFIX:PATH=${ROMOCC_EXTERNAL_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_STATIC_LIBS=ON
            -DBUILD_PACKAGING=OFF
            -DBUILD_TESTING=OFF
            -DBUILD_NC_TESTS=OFF
            -DBUILD_CONFIG_TESTS=OFF
            -DINSTALL_HEADERS=ON
        )

list(APPEND ROMOCC_INCLUDE_DIRS ${ROMOCC_EXTERNAL_INSTALL_DIR}/include/)

if(WIN32)
    set(ZMQ_LIBRARY libzmq-v142-mt-4_3_2.lib libzmq-v142-mt-s-4_3_2.lib)
else(WIN32)
    set(ZMQ_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}zmq${CMAKE_SHARED_LIBRARY_SUFFIX})
endif(WIN32)

list(APPEND LIBRARIES ${ZMQ_LIBRARY})
list(APPEND ROMOCC_EXTERNAL_DEPENDENCIES zeromq)