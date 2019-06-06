# Download and set up ZeroMQ

include(cmake/Externals.cmake)

ExternalProject_Add(ursim
        PREFIX ${ROMOCC_EXTERNAL_BUILD_DIR}/ursim
        DOWNLOAD_DIR ${ROMOCC_EXTERNAL_BUILD_DIR}/ursim
        URL https://s3-eu-west-1.amazonaws.com/ur-support-site/16646/ursim-3.0.16471.tar.gz
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND cd ${ROMOCC_EXTERNAL_BUILD_DIR}/ursim/src/ursim/ && ./install.sh
        BUILD_IN_SOURCE 1
        INSTALL_COMMAND ""
)

list(APPEND ROMOCC_EXTERNAL_DEPENDENCIES ursim)

file(   WRITE ${ROMOCC_EXTERNAL_BUILD_DIR}/ursim/src/tmp/startSimulator.sh
        "${ROMOCC_EXTERNAL_BUILD_DIR}/ursim/src/ursim/start-ursim.sh")

file(   COPY ${ROMOCC_EXTERNAL_BUILD_DIR}/ursim/src/tmp/startSimulator.sh
        DESTINATION ${PROJECT_BINARY_DIR}/bin/
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)