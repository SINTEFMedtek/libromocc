# Setup all dependencies, both internal (have to be installed on the system)
# and external (downloaded and built automatically)

## Qt
if(DOWNLOAD_AND_BUILD_QT5)
    include(cmake/ExternalQt5.cmake)
    # Use Project Qt CMake files
    set(Qt5Core_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Core)
    set(Qt5Network_DIR ${PROJECT_SOURCE_DIR}/cmake/Qt5Network)
    find_package(Qt5Network REQUIRED PATHS ${PROJECT_SOURCE_DIR}/cmake/)
    list(APPEND LIBRARIES ${Qt5Core_LIBRARY})
    list(APPEND LIBRARIES ${Qt5Network_LIBRARY})
else(DOWNLOAD_AND_BUILD_QT5)
    find_package(Qt5 REQUIRED COMPONENTS Core Network)
    list(APPEND LIBRARIES Qt5::Core)
    list(APPEND LIBRARIES Qt5::Network)
endif(DOWNLOAD_AND_BUILD_QT5)

list(APPEND CORAH_INCLUDE_DIRS ${Qt5Core_INCLUDE_DIRS})
list(APPEND CORAH_INCLUDE_DIRS ${Qt5Network_INCLUDE_DIRS})
list(APPEND CORAH_EXTERNAL_DEPENDENCIES Qt5::Core Qt5::Network)

#set(CMAKE_AUTOMOC ON)
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_definitions("-fPIC") # Get rid of Qt error with position independent code
endif()

## External depedencies
include(cmake/ExternalEigen.cmake)
include(cmake/ExternalOrocos.cmake)

# Make sure project can find external includes and libaries
link_directories(${CORAH_EXTERNAL_INSTALL_DIR}/lib/)
list(APPEND INCLUDE_DIRS ${CORAH_EXTERNAL_INSTALL_DIR}/include)
