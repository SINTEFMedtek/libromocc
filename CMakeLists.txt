cmake_minimum_required(VERSION 3.1)

project(corah)

# Current version
set (CORAH_VERSION_MAJOR 0)
set (CORAH_VERSION_MINOR 0)
set (CORAH_VERSION_PATCH 1)

include(cmake/Macros.cmake)

#### Options
option(DOWNLOAD_AND_BUILD_QT5 "Download and build Qt 5.9. Turn this OFF if you want to use pre-built binaries you already have on your machine." OFF)

#### Set build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message("-- Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build types for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

#### C++ Standard
set(CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations") # Remove deprecated warnings GCC
endif()

#### Paths
set(CORAH_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} "source/corah/")
set(CORAH_CMAKE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

#### Module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/) # For finding the custom Find modules

#### Includes
set(CORAH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/source ${CMAKE_CURRENT_BINARY_DIR})

#### Setup all external depedencies
include(cmake/Requirements.cmake)
message("-- Qt version: ${Qt5Widgets_VERSION}")

#### Set include dirs
include_directories(${CORAH_INCLUDE_DIRS})
get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
    set(CORAH_INCLUDE_DIRS ${CORAH_INCLUDE_DIRS} PARENT_SCOPE)
endif()

#### Add all subdirs
corah_add_subdirectories(source/corah)
qt5_wrap_cpp(HEADERS_MOC ${CORAH_SOURCE_FILES})

message("-- CORAH SOURCE FILES: ${CORAH_SOURCE_FILES}")
message("-- CORAH HEADERS MOC: ${HEADERS_MOC}")
message("-- CORAH INCLUDE DIRS: ${CORAH_INCLUDE_DIRS}")
message("-- CORAH DEPENDENCIES: ${CORAH_EXTERNAL_DEPENDENCIES}")

#### Create library and executables
add_library(corah SHARED ${CORAH_SOURCE_FILES} ${HEADERS_MOC})
add_dependencies(corah ${CORAH_EXTERNAL_DEPENDENCIES})
include(GenerateExportHeader)
generate_export_header(corah EXPORT_FILE_NAME corahExport.hpp)

#### Link everything
message("-- LIBS: ${LIBRARIES} ${CORAH_SYSTEM_LIBRARIES}")
target_link_libraries(corah PUBLIC ${LIBRARIES} ${CORAH_SYSTEM_LIBRARIES} orocos-kdl)

add_executable(setup_robot_example source/corah/examples/setup_robot.cpp)
target_link_libraries(setup_robot_example corah)

#install(TARGETS corah DESTINATION lib RUNTIME DESTINATION bin)
#install(DIRECTORY ${PROJECT_BINARY_DIR} DESTINATION include)
include(cmake/InstallCorah.cmake)