cmake_minimum_required(VERSION 3.1)

project(romocc)

# Current version
set (VERSION_MAJOR 0)
set (VERSION_MINOR 0)
set (VERSION_PATCH 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

include(cmake/Macros.cmake)

#### Options
option(ROMOCC_BUILD_TESTS "Build tests." ON)
option(ROMOCC_BUILD_EXAMPLES "Build examples." ON)
option(ROMOCC_BUILD_URSIMULATOR "Download and install UR Simulator" ON)

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

## Set build folders
# First for the generic no-config case (e.g. with mingw)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
# Second, for multi-config builds (e.g. msvc)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    # Force libs and bins to go into bin and lib directory instead of Release/Debug
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/bin )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

#### Paths
set(ROMOCC_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} "source/romocc/")
set(ROMOCC_CMAKE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

#### Module path
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

#### Includes
set(ROMOCC_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/source/ ${CMAKE_CURRENT_BINARY_DIR})

#### Setup all external depedencies
include(cmake/Requirements.cmake)

#### Set include dirs
include_directories(${ROMOCC_INCLUDE_DIRS})
get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
    set(ROMOCC_INCLUDE_DIRS ${ROMOCC_INCLUDE_DIRS} PARENT_SCOPE)
endif()

#### Add all subdirs
romocc_add_subdirectories(source/romocc)

message("-- ROMOCC SOURCE FILES: ${ROMOCC_SOURCE_FILES}")
message("-- ROMOCC INCLUDE DIRS: ${ROMOCC_INCLUDE_DIRS}")
message("-- ROMOCC DEPENDENCIES: ${ROMOCC_EXTERNAL_DEPENDENCIES}")

#### Create library and executables
add_library(romocc SHARED ${ROMOCC_SOURCE_FILES} ${HEADERS_MOC})
add_dependencies(romocc ${ROMOCC_EXTERNAL_DEPENDENCIES})
include(GenerateExportHeader)
generate_export_header(romocc EXPORT_FILE_NAME romoccExport.hpp)

#### Link everything
message("-- LIBS: ${LIBRARIES} ${ROMOCC_SYSTEM_LIBRARIES}")
target_link_libraries(romocc PUBLIC ${LIBRARIES} ${ROMOCC_SYSTEM_LIBRARIES} pthread)

## Build examples
if(ROMOCC_BUILD_EXAMPLES)
    romocc_add_subdirectories(examples)
endif()

## Build test executable
if(ROMOCC_BUILD_TESTS)
    add_executable(testROMOCC ${ROMOCC_TEST_SOURCE_FILES})
    target_link_libraries(testROMOCC romocc)
endif()

include(cmake/InstallRomocc.cmake)