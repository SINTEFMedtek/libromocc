# Set up swig
if(ROMOCC_BUILD_PYTHON)

    find_package(SWIG REQUIRED)
    include(${SWIG_USE_FILE})

    find_package(PythonLibs 3 REQUIRED)
    include_directories(${PYTHON_INCLUDE_DIRS})

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    set(CMAKE_SWIG_FLAGS "")

    foreach(FILE ${ROMOCC_SOURCE_FILES})
        if(${FILE} MATCHES "^.*hpp$" OR ${FILE} MATCHES "^.*h$")
            set(PYROMOCC_HEADER_INCLUDES "${PYROMOCC_HEADER_INCLUDES}#include \"${FILE}\"\n")
        endif()
    endforeach()

    foreach(FILE ${ROMOCC_PYTHON_INTERFACE_FILES})
        set(PYROMOCC_INTERFACE_INCLUDES "${PYROMOCC_INTERFACE_INCLUDES}%include \"${FILE}\"\n")
    endforeach()

    set(PYROMOCC_FILE "${PROJECT_BINARY_DIR}/romocc.i")
    configure_file("${PROJECT_SOURCE_DIR}/source/python/romocc.i" ${PYROMOCC_FILE})

    set_property(SOURCE ${PYROMOCC_FILE} PROPERTY GENERATED TRUE)
    set_property(SOURCE ${PYROMOCC_FILE} PROPERTY CPLUSPLUS ON)
    set(CMAKE_SWIG_OUTDIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/romocc/)
    file(MAKE_DIRECTORY ${CMAKE_SWIG_OUTDIR})

    swig_add_library(romocc
            LANGUAGE python
            OUTPUT_DIR ${CMAKE_BINARY_DIR}/python/${PROJECT_NAME}/
            SOURCES ${PYROMOCC_FILE})
    swig_link_libraries(romocc ${PYTHON_LIBRARIES})

    set_target_properties(_romocc PROPERTIES INSTALL_RPATH "$ORIGIN/../../lib")

    configure_file(
            "${PROJECT_SOURCE_DIR}/source/python/__init__.py"
            ${CMAKE_SWIG_OUTDIR}__init__.py
    )

endif()
