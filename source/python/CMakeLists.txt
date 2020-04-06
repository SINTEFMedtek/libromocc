pybind11_add_module(pyromocc source/pyromocc.cpp)

target_link_libraries(pyromocc PRIVATE romocc)
set_target_properties(pyromocc PROPERTIES
                                PREFIX ""
                                LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/pyromocc")

configure_file(${PROJECT_SOURCE_DIR}/source/python/__init__.py.in
        ${PROJECT_BINARY_DIR}/pyromocc/__init__.py
        )

configure_file(${PROJECT_SOURCE_DIR}/source/python/setup.py.in
        ${PROJECT_BINARY_DIR}/pyromocc/setup.py
        )

configure_file(${PROJECT_SOURCE_DIR}/source/python/robot.py
        ${PROJECT_BINARY_DIR}/pyromocc/robot.py
        )

message("-- Setup python module pyromocc")
