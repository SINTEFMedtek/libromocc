# This will install romocc binaries, libraries and necessary include files to the path given by CMAKE_INSTALL_PREFIX

# Install romocc library
if(WIN32)
# DLL should be in binary folder
install(TARGETS romocc
	DESTINATION bin
)
else()
install(TARGETS romocc
	DESTINATION lib
)
endif()

# Examples are installed in the macro fast_add_example

# Install dependency libraries
install(FILES ${PROJECT_BINARY_DIR}/romoccExport.hpp
    DESTINATION include
)

if(WIN32)
	file(GLOB DLLs ${PROJECT_BINARY_DIR}/bin/*.dll)
	install(FILES ${DLLs}
		DESTINATION bin
	)
	file(GLOB DLLs ${PROJECT_BINARY_DIR}/lib/*.lib)
	install(FILES ${DLLs}
		DESTINATION lib
	)
elseif(APPLE)
	file(GLOB SOs ${PROJECT_BINARY_DIR}/lib/*.dylib)
	install(FILES ${SOs}
        DESTINATION lib
    )
else()
	file(GLOB SOs ${PROJECT_BINARY_DIR}/lib/*.so*)
	install(FILES ${SOs}
		DESTINATION lib
	)
endif()

# Install Qt plugins
install(DIRECTORY ${PROJECT_BINARY_DIR}/plugins/
    DESTINATION QtPlugins/
)

## Install qt.conf
install(FILES ${PROJECT_SOURCE_DIR}/cmake/InstallFiles/qt.conf
    DESTINATION bin
)

# Install headers
install(DIRECTORY ${ROMOCC_SOURCE_DIR}
	DESTINATION include/romocc/
	FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY ${ROMOCC_SOURCE_DIR}
	DESTINATION include/romocc/
	FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY ${PROJECT_BINARY_DIR}/include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY ${PROJECT_BINARY_DIR}/include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY ${PROJECT_BINARY_DIR}/include/
    DESTINATION include/
    FILES_MATCHING REGEX "/[^.]+$" # Files with no extension
)
install(DIRECTORY ${PROJECT_BINARY_DIR}/include/
        DESTINATION include/
        FILES_MATCHING PATTERN "*.inl"
)