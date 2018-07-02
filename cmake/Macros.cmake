#### Macro for adding source files and directories
macro (corah_add_sources)
  file (RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
  foreach (_src ${ARGN})
    if (_relPath)
      list (APPEND CORAH_SOURCE_FILES "${_relPath}/${_src}")
    else()
      list (APPEND CORAH_SOURCE_FILES "${_src}")
    endif()
  endforeach()
  if (_relPath)
    # propagate CORAH_SOURCE_FILES to parent directory
    set (CORAH_SOURCE_FILES ${CORAH_SOURCE_FILES} PARENT_SCOPE)
  endif()
endmacro()

macro (corah_add_subdirectories)
    file (RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        add_subdirectory(${_src})
    endforeach()
    if (_relPath)
        # propagate to parent directory
        set (CORAH_SOURCE_FILES ${CORAH_SOURCE_FILES} PARENT_SCOPE)
    endif()
endmacro()