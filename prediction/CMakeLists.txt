_get_cmake_dir_list(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(subdir ${SUBDIRS})
  add_subdirectory(${subdir})
endforeach()