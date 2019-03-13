if(NOT LIBSDRPLAY3_FOUND)
  pkg_check_modules (LIBSDRPLAY3_PKG libsdrplay3)
  find_path(LIBSDRPLAY3_INCLUDE_DIRS NAMES sdrplay_api.h
    PATHS
    ${LIBSDRPLAY3_PKG_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
  )

  find_library(LIBSDRPLAY3_LIBRARIES NAMES sdrplay_api
    PATHS
    ${LIBSDRPLAY3_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/lib64
    /usr/local/lib
    /usr/local/lib64
  )

if(LIBSDRPLAY3_INCLUDE_DIRS AND LIBSDRPLAY3_LIBRARIES)
  set(LIBSDRPLAY3_FOUND TRUE CACHE INTERNAL "libsdrplay3 found")
  message(STATUS "Found libsdrplay3: ${LIBSDRPLAY3_INCLUDE_DIRS}, ${LIBSDRPLAY3_LIBRARIES}")
else(LIBSDRPLAY3_INCLUDE_DIRS AND LIBSDRPLAY3_LIBRARIES)
  set(LIBSDRPLAY3_FOUND FALSE CACHE INTERNAL "libsdrplay3 found")
  message(STATUS "libsdrplay3 not found.")
endif(LIBSDRPLAY3_INCLUDE_DIRS AND LIBSDRPLAY3_LIBRARIES)

mark_as_advanced(LIBSDRPLAY3_LIBRARIES LIBSDRPLAY3_INCLUDE_DIRS)

endif(NOT LIBSDRPLAY3_FOUND)
