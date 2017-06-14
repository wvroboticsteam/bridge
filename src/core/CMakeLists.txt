IF(UNIX)
  ADD_SUBDIRECTORY(linux)
ELSE(WIN32)
  ADD_SUBDIRECTORY(windows)
ENDIF()

SET(INSTALLED_HEADERS
  CoreTypes.hpp
  Mutex.hpp
  SystemCalls.hpp
  Thread.hpp
)

INSTALL(FILES ${INSTALLED_HEADERS}
  DESTINATION "include/${PROJECT_NAME}/core"
)
