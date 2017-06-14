SET(INSTALLED_HEADERS
  Mutex.hpp
  Thread.hpp
  StaticThread.hpp
)

ADD_LIBRARY(SystemToolkit-Core SHARED
  Mutex.cpp
  StaticThread.cpp
)

TARGET_LINK_LIBRARIES(SystemToolkit-Core
  pthread
)

INSTALL(TARGETS SystemToolkit-Core 
  ARCHIVE DESTINATION lib  
  LIBRARY DESTINATION lib
)

INSTALL(FILES ${INSTALLED_HEADERS}
  DESTINATION "include/${PROJECT_NAME}/core/linux"
)
