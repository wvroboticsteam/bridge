SET(INSTALLED_HEADERS
  CommunicationsTypes.hpp
  Socket.hpp
  SocketServer.hpp
  SocketClient.hpp
)

ADD_LIBRARY(SystemToolkit-Communications SHARED
  Socket.cpp
  SocketServer.cpp
  SocketClient.cpp
)

TARGET_LINK_LIBRARIES(SystemToolkit-Communications
  SystemToolkit-Math
  pthread
)

INSTALL(TARGETS SystemToolkit-Communications 
  ARCHIVE DESTINATION lib  
  LIBRARY DESTINATION lib
)

INSTALL(FILES ${INSTALLED_HEADERS}
  DESTINATION "include/${PROJECT_NAME}/communications"
)

IF(BUILD_TESTS_ENABLED)
  ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTS_ENABLED)