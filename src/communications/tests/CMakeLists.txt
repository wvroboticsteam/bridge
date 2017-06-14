ADD_EXECUTABLE(SocketTest
  SocketTest.cpp
)
TARGET_LINK_LIBRARIES(SocketTest
  SystemToolkit-Communications
  SystemToolkit-Core
)

SET(FINAL_INSTALL_TARGETS
  SocketTest
)

INSTALL(TARGETS ${FINAL_INSTALL_TARGETS}
  DESTINATION bin/tests
)
