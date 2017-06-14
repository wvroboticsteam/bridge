ADD_EXECUTABLE(ArrayTest
  ArrayTest.cpp
)
TARGET_LINK_LIBRARIES(ArrayTest
  SystemToolkit-Math
)

ADD_EXECUTABLE(MatrixTest
  MatrixTest.cpp
)
TARGET_LINK_LIBRARIES(MatrixTest
  SystemToolkit-Math
)

SET(FINAL_INSTALL_TARGETS
  ArrayTest
  MatrixTest
)

INSTALL(TARGETS ${FINAL_INSTALL_TARGETS}
  DESTINATION bin/tests
)
