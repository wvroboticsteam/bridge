ADD_EXECUTABLE(RandomSequenceTest
  RandomSequenceTest.cpp
)
TARGET_LINK_LIBRARIES(RandomSequenceTest
  SystemToolkit-Math
)

SET(FINAL_INSTALL_TARGETS
  RandomSequenceTest
)

INSTALL(TARGETS ${FINAL_INSTALL_TARGETS}
  DESTINATION bin/tests
)
