ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(communications)
ADD_SUBDIRECTORY(types)
ADD_SUBDIRECTORY(math)

ADD_LIBRARY(ros-bridge
  RosBridge.cpp
  RobotCommands.cpp
)
TARGET_INCLUDE_DIRECTORIES(ros-bridge PUBLIC 
  $<BUILD_INTERFACE:${CATKIN_DEVEL_PREFIX}/include/bridge>
  $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/bridge>
)
TARGET_LINK_LIBRARIES(ros-bridge
  ${catkin_LIBRARIES}
)

ADD_EXECUTABLE(bridge-app
  bridge-app.cpp
)
TARGET_LINK_LIBRARIES(bridge-app
  ros-bridge
  SystemToolkit-Communications
  SystemToolkit-Core
)

SET(INSTALLED_HEADERS
  BridgeTypes.hpp
  RobotCommands.hpp
)

INSTALL(TARGETS ros-bridge bridge-app
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

INSTALL(FILES ${INSTALLED_HEADERS} DESTINATION "include/${PROJECT_NAME}")
