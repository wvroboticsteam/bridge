SET(INSTALLED_HEADERS
  Array.hpp
  TypesTypes.hpp
  CommandLineArgument.hpp
)

ADD_LIBRARY(SystemToolkit-Types
  CommandLineArgument.cpp
)

INSTALL(TARGETS SystemToolkit-Types 
  ARCHIVE DESTINATION lib  
  LIBRARY DESTINATION lib
)

INSTALL(FILES ${INSTALLED_HEADERS}
  DESTINATION "include/${PROJECT_NAME}/types"
)

ADD_SUBDIRECTORY(tests)