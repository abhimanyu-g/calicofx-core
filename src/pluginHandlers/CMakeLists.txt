target_sources(calicofx
  PUBLIC lv2Handler.cpp)

target_include_directories(calicofx
  PRIVATE ${LILV_INCLUDE_DIRS})

target_link_libraries(calicofx
	lilv-0
)
