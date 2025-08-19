target_sources(calicofx
  PUBLIC lv2Handler.cpp
  PUBLIC worker.cpp)

target_include_directories(calicofx
  PRIVATE ${LILV_INCLUDE_DIRS}
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/worker.hpp")

target_link_libraries(calicofx
	lilv-0
)
