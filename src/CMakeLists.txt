target_sources(calicofx
  PUBLIC main.cpp
  PUBLIC pw-client.cpp
  PUBLIC logger.cpp)

target_include_directories(calicofx
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc
  PRIVATE ${PIPEWIRE_INCLUDE_DIRS})

 target_link_libraries(calicofx
   pipewire-0.3)
#   lilv-0)

add_subdirectory(pluginHandlers)
