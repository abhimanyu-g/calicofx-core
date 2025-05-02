target_sources(calicofx
 	PUBLIC main.cpp
	PUBLIC logger.cpp
	PUBLIC sessionMgr.cpp
	PUBLIC pw-client.cpp
	PUBLIC lv2PluginMgr.cpp
)

target_include_directories(calicofx
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc
	PRIVATE ${PIPEWIRE_INCLUDE_DIRS}
	PRIVATE ${LILV_INCLUDE_DIRS}
)

target_link_libraries(calicofx
	pipewire-0.3
	lilv-0
)
