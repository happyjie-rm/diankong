set(_jie_max_component_sources
    ${CMAKE_CURRENT_LIST_DIR}/comp_utils.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_component_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
