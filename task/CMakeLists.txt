set(_jie_max_task_sources
    ${CMAKE_CURRENT_LIST_DIR}/dr16_task/dr16_task.c
    ${CMAKE_CURRENT_LIST_DIR}/game_task/game_task.c
    ${CMAKE_CURRENT_LIST_DIR}/lx824_task/lx824_task.c
    ${CMAKE_CURRENT_LIST_DIR}/vofa_task/vofa_task.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_task_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/dr16_task
    ${CMAKE_CURRENT_LIST_DIR}/game_task
    ${CMAKE_CURRENT_LIST_DIR}/lx824_task
    ${CMAKE_CURRENT_LIST_DIR}/vofa_task
)
