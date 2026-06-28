set(_jie_max_module_sources
    ${CMAKE_CURRENT_LIST_DIR}/DR16/dev_dr16.c
    ${CMAKE_CURRENT_LIST_DIR}/game/game.c
    ${CMAKE_CURRENT_LIST_DIR}/LX824/lx824.c
    ${CMAKE_CURRENT_LIST_DIR}/Vofa/vofa.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_module_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/DR16
    ${CMAKE_CURRENT_LIST_DIR}/game
    ${CMAKE_CURRENT_LIST_DIR}/LX824
    ${CMAKE_CURRENT_LIST_DIR}/Vofa
)
