set(_jie_max_arithmetic_sources
    ${CMAKE_CURRENT_LIST_DIR}/pid/pid_incremental/pid_incremental.c
    ${CMAKE_CURRENT_LIST_DIR}/pid/pid_location/pid_location.c
    ${CMAKE_CURRENT_LIST_DIR}/referee/crc_ref.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_arithmetic_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/pid/pid_incremental
    ${CMAKE_CURRENT_LIST_DIR}/pid/pid_location
    ${CMAKE_CURRENT_LIST_DIR}/referee
)
