project(frame_stream)

set(SRC_LISTS frame_stream.c frame.c
)




add_library(${PROJECT_NAME} ${SRC_LISTS})


set(APP_NAME test_frame_stream)
add_executable(${APP_NAME} main.c ${SRC_LISTS})
target_link_libraries(${APP_NAME} socket)