

add_executable( http http.cpp)
target_link_libraries(http socket++)

add_executable( http_2 http_2.cpp)
target_link_libraries(http_2 socket++)


add_executable( http_epoll http_epoll.cpp)
target_link_libraries(http_epoll socket++)