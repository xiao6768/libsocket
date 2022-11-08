

add_executable( simple_tcp_srv server.cpp)
target_link_libraries(simple_tcp_srv socket++)

add_executable( simple_tcp_cl client.cpp)
target_link_libraries(simple_tcp_cl socket++)

