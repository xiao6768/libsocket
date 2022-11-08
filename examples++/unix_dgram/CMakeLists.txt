

add_executable( unix_dgram_srv unix_dgram_server.cpp)
target_link_libraries(unix_dgram_srv socket++)

add_executable( unix_dgram_cl unix_dgram_client.cpp)
target_link_libraries(unix_dgram_cl socket++)

