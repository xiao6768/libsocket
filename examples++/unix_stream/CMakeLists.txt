

add_executable( unix_stream_srv unix_server_stream.cpp)
target_link_libraries(unix_stream_srv socket++)

add_executable( unix_stream_cl  unix_client_stream.cpp)
target_link_libraries(unix_stream_cl socket++)

