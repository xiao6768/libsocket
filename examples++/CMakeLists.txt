

add_executable( dgram_over_stream dgram_over_stream.cpp)
target_link_libraries(dgram_over_stream socket++)

add_executable( framing framing.cpp)
target_link_libraries(framing socket++)


add_executable( unix_dgram_syslogclient unix_dgram_syslogclient.cpp)
target_link_libraries(unix_dgram_syslogclient socket++)


add_subdirectory(echo)
add_subdirectory(http_examples)
add_subdirectory(simple_tcp)
add_subdirectory(unix_dgram)
add_subdirectory(unix_stream)