

add_executable( conn echo_client_conn.cpp)
target_link_libraries(conn socket++)

add_executable( sndto echo_client_sndto.cpp)
target_link_libraries(sndto socket++)


add_executable( srv echo_server.cpp)
target_link_libraries(srv socket++)