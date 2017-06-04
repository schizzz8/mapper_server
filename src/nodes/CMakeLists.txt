add_executable(mapper_server_node mapper_server_node.cpp)

target_link_libraries(mapper_server_node   
    mapper_server_library
    ${catkin_LIBRARIES}
  )  

add_executable(mapper_client_node mapper_client_node.cpp)

target_link_libraries(mapper_client_node   
    ${catkin_LIBRARIES}
  )  

