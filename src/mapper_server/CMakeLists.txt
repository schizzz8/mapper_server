add_library(mapper_server_library SHARED
  call_mapper_trigger.h call_mapper_trigger.cpp
  pointcloud_publisher_trigger.h pointcloud_publisher_trigger.cpp
  surface_extractor.h surface_extractor.cpp
  sparse_grid.h sparse_grid.cpp
  mapper.h mapper.cpp
)

target_link_libraries(mapper_server_library 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
