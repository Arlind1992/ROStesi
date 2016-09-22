add_executable(test_kinematics TestKinematics.cpp
                               ../src/kinematics_models/DifferentialDrive.cpp)
                               
target_link_libraries(test_kinematics ${catkin_LIBRARIES})
target_link_libraries(test_kinematics rrt_planner ${catkin_LIBRARIES})

add_executable(test_node TestNode.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
