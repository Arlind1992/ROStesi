add_executable(test_kinematics TestKinematics.cpp
                               ../src/kinematics_models/DifferentialDrive.cpp)
                               
target_link_libraries(test_kinematics ${catkin_LIBRARIES})