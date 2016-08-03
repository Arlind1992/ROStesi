cmake_minimum_required(VERSION 2.8.3)

find_package(catkin)
project(rtt_planning)

catkin_package()

# ----------------------- INCLUDES ----------------------------

include_directories(include)

# ----------------------- LIBRARIES ---------------------------

find_package(Eigen3 REQUIRED)
find_package(Boost 1.53 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})



# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ------------------------ MESSAGES ----------------------------


# ------------------------ BUILD -------------------------------


add_library(rtt_planner src/RTTPlanner.cpp
                        src/kinematics_models/DifferentialDrive.cpp)
                        
                        
# ------------------------ TESTS -------------------------------  
add_subdirectory(test)                      

# ------------------------ INSTALLATION ------------------------

# Mark executables and/or libraries for installation
install(TARGETS rtt_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Install headers
install(DIRECTORY include
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


# ------------------------ DOCUMENTATION ------------------------
#add_subdirectory(doc EXCLUDE_FROM_ALL)




