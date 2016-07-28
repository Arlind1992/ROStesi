cmake_minimum_required(VERSION 2.8.3)

find_package(catkin)
project(rtt_planning)

catkin_package()

# ----------------------- INCLUDES ----------------------------

include_directories(include)

# ----------------------- LIBRARIES ---------------------------


# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ------------------------ MESSAGES ----------------------------


# ------------------------ BUILD -------------------------------


add_library(rtt_planner src/RTTPlanner.cpp)

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




