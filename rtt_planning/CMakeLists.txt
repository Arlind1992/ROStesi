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

#Bfile(GLOB_RECURSE SOURCE_FILES
#B     src/*.cpp)
#B
#Badd_executable(rtt_planning ${SOURCE_FILES})

# ------------------------ INSTALLATION ------------------------

# Mark executables and/or libraries for installation
#install(TARGETS rele
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

# Install headers
#install(DIRECTORY include/rele
#        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#        PATTERN ".svn" EXCLUDE
#        )


# ------------------------ DOCUMENTATION ------------------------
#add_subdirectory(doc EXCLUDE_FROM_ALL)




