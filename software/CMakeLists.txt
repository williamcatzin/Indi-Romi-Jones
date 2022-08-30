cmake_minimum_required(VERSION 3.0.0)
project(romi VERSION 0.1.0)

include(CTest)
enable_testing()

add_subdirectory(libraries/kobuki)

add_executable(romi main.c)

target_link_libraries(romi PUBLIC kobuki)

target_include_directories(kobuki PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/libraries/kobuki"
                          )

# ############## YDLIDAR SDK START#####################################
# #find ydlidar_sdk package
# find_package(ydlidar_sdk)
# #Include directories
# INCLUDE_DIRECTORIES(
#   ${YDLIDAR_SDK_INCLUDE_DIRS}
# )
# #link library directories
# link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

# #Link your project to ydlidar_sdk library.
# target_link_libraries(romi PUBLIC ${YDLIDAR_SDK_LIBRARIES})

# ############## YDLIDAR SDK END#####################################
############## YDLIDAR SDK START#####################################
#Include directories
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/YDLidar
  ${CMAKE_SOURCE_DIR}/YDLidar/src
  ${CMAKE_CURRENT_BINARY_DIR}/YDLidar
)
#Add YDLIDAR SDK sub project
add_subdirectory(YDLidar)

#Link your project to ydlidar_sdk library.
target_link_libraries(romi PUBLIC ydlidar_sdk)

############## YDLIDAR SDK END#####################################

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
