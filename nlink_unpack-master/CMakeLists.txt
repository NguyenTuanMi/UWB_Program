cmake_minimum_required(VERSION 3.10.0)

set(TARGET_NAME nlink_unpack_com)

project(${TARGET_NAME} VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(
  ${TARGET_NAME}
  UWB_Sender.c # Insert source code in here
  src/nlink_utils.c
  src/nlink_linktrack_nodeframe0.c
  src/nlink_linktrack_nodeframe1.c
  src/nlink_linktrack_nodeframe2.c
  src/nlink_linktrack_nodeframe3.c
  src/nlink_linktrack_aoa_nodeframe0.c
  src/nlink_linktrack_tagframe0.c
  src/nlink_linktrack_anchorframe0.c
  src/nlink_tofsense_frame0.c
  src/nlink_tofsensem_frame0.c)

include_directories(
  "src/"
)

target_link_libraries(${TARGET_NAME} ws2_32.lib)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
