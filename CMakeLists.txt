cmake_minimum_required(VERSION 3.8)
project(glim_scan_saver_ext)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default build type is release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################
# Dependencies #
################
find_package(glim REQUIRED)

# Happly for ply file writing
include(FetchContent)
FetchContent_Declare(
  happly
  GIT_REPOSITORY https://github.com/nmwsharp/happly.git
  GIT_TAG 8a606309daaa680eee495c8279feb0b704148f4a
)
FetchContent_MakeAvailable(happly)
message(STATUS ${happly_SOURCE_DIR})
######################
# Add Plugin Library #
######################
add_library(${PROJECT_NAME} SHARED src/glim_scan_saver.cpp)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${happly_SOURCE_DIR}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC glim::glim)

##############################
# Install the plugin library #
##############################
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
