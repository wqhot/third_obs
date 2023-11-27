cmake_minimum_required(VERSION 3.0)
if (PLATFORM STREQUAL "ARM")
include(cross.cmake)
set(CMAKE_CROSSCOMPILING on)

else ()
SET(CMAKE_FIND_ROOT_PATH /usr)
endif ()

project(thirdobs)

if (BUILD_TYPE STREQUAL "RELEASE")
set(CMAKE_BUILD_TYPE "Release")
else (BUILD_TYPE STREQUAL "RELEASE")
set(CMAKE_BUILD_TYPE "Debug")
endif (BUILD_TYPE STREQUAL "RELEASE")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -pthread -fopenmp -fPIC -s -Wl,-Bsymbolic")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -pthread -fopenmp -fPIC -s -Wl,-Bsymbolic")
else(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0 -g -Wall -pthread -fopenmp -pg -fPIC -Wl,-Bsymbolic")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -Wall -pthread -fopenmp -pg -fPIC -Wl,-Bsymbolic")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")

set(CMAKE_CXX_STANDARD 17)

find_package(Git)
# 生成版本描述字符串类似 TAG-X-gHASH
execute_process(COMMAND ${GIT_EXECUTABLE} describe --abbrev=6 --dirty --always --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE  GIT_REPO_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
# 获取最新 commit 日期，YYYY-MM-DD
execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=short
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE  GIT_REPO_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
# 获取最新 commit Hash
execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%H
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE  GIT_REPO_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
# 写入头文件宏定义的版本字符串，日期和 Hash
file(WRITE ${PROJECT_BINARY_DIR}/repo_version.h
    "#define REPO_VERSION \"${GIT_REPO_VERSION}\"\n#define REPO_DATE \"${GIT_REPO_DATE}\"\n#define REPO_HASH \"${GIT_REPO_HASH}\"\n"
)

find_package(Threads)

include_directories(
        ${CMAKE_FIND_ROOT_PATH}/include
        ${PROJECT_SOURCE_DIR}/third_party/spdlog-1.x/include
        ${PROJECT_SOURCE_DIR}/third_party/eigen-3.4.0
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}
)

add_subdirectory(third_party/eigen-3.4.0)
add_subdirectory(third_party/spdlog-1.x)
add_subdirectory(third_obs)