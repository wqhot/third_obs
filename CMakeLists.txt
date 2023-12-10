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
        ${PROJECT_SOURCE_DIR}/third_party/yaml-cpp-0.8.0/include
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}
)

add_subdirectory(third_party/eigen-3.4.0)
add_subdirectory(third_party/spdlog-1.x)
add_subdirectory(third_party/yaml-cpp-0.8.0)
add_subdirectory(softrender_point)
add_subdirectory(demos)

#准备的版本设置
SET(MAJOR_VERSION 1)
SET(MINOR_VERSION 0)
SET(PATCH_VERSION ${GIT_REPO_VERSION})

#说明要生成的是deb包
SET(CPACK_GENERATOR "DEB")

##############设置debian/control文件中的内容###############

#设置版本信息
SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")

#设置安装包的包名，打好的包将会是packagename-version-linux.deb，如果不设置，默认是工程名
set(CPACK_PACKAGE_NAME "softrender_point")

#设置程序名，就是程序安装后的名字
set(CPACK_DEBIAN_PACKAGE_NAME "softrender_point")

#设置架构
if (PLATFORM STREQUAL "ARM")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/aarch64-linux-gnu)
else ()
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/x86_64-linux-gnu)
endif ()

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

#设置依赖
if (PLATFORM STREQUAL "ARM")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libstdc++6")
else ()
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libstdc++6")
endif ()


#设置description
SET(CPACK_PACKAGE_DESCRIPTION "softrender_point")

#设置联系方式
SET(CPACK_PACKAGE_CONTACT "wqhot@outlook.com")

#设置维护人
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "wq")

include(CPack)

SET(LIBS_DIR ${CMAKE_BINARY_DIR}/softrender_point)
SET(INC_DIR ${CMAKE_SOURCE_DIR}/include)
SET(DEMO_DIR ${CMAKE_SOURCE_DIR}/demos)

#将库文件安装到/usr/lib下
INSTALL(FILES ${LIBS_DIR}/libsoftrender_point.so  DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(FILES ${INC_DIR}/camera_model.hh ${INC_DIR}/softrender_point.hh DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/softrender_point)
install(DIRECTORY ${DEMO_DIR} DESTINATION /usr/share/softrender_point)