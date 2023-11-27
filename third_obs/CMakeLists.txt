cmake_minimum_required(VERSION 3.0)

PROJECT(third_obs)

file(GLOB THIRD_OBS_SRCS *.cpp *.c)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library( ${PROJECT_NAME} ${THIRD_OBS_SRCS})
target_link_libraries(
    ${PROJECT_NAME}
    spdlog::spdlog
)