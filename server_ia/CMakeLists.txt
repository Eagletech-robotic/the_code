cmake_minimum_required(VERSION 3.10)
project(http_server C)
add_executable(http_server http_server.c)
target_link_libraries(http_server cjson)