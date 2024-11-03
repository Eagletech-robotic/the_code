cmake_minimum_required(VERSION 3.10)
project(http_server C)
add_executable(http_server
	http_server.c
	../helloworld2/Core/Src/eaglesteward/falcon.c
	../helloworld2/Core/Src/robotic/carre.c
	../helloworld2/Core/Src/robotic/inertial.c
	../helloworld2/Core/Src/robotic/pid.c
)
include_directories(../helloworld2/Core/Inc/)
target_link_libraries(http_server cjson m)
