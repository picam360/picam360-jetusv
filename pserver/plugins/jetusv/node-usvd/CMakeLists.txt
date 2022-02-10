#snipet cmake-js compile -G \"Visual Studio 15 2017 Win64\"
#snipet cmake-js compile -G \"Unix Makefiles\"

cmake_minimum_required(VERSION 3.9)
cmake_policy(SET CMP0042 NEW)
set (CMAKE_CXX_STANDARD 11)

project (node-usvd)
include_directories(${CMAKE_JS_INC})
set(SOURCE_FILES "usvd-napi.cpp" "usvd.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${CMAKE_JS_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "" SUFFIX ".node")
target_link_libraries(${PROJECT_NAME} ${CMAKE_JS_LIB})

target_include_directories(${PROJECT_NAME} PRIVATE
	./
	node_modules/node-addon-api/
)


# Define NAPI_VERSION
add_definitions(-DNAPI_VERSION=4)
