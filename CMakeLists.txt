cmake_minimum_required(VERSION 3.5)
project(WLANOptimizer)

if(UNIX)
    SET(CMAKE_SYSTEM_NAME Windows)
    SET(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++-posix)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++ -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive")
endif()

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

set(CMAKE_CXX_STANDARD 14)

set(LIB_SOURCE_FILES
        WLANOptimizer.cpp
        WLANOptimizer.h
        )

set(STAND_ALONE_SOURCE_FILES
        tests/StandAlone.cpp
        )

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
else()
    # Remove -Wno-unused-local-typedef after updating Asio code
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-local-typedef")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -march=native")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()

add_library(WLANOptimizer STATIC ${LIB_SOURCE_FILES})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(stand_alone.exe WIN32 ${STAND_ALONE_SOURCE_FILES})
target_link_libraries(stand_alone.exe WLANOptimizer Threads::Threads wlanapi)
