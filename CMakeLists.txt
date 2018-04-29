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

if(MSVC)
  add_executable(stand_alone.exe ${STAND_ALONE_SOURCE_FILES})
else()
  add_executable(stand_alone.exe WIN32 ${STAND_ALONE_SOURCE_FILES})
endif()

target_link_libraries(stand_alone.exe
  WLANOptimizer
  Threads::Threads
  wlanapi
)

set(COMMON_SOURCE_FILES
        thirdparty/IncludeAsio.h
        thirdparty/asio.hpp
        tests/tools/Logger.cpp
        tests/tools/Logger.h
        tests/tools/PacketAllocator.cpp
        tests/tools/PacketAllocator.h
        tests/tools/TimeSync.cpp
        tests/tools/TimeSync.h
        tests/tools/Counter.h
        tests/tools/ccat.cpp
        tests/tools/ccat.h
        tests/tools/CCatCodec.cpp
        tests/tools/CCatCodec.h
        tests/tools/CCatCpp.h
        tests/tools/gf256.cpp
        tests/tools/gf256.h
        tests/tools/StrikeRegister.cpp
        tests/tools/StrikeRegister.h
        tests/Tools.cpp
        tests/Tools.h
        tests/Connection.cpp
        tests/Connection.h
        )

set(PEER2PEER_NO_GUI_SOURCE_FILES
        ${COMMON_SOURCE_FILES}
        tests/TestPeer2Peer_NoGUI.cpp
        )

add_executable(test_peer2peer_no_gui ${PEER2PEER_NO_GUI_SOURCE_FILES})
target_link_libraries(test_peer2peer_no_gui
  WLANOptimizer
  Threads::Threads
  wlanapi
)


# Peer2Peer test app with a weird IMGUI/DirectX11 UI
if(MSVC)

set(GUI_SOURCE_FILES
        thirdparty/imgui.cpp
        thirdparty/imgui_draw.cpp
        thirdparty/imgui.h
        thirdparty/imconfig.h
        thirdparty/imgui_internal.h
        thirdparty/imgui_impl_dx11.cpp
        thirdparty/imgui_impl_dx11.h
        thirdparty/stb_rect_pack.h
        thirdparty/stb_textedit.h
        thirdparty/stb_truetype.h
)

set(PEER2PEER_GUI_SOURCE_FILES
        ${COMMON_SOURCE_FILES}
        ${GUI_SOURCE_FILES}
        tests/TestPeer2Peer_GUI.cpp
        )

add_executable(test_peer2peer_gui ${PEER2PEER_GUI_SOURCE_FILES})
target_link_libraries(test_peer2peer_gui
  WLANOptimizer
  Threads::Threads
  wlanapi
  d3d11
  d3dcompiler
  dxgi
  imm32
)

# End if msvc
endif()
