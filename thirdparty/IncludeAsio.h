#pragma once

#define ASIO_STANDALONE 1
#define ASIO_NO_DEPRECATED 1
#define ASIO_HEADER_ONLY 1

// Enable all Asio WinAPI features and fix warning message
#if !defined(_WIN32_WINNT)
    #define TONK_DEFINED_WIN32_WINNT
    #define _WIN32_WINNT 0x0501
#endif // _WIN32_WINNT

#include "asio.hpp"

#if defined(TONK_DEFINED_WIN32_WINNT)
    #undef _WIN32_WINNT
#endif // TONK_DEFINED_WIN32_WINNT
