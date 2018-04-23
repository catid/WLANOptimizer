/** \file
    \brief Logging Module
    \copyright Copyright (c) 2017 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of Logger nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

/** \page Logger Logging Module

    Feature-rich portable C++ logging subsystem in 650 lines of code.
    The library is self-contained and is easy to incorporate into iOS,
    Android, Windows, Linux, and Mac projects.

    Quick Start guide:

        // Example logging channel
        static logger::Channel Logger("Decoder", logger::Level::Debug);

        // Log out a message
        Logger.Info("Got first recovery packet: ColumnStart=",
            metadata.ColumnStart, " SumCount=", metadata.SumCount);


    After trying several open-source loggers I ended up writing my own.
    I was mainly looking for a multithreaded logger supporting levels,
    and found a lot of buggy software that was hard to use or missing features.

    Primary Logger features:

    * Automatic initialization and shutdown just like 'cout'.
    * Low performance impact since the logging occurs on a background thread.
    * Automatically flushes message queue on shutdown.  (And it isn't buggy.)

    Additional extra features:

    * Logging Channel objects to sort messages by subsystem.
    * Logging Levels enabling runtime filtering of messages and faster offline analysis.
    * Serialization overloads in application code via LogStringize() overrides.
    * Supports runtime-configurable prefix tags for each Channel.

    On Android it uses __android_log_print().
    On other platforms it uses std::fwrite(stdout).
    On Windows it also uses OutputDebugStringA().

    Modify this function to change how it writes output:
        OutputWorker::Log()
*/

#include <sstream>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <list>
#include <thread>
#include <memory>


/// Define this to prevent the logger from hooking atexit().
/// The application must manually call logger::Stop().
//#define LOGGER_DISABLE_ATEXIT

/**
    This will flush before and after errors, so errors can never get lost.
    A disadvantage is that if the console is hung for some reason, then the
    logger call sites will hang too, and it will also cause error logging to
    run much slower than normal.  This is only recommended if software is
    written to the principle that errors are actually errors.
*/
//#define LOGGER_FLUSH_ERRORS

// Compiler-specific force inline keyword
#if defined(_MSC_VER)
    #define LOGGER_FORCE_INLINE inline __forceinline
#else
    #define LOGGER_FORCE_INLINE inline __attribute__((always_inline))
#endif

// Compiler-specific debug break
#if defined(_DEBUG) || defined(DEBUG)
    #define LOGGER_DEBUG
    #if defined(_WIN32)
        #define LOGGER_DEBUG_BREAK() __debugbreak()
    #else
        #define LOGGER_DEBUG_BREAK() __builtin_trap()
    #endif
    #define LOGGER_DEBUG_ASSERT(cond) { if (!(cond)) { LOGGER_DEBUG_BREAK(); } }
#else
    #define LOGGER_DEBUG_BREAK() do {} while (false);
    #define LOGGER_DEBUG_ASSERT(cond) do {} while (false);
#endif


namespace logger {


//------------------------------------------------------------------------------
// Configuration

/**
    Message queue overflow behavior:

    If messages are logged faster than we can write them to the console, then
    after the queue grows beyond kWorkQueueLimit messages it will drop data and
    write how many were dropped.  The logger prefers to lose messages rather
    than affect performance of the application software, by default.
    Errors bypass this limit and will force a Flush, so errors are always logged.

    When LOGGER_NEVER_DROP is defined, the Logger will Flush() and retry when
    the queue becomes full, so the application will stall rather than lose data.

    When LOGGER_NEVER_DROP is not defined, the application software can call
    Flush() after a complex operation to make sure that new log messages land.
*/
//#define LOGGER_NEVER_DROP

/// Tune the number of work queue items before we drop log messages on the floor.
/// If LOGGER_NEVER_DROP is defined this is when we block and flush.
static const size_t kWorkQueueLimit = 1024;


//------------------------------------------------------------------------------
// Level

/// Logging level
enum class Level
{
    Trace,   ///< Trace-level logging (off by default)
    Debug,   ///< Debug logging (on by default)
    Info,    ///< Info (normal) logging
    Warning, ///< Warnings
    Error,   ///< Errors
    Silent,  ///< Silent level (always off)

    Count    ///< For static assert
};

/// Stringize level values
const char* LevelToString(Level level);
char LevelToChar(Level level);


//------------------------------------------------------------------------------
// Buffer

struct LogStringBuffer
{
    const char* ChannelName;
    Level LogLevel;
    std::ostringstream LogStream;

    LogStringBuffer(const char* channel, Level level) :
        ChannelName(channel),
        LogLevel(level),
        LogStream()
    {
    }
};


//------------------------------------------------------------------------------
// Stringize

template<typename T>
LOGGER_FORCE_INLINE void LogStringize(LogStringBuffer& buffer, const T& first)
{
    buffer.LogStream << first;
}

/// Overrides for various types we want to handle specially:

template<>
LOGGER_FORCE_INLINE void LogStringize(LogStringBuffer& buffer, const bool& first)
{
    buffer.LogStream << (first ? "true" : "false");
}


//------------------------------------------------------------------------------
// OutputWorker

class OutputWorker
{
    /// Singleton pattern
    explicit OutputWorker();
    OutputWorker& operator=(const OutputWorker&) = delete;

public:
    /// Get singleton instance
    static OutputWorker& GetInstance();

    /// Manually start the output worker.  This is not required
    void Start();

    /// Manually stop the output worker.  This is also not required
    void Stop();

    /// Manually flush any items in the work queue and block here until the queue is flushed
    void Flush();

    /// Write a log message.  Use logger::Channel rather than calling this directly
    void Write(LogStringBuffer& buffer);

protected:
    struct QueuedMessage
    {
        Level LogLevel;
        const char* ChannelName;
        std::string Message;

        QueuedMessage(Level level, const char* channel, std::string& message)
            : LogLevel(level)
            , ChannelName(channel)
            , Message(message)
        {
        }
    };


    /// Lock preventing thread safety issues around Start() and Stop()
    mutable std::mutex StartStopLock;

    /// Lock protecting QueuePublic and QueueCondition
    mutable std::mutex QueueLock;

    /// Condition that indicates the thread should wake up
    std::condition_variable QueueCondition;

    /// List of messages that are being delivered to thread
    std::list<QueuedMessage> QueuePublic;

    /// Private list of messages being logged out by thread
    std::list<QueuedMessage> QueuePrivate;

#if !defined(LOGGER_NEVER_DROP)
    /// Number of log queue overruns
    std::atomic<int> Overrun = ATOMIC_VAR_INIT(0);
#endif // LOGGER_NEVER_DROP

    /// Has a flush been requested?
    std::atomic<bool> FlushRequested = ATOMIC_VAR_INIT(false);

    /// Condition variable to wait for Flush result
    std::condition_variable FlushCondition;

    /// Queue processing thread
    std::shared_ptr<std::thread> Thread;

    /// Should thread terminate?
    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(true);

#if defined(_WIN32)
    bool CachedIsDebuggerPresent = false;
    void* ThreadNativeHandle = nullptr;
#endif // _WIN32


    /// Queue processing loop
    void Loop();

    /// Internal log message dispatch function
    void Log(QueuedMessage& message);
};


//------------------------------------------------------------------------------
// Channel

/// Logging channel object: Each instance is given a channel name, and then the
/// logging channel is used to output log messages.
class Channel
{
public:
    /// Specify channel name and minimum output level (Level::Silent for silent)
    explicit Channel(const char* name, Level minLevel);


    /// Channel name
    const char* const ChannelName;

    /// Minimum channel log level
    const Level ChannelMinLevel;


    /// Should we log at this level?
    LOGGER_FORCE_INLINE bool ShouldLog(Level level) const
    {
        return level >= ChannelMinLevel;
    }


    /// Thread-safe get or set runtime-selected prefix for the logging channel
    std::string GetPrefix() const;
    void SetPrefix(const std::string& prefix);


    /// Log a message at a specified level
    template<typename... Args>
    LOGGER_FORCE_INLINE void Log(Level level, Args&&... args) const
    {
        if (ShouldLog(level))
            writeLogLine(level, std::forward<Args>(args)...);
    }

    /// Log an Error level message
    template<typename... Args>
    LOGGER_FORCE_INLINE void Error(Args&&... args) const
    {
#if !defined(LOGGER_NEVER_DROP) && defined(LOGGER_FLUSH_ERRORS)
        OutputWorker::GetInstance().Flush();
#endif // LOGGER_NEVER_DROP

        Log(Level::Error, std::forward<Args>(args)...);

#if !defined(LOGGER_NEVER_DROP) && defined(LOGGER_FLUSH_ERRORS)
        OutputWorker::GetInstance().Flush();
#endif // LOGGER_NEVER_DROP
    }

    /// Log a Warning level message
    template<typename... Args>
    LOGGER_FORCE_INLINE void Warning(Args&&... args) const
    {
        Log(Level::Warning, std::forward<Args>(args)...);
    }

    /// Log an Info level message
    template<typename... Args>
    LOGGER_FORCE_INLINE void Info(Args&&... args) const
    {
        Log(Level::Info, std::forward<Args>(args)...);
    }

    /// Log a Debug level message
    template<typename... Args>
    LOGGER_FORCE_INLINE void Debug(Args&&... args) const
    {
        Log(Level::Debug, std::forward<Args>(args)...);
    }

    /// Log a Trace level message
    template<typename... Args>
    LOGGER_FORCE_INLINE void Trace(Args&&... args) const
    {
        Log(Level::Trace, std::forward<Args>(args)...);
    }

protected:
    /// Runtime-selected channel prefix
    mutable std::mutex PrefixLock;
    std::string Prefix;


    template<typename T>
    LOGGER_FORCE_INLINE void writeLogBuffer(LogStringBuffer& buffer, T&& arg) const
    {
        LogStringize(buffer, arg);
    }

    template<typename T, typename... Args>
    LOGGER_FORCE_INLINE void writeLogBuffer(LogStringBuffer& buffer, T&& arg, Args&&... args) const
    {
        writeLogBuffer(buffer, arg);
        writeLogBuffer(buffer, args...);
    }

    template<typename... Args>
    LOGGER_FORCE_INLINE void writeLogLine(Level level, Args&&... args) const
    {
        LogStringBuffer buffer(ChannelName, level);
        writeLogBuffer(buffer, Prefix, args...);
        OutputWorker::GetInstance().Write(buffer);
    }
};

/// Flush log output to console
LOGGER_FORCE_INLINE void Flush()
{
    OutputWorker::GetInstance().Flush();
}

/// [Re-]Start the logger
LOGGER_FORCE_INLINE void Start()
{
    OutputWorker::GetInstance().Start();
}

/// Stop the logger
LOGGER_FORCE_INLINE void Stop()
{
    OutputWorker::GetInstance().Stop();
}


} // namespace logger
