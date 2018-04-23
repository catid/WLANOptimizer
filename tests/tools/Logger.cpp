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

#include "Logger.h"

#if !defined(ANDROID)
    #include <cstdio> // fwrite, stdout
#endif

#if defined(_WIN32)
    #if !defined(NOMINMAX)
        #define NOMINMAX
    #endif
    #include <windows.h> // OutputDebugStringA
#endif


namespace logger {


//------------------------------------------------------------------------------
// Level

static const char* kLevelStrings[(int)Level::Count] = {
    "Trace", "Debug", "Info", "Warning", "Error", "Silent"
};

const char* LevelToString(Level level)
{
    static_assert((int)Level::Count == 6, "Update this switch");
    LOGGER_DEBUG_ASSERT((int)level >= 0 && level < Level::Count);
    return kLevelStrings[(int)level];
}

static const char kLevelChars[(int)Level::Count] = {
    't', 'd', 'I', 'W', '!', '?'
};

char LevelToChar(Level level)
{
    static_assert((int)Level::Count == 6, "Update this switch");
    LOGGER_DEBUG_ASSERT((int)level >= 0 && level < Level::Count);
    return kLevelChars[(int)level];
}


//------------------------------------------------------------------------------
// OutputWorker

OutputWorker& OutputWorker::GetInstance()
{
    // Fix crash on shutdown due to MSVCRT bug prior to version 2015
    static std::once_flag m_singletonFlag;
    static std::unique_ptr<OutputWorker> instance;

    std::call_once(m_singletonFlag, []()
    {
        instance.reset(new OutputWorker);
    });

    return *instance.get();
}

#if !defined(LOGGER_DISABLE_ATEXIT)
static void AtExitWrapper()
{
    OutputWorker::GetInstance().Stop();
}
#endif // LOGGER_DISABLE_ATEXIT

OutputWorker::OutputWorker()
{
    Start();

#if !defined(LOGGER_DISABLE_ATEXIT)
    // Register an atexit() callback so we do not need manual shutdown in app code
    // Application code can still manually shutdown by calling OutputWorker::Stop()
    std::atexit(AtExitWrapper);
#endif // LOGGER_DISABLE_ATEXIT
}

void OutputWorker::Start()
{
    std::lock_guard<std::mutex> locker(StartStopLock);

    // If thread is already spinning:
    if (Thread)
        return;

#if defined(_WIN32)
    CachedIsDebuggerPresent = (::IsDebuggerPresent() != FALSE);
#endif // _WIN32

    QueuePublic.clear();
    QueuePrivate.clear();

#if !defined(LOGGER_NEVER_DROP)
    Overrun = 0;
#endif // LOGGER_NEVER_DROP
    FlushRequested = false;
    Terminated = false;
    Thread = std::make_shared<std::thread>(&OutputWorker::Loop, this);

#if defined(_WIN32)
    ThreadNativeHandle = Thread->native_handle();
#endif // _WIN32
}

void OutputWorker::Stop()
{
    std::lock_guard<std::mutex> locker(StartStopLock);

    if (Thread)
    {
#if defined(_WIN32)
        // If thread was force-terminated by something:
        if (WAIT_OBJECT_0 == ::WaitForSingleObject(ThreadNativeHandle, 0))
        {
            LOGGER_DEBUG_BREAK(); // Application force-terminated the thread
            return; // Abort shutdown...
        }
#endif // _WIN32

        Flush();

        Terminated = true;

        // Make sure that queue notification happens after termination flag is set
        {
            std::unique_lock<std::mutex> qlocker(QueueLock);
            QueueCondition.notify_all();
        }

        try
        {
            if (Thread->joinable())
                Thread->join();
        }
        catch (std::system_error& /*err*/)
        {
        }
    }
    Thread = nullptr;

    // Make sure that concurrent Flush() calls do not block
    {
        std::unique_lock<std::mutex> qlocker(QueueLock);
        FlushCondition.notify_all();
    }
}

void OutputWorker::Write(LogStringBuffer& buffer)
{
    std::string str = buffer.LogStream.str();

#if defined(_WIN32)
    // If a debugger is present:
    if (CachedIsDebuggerPresent)
    {
        // Log all messages immediately to the Visual Studio Output Window
        // to allow logging while single-stepping in a debugger.
        ::OutputDebugStringA((str + "\n").c_str());
    }
#endif // _WIN32

#if defined(LOGGER_NEVER_DROP)
    for (;; Flush())
    {
        std::lock_guard<std::mutex> locker(QueueLock);

        if (QueuePublic.size() >= kWorkQueueLimit)
        {
            continue;
        }
        else
        {
            QueuePublic.emplace_back(buffer.LogLevel, buffer.ChannelName, str);
            break;
        }
    }
#else // LOGGER_NEVER_DROP
    {
        std::lock_guard<std::mutex> locker(QueueLock);

        if (QueuePublic.size() >= kWorkQueueLimit)
            Overrun++;
        else
            QueuePublic.emplace_back(buffer.LogLevel, buffer.ChannelName, str);
    }
#endif // LOGGER_NEVER_DROP

    QueueCondition.notify_all();
}

void OutputWorker::Loop()
{
    while (!Terminated)
    {
        int overrun = 0;
        bool flushRequested = false;
        {
            // unique_lock used since QueueCondition.wait requires it
            std::unique_lock<std::mutex> locker(QueueLock);

            if (QueuePublic.empty() && !FlushRequested && !Terminated)
                QueueCondition.wait(locker);

            std::swap(QueuePublic, QueuePrivate);

#if !defined(LOGGER_NEVER_DROP)
            overrun = Overrun;
            Overrun = 0;
#endif // LOGGER_NEVER_DROP

            flushRequested = FlushRequested;
            FlushRequested = false;
        }

        for (auto& log : QueuePrivate)
            Log(log);

        // Handle log message overrun
        if (overrun > 0)
        {
            std::ostringstream oss;
            oss << "Queue overrun. Lost " << overrun << " log messages";
            std::string str = oss.str();
            QueuedMessage qm(Level::Error, "Logger", str);
            Log(qm);
        }

        QueuePrivate.clear();

        if (flushRequested)
            FlushCondition.notify_all();
    }

    // Log out that logger is terminating
    std::string terminatingMessage = "Terminating";
    QueuedMessage qm(Level::Info, "Logger", terminatingMessage);
    Log(qm);
}

void OutputWorker::Log(QueuedMessage& message)
{
    std::ostringstream ss;
    ss << '{' << LevelToChar(message.LogLevel) << '-' << message.ChannelName << "} " << message.Message;

#if defined(ANDROID)
    std::string fmtstr = ss.str();
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "%s", fmtstr.c_str());
#else // ANDROID
    ss << std::endl;
    std::string fmtstr = ss.str();
    fwrite(fmtstr.c_str(), 1, fmtstr.size(), stdout);
#endif // ANDROID
}

void OutputWorker::Flush()
{
    // unique_lock used since FlushCondition.wait requires it
    std::unique_lock<std::mutex> locker(QueueLock);

    if (!Terminated)
    {
        FlushRequested = true;
        QueueCondition.notify_all();

        FlushCondition.wait(locker);
    }
}


//------------------------------------------------------------------------------
// Channel

Channel::Channel(const char* name, Level minLevel)
    : ChannelName(name)
    , ChannelMinLevel(minLevel)
{
}

std::string Channel::GetPrefix() const
{
    std::lock_guard<std::mutex> locker(PrefixLock);
    return Prefix;
}

void Channel::SetPrefix(const std::string& prefix)
{
    std::lock_guard<std::mutex> locker(PrefixLock);
    Prefix = prefix;
}


} // namespace logger


// Fix hang on shutdown due to MSVCRT bug prior to version 2015
#if defined(_MSC_VER) && (_MSC_VER < 1900)

// https://stackoverflow.com/questions/10915233/stdthreadjoin-hangs-if-called-after-main-exits-when-using-vs2012-rc
#pragma warning(disable:4073) // initializers put in library initialization area
#pragma init_seg(lib)

struct VS2013_threading_fix
{
    VS2013_threading_fix()
    {
        _Cnd_do_broadcast_at_thread_exit();
    }
} threading_fix;

#endif // _MSC_VER
