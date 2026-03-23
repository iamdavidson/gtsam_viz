#pragma once
#include <imgui.h>
#include <string>
#include <vector>
#include <mutex>

namespace gtsam_viz {

enum class LogLevel { Debug, Info, Warning, Error };

struct LogEntry {
    LogLevel    level;
    std::string msg;
    double      time;
};

// Global singleton logger (thread-safe ring buffer)
class AppLogger {
public:
    static AppLogger& get();

    void log(LogLevel lvl, const std::string& msg);
    void info(const std::string& msg)    { log(LogLevel::Info,    msg); }
    void warn(const std::string& msg)    { log(LogLevel::Warning, msg); }
    void error(const std::string& msg)   { log(LogLevel::Error,   msg); }
    void debug(const std::string& msg)   { log(LogLevel::Debug,   msg); }

    const std::vector<LogEntry>& entries() const { return entries_; }
    void clear() { std::lock_guard<std::mutex> g(mtx_); entries_.clear(); }

private:
    AppLogger() = default;
    static constexpr size_t MAX_ENTRIES = 2000;
    std::vector<LogEntry> entries_;
    std::mutex            mtx_;
};

// Panel widget
class LogPanel {
public:
    void draw();
private:
    LogLevel   minLevel_   = LogLevel::Debug;
    bool       autoScroll_ = true;
    char       filter_[128]= {};
};

} // namespace gtsam_viz

// Convenience macros
#define GVLOG_INFO(msg)  ::gtsam_viz::AppLogger::get().info(msg)
#define GVLOG_WARN(msg)  ::gtsam_viz::AppLogger::get().warn(msg)
#define GVLOG_ERR(msg)   ::gtsam_viz::AppLogger::get().error(msg)
#define GVLOG_DEBUG(msg) ::gtsam_viz::AppLogger::get().debug(msg)
