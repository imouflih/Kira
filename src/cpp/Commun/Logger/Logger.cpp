#include "Logger.hpp"

#include <iostream>
#include <chrono>
#include <iomanip>

// Get current date and time in a format suitable for a filename. Returns a string in the format "YYYY-MM-DD_HH-MM-SS.log"
std::string getCurrentDateTimeFilename() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto local_time = std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S") << ".log";
    return oss.str();
}

// Singleton
Logger& Logger::getInstance(const std::string& filename) {
    static Logger instance(filename.empty() ? getCurrentDateTimeFilename() : filename);
    return instance;
}

Logger::Logger(const std::string& filename)
    : std::ostream(std::cout.rdbuf()), log_file("./logs/" + filename, std::ios_base::out | std::ios_base::app) {
    if (!log_file.is_open()) {
        std::cerr << "Failed to open log file: " << filename << std::endl;
        exit(1);
    }
    original_cout_buffer = std::cout.rdbuf();
    std::cout.rdbuf(log_file.rdbuf());
}

Logger::~Logger() {
    if (log_file.is_open()) {
        std::cout.rdbuf(original_cout_buffer);
        log_file.close();
    }
}
