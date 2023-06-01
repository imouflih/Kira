#include <fstream>
#include <string>
#include <streambuf>
#include <ostream>

// Logger class is a custom output stream that redirects standard output (std::cout) to a log file
class Logger : public std::ostream {
public:
    static Logger& getInstance(const std::string& filename = "");

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    Logger(const std::string& filename);
    ~Logger();

    std::ofstream log_file; // The log file
    std::streambuf* original_cout_buffer; // A pointer to the original buffer of std::cout
};
