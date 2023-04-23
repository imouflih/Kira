#include <fstream>
#include <string>
#include <streambuf>
#include <ostream>

class Logger: public std::ostream {
public:
    static Logger& getInstance(const std::string& filename = "");

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    Logger(const std::string& filename);
    ~Logger();

    std::ofstream log_file;
    std::streambuf* original_cout_buffer;
};
