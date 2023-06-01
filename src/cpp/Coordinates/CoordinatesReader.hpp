#include <string>
#include <vector>
#include <tuple>

// The CoordinatesReader class provides method to read coordinates from json file
class CoordinatesReader {
public:
    CoordinatesReader(const std::string& filename);
    std::vector<std::tuple<std::string, int, int, float, float, int, int>> getCoordinates() const;

private:
    std::string filename;
};
