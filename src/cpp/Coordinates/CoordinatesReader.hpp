#include <string>
#include <vector>
#include <tuple>

class CoordinatesReader {
public:
    CoordinatesReader(const std::string& filename);
    std::vector<std::tuple<std::string, int, int, float, int, int, int>> getCoordinates() const;

private:
    std::string filename;
};
