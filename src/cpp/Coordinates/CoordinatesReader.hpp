#include <string>
#include <vector>
#include <tuple>

class CoordinatesReader {
public:
    CoordinatesReader(const std::string& filename);
    std::vector<std::tuple<std::string, int, int, double>> getCoordinates() const;

private:
    std::string filename;
};
