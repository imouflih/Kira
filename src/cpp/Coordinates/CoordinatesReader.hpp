#include <string>
#include <vector>
#include <utility>

class CoordinatesReader {
public:
    CoordinatesReader(const std::string& filename);
    std::vector<std::pair<int, int>> getCoordinates() const;

private:
    std::string filename;
};
