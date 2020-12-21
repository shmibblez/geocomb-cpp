#include "triangle.hpp"
#include <vector>

enum hash_type { rowCol,
    nessted };

enum map_orientation { ECEF,
    dymaxion };

enum rotation_method { gnomonic,
    quaternion };

class Icosahedron {

public:
    std::vector<Triangle> triangles;
    map_orientation map;
    rotation_method rotation;
};
