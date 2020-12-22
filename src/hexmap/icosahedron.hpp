#include "point3.hpp"
#include "triangle.hpp"
#include <vector>

enum hash_type { rowCol, nessted };

enum map_orientation { ECEF, dymaxion };

enum rotation_method { gnomonic, quaternion };

class Icosahedron {

public:
  std::vector<Triangle> triangles;
  map_orientation map;
  rotation_method rotation;
};

class Icosahedron {

public:
  map_orientation orientation;
  rotation_method rotation;
  Icosahedron(map_orientation orientation = map_orientation::ECEF,
              rotation_method rotation = rotation_method::gnomonic);

  /**
   * generate point from coordinates (degrees)
   * @param lat latitude
   * @param lon longitude */
  Point3 point_from_coords(double lat, double lon);
  /**
   * checks if point is phex center
   * @param res resolution */
  bool is_phex_center(int res, int row, int col);

  // TODO: make all methods static, and in js wrapper make
  // them instance methods (makes calling from js easier, also
  // don't think can export whole c++ obj to module)
};