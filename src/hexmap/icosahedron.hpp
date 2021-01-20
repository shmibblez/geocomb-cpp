#include "point3.hpp"
#include "triangle.hpp"
#include <vector>

enum hash_type { rowCol, nested };

enum map_orientation { ECEF, dymaxion };

enum rotation_method { gnomonic, quaternion };

class Icosahedron {

public:
  std::vector<Triangle> triangles;
  map_orientation orientation;
  rotation_method rotation;
  Icosahedron(map_orientation orientation = map_orientation::ECEF,
              rotation_method rotation = rotation_method::gnomonic);

  /**
   * generate point from coordinates (degrees)
   * @param lat latitude
   * @param lon longitude */
  Point3 point_from_coords(double lat, double lon);

  // TODO: generate hash here, in js version it's in Point3, point_from_coords
  // is also in Point3 in js version
  std::vector<std::any> hash(Point3 p, map_orientation orientation,
                             rotation_method rotation);

  // TODO: make all methods static, and in js wrapper make
  // them instance methods (makes calling from js easier, also
  // don't think can export whole c++ obj to module)
};