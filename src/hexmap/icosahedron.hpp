#include "phex.hpp"
#include "point3.hpp"
#include "triangle.hpp"
#include <map>
#include <vector>

class Icosahedron {

public:
  enum hash_type { rowCol, nested };
  enum map_orientation { ECEF, dymaxion };
  enum rotation_method { gnomonic, quaternion };

  const std::vector<Triangle> tris;
  const map_orientation mo;
  const rotation_method rm;
  const hash_type ht;
  Icosahedron(map_orientation orientation = map_orientation::ECEF,
              rotation_method rotation = rotation_method::gnomonic,
              hash_type ht = hash_type::rowCol);

  typedef std::vector<std::vector<GPoint3>> all_icosahedron_points;
  typedef std::vector<std::vector<GPoint3>> lazy_icosahedron_points;

  struct hash_properties {
    int res;
    int row;
    int col;
    rotation_method rm;
    map_orientation mo;
    hash_type ht;
  };

  /**
   * @param mo map_orientation
   * @returns map_orientation key for mo
   **/
  static std::string map_orientation_key(map_orientation mo);

  /**
   * @param rm rotation_method
   * @returns rotation_method key for rm
   **/
  static std::string rotation_method_key(rotation_method rm);

  /**
   * generates icosahedron triangles
   * @returns vector of icosahedron triangles
   */
  static std::vector<Triangle> triangles();

  /**
   * @param indx index of triangle to generate
   * @returns icosahedron triangle at [indx]
   **/
  static Triangle triangle(const int indx);

  /**
   * generate point from coordinates (degrees)
   * @param lat latitude
   * @param lon longitude */
  Point3 point_from_coords(double lat, double lon) const;

  // TODO: generate hash here, in js version it's in Point3, point_from_coords
  // is also in Point3 in js version
  /**
   * @param p point to generate hash for
   * @param res resolution
   * @returns hash properties of containing phex
   **/
  hash_properties hash(Point3 p, int res);

  /**
   * @param p point
   * @param res resolution
   * @returns lazily generated points around p
   **/
  std::vector<std::vector<GPoint3>> lazy_points_around(Point3 p, int res) const;

  /**
   * @param p point to test
   * @returns icosahedron triangle containing p
   **/
  Triangle containing_triangle(Point3 p) const;

  /**
   * @param hash hexmap hash in format res|row|col
   * @returns point referenced by hash
   **/
  GPoint3 parse_hash(Icosahedron::hash_properties hash) const;

  /**
   * @param res resolution
   * @returns all icosahedron points for resolution
   **/
  all_icosahedron_points all_points(int res = 1) const;

  /**
   * @param res resolution
   * @returns vector of all phexes for res
   **/
  std::vector<Phex> all_phexes(int res);

  /**
   * -!-> not lazy, generates all points & phexes
   * @param p point
   * @param res resolution
   * @returns phex for res containing p
   **/
  Phex not_lazy_containing_phex(Point3 p, int res) const;

  /**
   * @param p point
   * @returns triangle containing p
   **/
  Triangle containing_triangle(Point3 p) const;
};