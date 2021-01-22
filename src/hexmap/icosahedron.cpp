#include "icosahedron.hpp"
#include <cmath>
#include <string>

using std::cos;
using std::sin;

Icosahedron::Icosahedron(map_orientation orientation = map_orientation::ECEF,
                         Icosahedron::rotation_method rotation =
                             Icosahedron::rotation_method::gnomonic)
    : orientation(orientation), rotation(rotation) {
  const double gr = constants::golden_ratio;
  const double r = constants::radius;
  const double factor = r / sqrt(gr * gr + 1);

  const double &_1 = factor;
  const double _gr = gr * factor;

  // rotate all base points so north pole aligns with z axis (this is angle
  // between vectors)
  const double rads = -acos(gr / sqrt(1 + gr * gr));
  /**
   *     how points are numbered
   *        N       N       N       N       N           - all top pent tris
   *        *       *       *       *       *             share north point
   *  top1    top2    top3    top4    top5    top1      - points are vertices
   *   *       *       *       *       *       *          of base of top pent
   *      bot1    bot2    bot3    bot4    bot5    bot1  - points ore vertices
   *       *       *       *       *       *       *      of base of bottom pent
   *           S       S       S       S       S        - all bottom pent tris
   *           *       *       *       *       *          share south point
   */
  Point3 north = Point3(_1, 0, _gr, true);
  north.rotate_around_y(rads);
  Point3 top1 = Point3(_gr, -_1, 0, true);
  top1.rotate_around_y(rads);
  Point3 top2 = Point3(_gr, _1, 0, true);
  top2.rotate_around_y(rads);
  Point3 top3 = Point3(0, _gr, _1, true);
  top3.rotate_around_y(rads);
  Point3 top4 = Point3(-_1, 0, _gr, true);
  top4.rotate_around_y(rads);
  Point3 top5 = Point3(0, -_gr, _1, true);
  top5.rotate_around_y(rads);

  Point3 bot1 = Point3(_1, 0, -_gr, true);
  bot1.rotate_around_y(rads);
  Point3 bot2 = Point3(0, _gr, -_1, true);
  bot2.rotate_around_y(rads);
  Point3 bot3 = Point3(-_gr, _1, 0, true);
  bot3.rotate_around_y(rads);
  Point3 bot4 = Point3(-_gr, -_1, 0, true);
  bot4.rotate_around_y(rads);
  Point3 bot5 = Point3(0, -_gr, -_1, true);
  bot5.rotate_around_y(rads);

  Point3 south = Point3(-_1, 0, -_gr, true);
  south.rotate_around_y(rads);

  /**
   *    how triangles are numbered
   *        /\      /\      /\      /\      /\
   *      / 0 \   / 1 \   / 2 \   / 3 \   / 4 \
   *    /______\/______\/______\/______\/______\
   *    \      /\      /\      /\      /\      /\
   *     \ 5 / 6 \ 7 / 8 \  9/ 10\ 11/ 12\ 13/ 14\
   *      \/______\/______\/______\/______\/______\
   *       \      /\      /\      /\      /\      /
   *        \ 15/   \ 16/   \ 17/   \ 18/   \ 19/
   *         \/      \/      \/      \/      \/
   */
  /**
   * top pent
   */
  // 0
  this->triangles.push_back(
      Triangle(north, top2, top1, pointing::UP, position::TOP, 0, 1, 5, 4));
  // 1
  this->triangles.push_back(
      Triangle(north, top3, top2, pointing::UP, position::TOP, 1, 2, 7, 0));
  // 2
  this->triangles.push_back(
      Triangle(north, top4, top3, pointing::UP, position::TOP, 2, 3, 9, 1));
  // 3
  this->triangles.push_back(
      Triangle(north, top5, top4, pointing::UP, position::TOP, 3, 4, 11, 2));
  // 4
  this->triangles.push_back(
      Triangle(north, top1, top5, pointing::UP, position::TOP, 4, 0, 13, 3));
  /**
   * center triangles
   */
  // 5
  this->triangles.push_back(Triangle(bot1, top1, top2, pointing::DOWN,
                                     position::CENTER, 5, 14, 0, 16));
  // 6
  this->triangles.push_back(
      Triangle(top2, bot2, bot1, pointing::UP, position::CENTER, 6, 7, 15, 5));
  // 7
  this->triangles.push_back(
      Triangle(bot2, top2, top3, pointing::DOWN, position::CENTER, 7, 6, 1, 8));
  // 8
  this->triangles.push_back(
      Triangle(top3, bot3, bot2, pointing::UP, position::CENTER, 8, 9, 16, 7));
  // 9
  this->triangles.push_back(Triangle(bot3, top3, top4, pointing::DOWN,
                                     position::CENTER, 9, 8, 2, 10));
  // 10
  this->triangles.push_back(Triangle(top4, bot4, bot3, pointing::UP,
                                     position::CENTER, 10, 11, 17, 9));
  // 11
  this->triangles.push_back(Triangle(bot4, top4, top5, pointing::DOWN,
                                     position::CENTER, 11, 10, 3, 12));
  // 12
  this->triangles.push_back(Triangle(top5, bot5, bot4, pointing::UP,
                                     position::CENTER, 12, 13, 18, 11));
  // 13
  this->triangles.push_back(Triangle(bot5, top5, top1, pointing::DOWN,
                                     position::CENTER, 13, 12, 4, 14));
  // 14
  this->triangles.push_back(Triangle(top1, bot1, bot5, pointing::UP,
                                     position::CENTER, 14, 5, 19, 13));
  /**
   * bottom pent
   */
  // 15
  this->triangles.push_back(Triangle(south, bot1, bot2, pointing::DOWN,
                                     position::BOT, 15, 19, 6, 16));
  // 16
  this->triangles.push_back(Triangle(south, bot2, bot3, pointing::DOWN,
                                     position::BOT, 16, 15, 8, 17));
  // 17
  this->triangles.push_back(Triangle(south, bot3, bot4, pointing::DOWN,
                                     position::BOT, 17, 16, 10, 18));
  // 18
  this->triangles.push_back(Triangle(south, bot4, bot5, pointing::DOWN,
                                     position::BOT, 18, 17, 12, 19));
  // 19
  this->triangles.push_back(Triangle(south, bot5, bot1, pointing::DOWN,
                                     position::BOT, 19, 18, 14, 15));
};

std::string Icosahedron::map_orientation_key(map_orientation mo) {
  return std::vector<std::string>({"e", "d"})[mo];
};

std::string Icosahedron::rotation_method_key(rotation_method rm) {
  return std::vector<std::string>({"g", "q"})[rm];
};

Point3 Icosahedron::point_from_coords(double lat, double lon) const {
  if (!(lat <= 90 && lat >= 90)) {
    throw std::invalid_argument("lat must be between -90 and 90");
  }
  if (!(lon <= 180 && lon >= -180)) {
    throw std::invalid_argument("lon must be beween -180 and 180");
  }
  lat = hexmapf::deg_2_rad(lat);
  lon = hexmapf::deg_2_rad(lon);
  const double r = constants::radius;
  const double x = r * cos(lat) * cos(lon);
  const double y = r * cos(lat) * sin(lon);
  const double z = r * sin(lat);
  return Point3(x, y, z);
};

Icosahedron::hash_properties Icosahedron::hash(Point3 p, int res) {
  Icosahedron::all_icosahedron_points lazy_points =
      this->lazy_points_around(p, res);
  // closest point that is also phex center
  GPoint3 cp = p.closest_point_2d(lazy_points);
  return Icosahedron::hash_properties{.res = res, .row = cp.row, .col = cp.col};
};

/**
 * @param p point
 * @param res resolution
 * @returns lazily generated points around p
 **/
std::vector<std::vector<GPoint3>>
Icosahedron::lazy_points_around(Point3 p, int res) const {
  const Triangle tri = this->containing_triangle(p);
  const int nd = hexmapf::num_divisions(res);
  tri.lazy_points_around(p, res, this->rotation);
  // points and lazy range start indexes in relation to tri.C
  // TODO: left off here
};

/**
 * @param p point to test
 * @returns icosahedron triangle containing p
 **/
Triangle Icosahedron::containing_triangle(Point3 p) const {};

/**
 * @param hash hexmap hash in format res|row|col
 * @returns point referenced by hash
 **/
GPoint3 Icosahedron::parse_hash(std::string hash) const {};

/**
 * @param res resolution
 * @returns all icosahedron points for resolution
 **/
Icosahedron::all_icosahedron_points
Icosahedron::all_points(int res = 1) const {};

std::vector<Phex> Icosahedron::all_phexes(int res) {

  Icosahedron::all_icosahedron_points all_points = this->all_points(res);
  std::vector<GPoint3> centers = Phex::all_phex_centers(all_points);

  std::vector<Phex> phexes;

  // create phexes for each phex center point
  for (GPoint3 c : centers) {
    // first hex center index is -> row_num % 3
    phexes.push_back(Phex(Phex::not_lazy_surrounding_points(all_points, c), c));
  }

  return phexes;
};

/**
 * -!-> not lazy, generates all points & phexes
 * @param p point
 * @param res resolution
 * @returns phex for res containing p
 **/
Phex Icosahedron::not_lazy_containing_phex(Point3 p, int res) const {};

/**
 * @param p point
 * @returns triangle containing p
 **/
Triangle Icosahedron::containing_triangle(Point3 p) const {};
