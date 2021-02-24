#include "icosahedron.hpp"
#include "phex.hpp"
#include "point3.hpp"
#include "triangle.hpp"
#include <cmath>
#include <functional>
#include <string>

using std::cos;
using std::sin;
using std::trunc;

Icosahedron::Icosahedron(ico::map_orientation orientation,
                         ico::rotation_method rotation)
    : tris(Icosahedron::triangles()), mo(orientation), rm(rotation) {}

std::string Icosahedron::map_orientation_key(ico::map_orientation mo) {
  return std::vector<std::string>({"e", "d"})[mo];
}

std::string Icosahedron::rotation_method_key(ico::rotation_method rm) {
  return std::vector<std::string>({"g", "q"})[rm];
}

/**
 * generates icosahedron triangles
 * @returns vector of icosahedron triangles
 */
std::vector<Triangle> Icosahedron::triangles() {
  const long double gr = constants::golden_ratio;
  const long double r = constants::radius;
  const long double factor = r / sqrt(gr * gr + 1.0);

  const long double &_1 = factor;
  const long double _gr = gr * factor;

  // rotate all base points so north pole aligns with z axis (this is angle
  // between vectors)
  const long double rads = -acos(gr / sqrt(1.0 + gr * gr));
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

  Point3 north(_1, 0.0, _gr, true);
  north.rotate_around_y(rads);
  Point3 top1(_gr, -_1, 0.0, true);
  top1.rotate_around_y(rads);
  Point3 top2(_gr, _1, 0.0, true);
  top2.rotate_around_y(rads);
  Point3 top3(0, _gr, _1, true);
  top3.rotate_around_y(rads);
  Point3 top4(-_1, 0.0, _gr, true);
  top4.rotate_around_y(rads);
  Point3 top5(0, -_gr, _1, true);
  top5.rotate_around_y(rads);

  Point3 bot1(_1, 0.0, -_gr, true);
  bot1.rotate_around_y(rads);
  Point3 bot2(0, _gr, -_1, true);
  bot2.rotate_around_y(rads);
  Point3 bot3(-_gr, _1, 0.0, true);
  bot3.rotate_around_y(rads);
  Point3 bot4(-_gr, -_1, 0.0, true);
  bot4.rotate_around_y(rads);
  Point3 bot5(0, -_gr, -_1, true);
  bot5.rotate_around_y(rads);

  Point3 south(-_1, 0.0, -_gr, true);
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
  const std::vector<Triangle> tris({
      /**
       * top pent
       */
      // 0
      Triangle(north, top2, top1, tri::pointing::UP, tri::position::TOP, 0, 1,
               5, 4),
      // 1
      Triangle(north, top3, top2, tri::pointing::UP, tri::position::TOP, 1, 2,
               7, 0),
      // 2
      Triangle(north, top4, top3, tri::pointing::UP, tri::position::TOP, 2, 3,
               9, 1),
      // 3
      Triangle(north, top5, top4, tri::pointing::UP, tri::position::TOP, 3, 4,
               11, 2),
      // 4
      Triangle(north, top1, top5, tri::pointing::UP, tri::position::TOP, 4, 0,
               13, 3),
      /**
       * center triangles
       */
      // 5
      Triangle(bot1, top1, top2, tri::pointing::DOWN, tri::position::CENTER, 5,
               14, 0, 16),
      // 6
      Triangle(top2, bot2, bot1, tri::pointing::UP, tri::position::CENTER, 6, 7,
               15, 5),
      // 7
      Triangle(bot2, top2, top3, tri::pointing::DOWN, tri::position::CENTER, 7,
               6, 1, 8),
      // 8
      Triangle(top3, bot3, bot2, tri::pointing::UP, tri::position::CENTER, 8, 9,
               16, 7),
      // 9
      Triangle(bot3, top3, top4, tri::pointing::DOWN, tri::position::CENTER, 9,
               8, 2, 10),
      // 10
      Triangle(top4, bot4, bot3, tri::pointing::UP, tri::position::CENTER, 10,
               11, 17, 9),
      // 11
      Triangle(bot4, top4, top5, tri::pointing::DOWN, tri::position::CENTER, 11,
               10, 3, 12),
      // 12
      Triangle(top5, bot5, bot4, tri::pointing::UP, tri::position::CENTER, 12,
               13, 18, 11),
      // 13
      Triangle(bot5, top5, top1, tri::pointing::DOWN, tri::position::CENTER, 13,
               12, 4, 14),
      // 14
      Triangle(top1, bot1, bot5, tri::pointing::UP, tri::position::CENTER, 14,
               5, 19, 13),
      /**
       * bottom pent
       */
      // 15
      Triangle(south, bot1, bot2, tri::pointing::DOWN, tri::position::BOT, 15,
               19, 6, 16),
      // 16
      Triangle(south, bot2, bot3, tri::pointing::DOWN, tri::position::BOT, 16,
               15, 8, 17),
      // 17
      Triangle(south, bot3, bot4, tri::pointing::DOWN, tri::position::BOT, 17,
               16, 10, 18),
      // 18
      Triangle(south, bot4, bot5, tri::pointing::DOWN, tri::position::BOT, 18,
               17, 12, 19),
      // 19
      Triangle(south, bot5, bot1, tri::pointing::DOWN, tri::position::BOT, 19,
               18, 14, 15),
  });
  return tris;
}

/**
 * -!-> very inefficient, literally just generates all tris and returns one at
 * indx (need to optimize)
 * @param indx index of triangle to generate
 * @returns icosahedron triangle at [indx]
 **/
Triangle Icosahedron::triangle(const int indx) {
  // TODO: how to optimize? (only create necessary tri)
  // proposal: have static class that creates specific triangles (function for
  // each triangle), but then, how to call a function? maybe then store array of
  // lambdas and call the function at tri indx?
  if (indx < 0 || indx > 19) {
    throw std::invalid_argument("icosahedron triangle indx must be within "
                                "range [0, 19], provided indx: " +
                                std::to_string(indx));
  }
  return Icosahedron::triangles()[indx];
}

Point3 Icosahedron::point_from_coords(long double lat, long double lon) const {
  if (!(lat <= 90 && lat >= -90)) {
    throw std::invalid_argument("lat must be between -90 and 90");
  }
  if (!(lon <= 180 && lon >= -180)) {
    throw std::invalid_argument("lon must be beween -180 and 180");
  }
  lat = hexmapf::deg_2_rad(lat);
  lon = hexmapf::deg_2_rad(lon);
  const long double r = constants::radius;
  const long double x = r * cos(lat) * cos(lon);
  const long double y = r * cos(lat) * sin(lon);
  const long double z = r * sin(lat);
  return Point3(x, y, z);
}

Icosahedron::hash_properties Icosahedron::hash(Point3 p, int res) {
  Icosahedron::all_icosahedron_points lazy_points =
      this->lazy_points_around(p, res);

  // closest point that is also phex center
  GPoint3 cp = p.closest_point_2d(lazy_points);

  return Icosahedron::hash_properties{
      .res = res,
      .row = cp.row,
      .col = cp.col,
      .rm = this->rm,
      .mo = this->mo,
  };
}

std::vector<std::vector<GPoint3>>
Icosahedron::lazy_points_around(Point3 p, int res) const {
  const Triangle tri = this->containing_triangle(p);
  const int nd = hexmapf::num_divisions(res);
  // points and lazy range start indexes in relation to tri.C
  Triangle::lazy_points_around_result tri_points_around =
      tri.lazy_points_around(p, res, this->rm);

  const int lower_vert = tri_points_around.start_vert;
  const int lower_horz = tri_points_around.start_horz;

  const std::vector<std::vector<Point3>> points = tri_points_around.points;

  // TODO: move indexing functions to static function class instead of creating
  // lambdas every time below
  //.
  // index top
  const std::function index_top =
      [&lower_vert, &lower_horz, *this, &tri, &points,
       &res]() -> std::vector<std::vector<GPoint3>> {
    // TODO: test if works, point numbering and pushing to vecs
    std::vector<std::vector<GPoint3>> indexed_points;
    const int row_off = lower_vert;
    for (unsigned int r = 0; r < points.size() && points[0].size() > 0; r++) {
      const int row = row_off + r;
      indexed_points.push_back(std::vector<GPoint3>());
      for (unsigned int c = 0; c < points[r].size(); c++) {
        int col = (lower_vert + r) * tri.num + lower_horz + c;
        if (col == row * 5) {
          col = 0;
        }
        const Point3 p3 = points[r][c];
        const GPoint3 new_point(p3.x, p3.y, p3.z, res, row, col, this->mo,
                                this->rm,
                                GPoint3::is_phex_center(res, row, col));
        indexed_points[r].push_back(new_point);
      }
    }
    return indexed_points;
  };
  // index center up
  const std::function index_cen_up =
      [&nd, &lower_vert, &lower_horz, *this, &points, &res,
       &tri]() -> std::vector<std::vector<GPoint3>> {
    std::vector<std::vector<GPoint3>> indexed_points;
    const int num_tris_before = hexmapf::closest_even_num(tri.num - 5);
    const int row_off = nd + lower_vert;
    const int col_off =
        nd * (num_tris_before / 2) + (nd - lower_vert) + lower_horz;
    for (unsigned int r = 0; r < points.size() && points[r].size() > 0; r++) {
      const int row = row_off + r;
      indexed_points.push_back(std::vector<GPoint3>());
      for (unsigned int c = 0; c < points[r].size(); c++) {
        int col = col_off + c - r;
        if (col == nd * 5) {
          col = 0;
        }
        const Point3 p3 = points[r][c];
        const GPoint3 new_point(p3.x, p3.y, p3.z, res, row, col, this->mo,
                                this->rm,
                                GPoint3::is_phex_center(res, row, col));
        indexed_points[r].push_back(new_point);
      }
    }
    return indexed_points;
  };
  // index center down
  std::function index_cen_dn = [&nd, &lower_vert, &lower_horz, &res, *this,
                                &points,
                                &tri]() -> std::vector<std::vector<GPoint3>> {
    std::vector<std::vector<GPoint3>> indexed_points;
    const int num_tris_before = hexmapf::closest_even_num(tri.num - 5);
    const int row_off = nd + lower_vert;
    const int col_off = nd * (num_tris_before / 2);
    for (unsigned int r = 0; r < points.size() && points[r].size() > 0; r++) {
      const int row = row_off + r;
      indexed_points.push_back(std::vector<GPoint3>());
      for (unsigned int c = 0; c < points[r].size(); c++) {
        int col = col_off + nd * 2 - row - lower_horz - c;
        if (col == nd * 5) {
          col = 0;
        }
        const Point3 p3 = points[r][c];
        const GPoint3 new_point(p3.x, p3.y, p3.z, res, row, col, this->mo,
                                this->rm,
                                GPoint3::is_phex_center(res, row, col));
        indexed_points[r].push_back(new_point);
      }
    }
    return indexed_points;
  };
  // index bot
  std::function index_bot = [&nd, &lower_horz, &lower_vert, &res, *this,
                             &points,
                             &tri]() -> std::vector<std::vector<GPoint3>> {
    std::vector<std::vector<GPoint3>> indexed_points;
    const int row_off = nd * 2 + lower_vert;
    for (unsigned int r = 0; r < points.size() && points[r].size() > 0; r++) {
      const int row = row_off + r;
      indexed_points.push_back(std::vector<GPoint3>());
      for (unsigned int c = 0; c < points[r].size(); c++) {
        int col = (nd - lower_vert - r) * (tri.num - 15 + 1) - lower_horz - c;
        if (col == (nd * 3 - row) * 5) {
          col = 0;
        }
        const Point3 p3 = points[r][c];
        const GPoint3 new_point(p3.x, p3.y, p3.z, res, row, col, this->mo,
                                this->rm,
                                GPoint3::is_phex_center(res, row, col));
        indexed_points[r].push_back(new_point);
      }
    }
    return indexed_points;
  };

  // no more functions, now determine which one to use
  std::vector<std::vector<GPoint3>> lazy_points;
  switch (tri.pos) {
  case tri::position::TOP: {
    lazy_points = index_top();
    break;
  }
  case tri::position::CENTER: {
    lazy_points =
        tri.direction == tri::pointing::UP ? index_cen_up() : index_cen_dn();
    break;
  }
  case tri::position::BOT: {
    lazy_points = index_bot();
    break;
  }
  case tri::position::NA: {
    throw std::logic_error(
        "Icosahedron::lazy_points_around failed -> tri from icosahedron->tris "
        "didn't have pos for some reason, this should never happen");
  }
  }

  return lazy_points;
}

Triangle Icosahedron::containing_triangle(Point3 p) const {
  for (const Triangle &t : this->tris) {
    if (t.contains_point(p)) {

      return t;
    }
  }
  throw std::logic_error(
      "Icosahedron::containing_triangle failed to find containing triangle. "
      "This is likely due to a rounding error. Point coords -> x: " +
      std::to_string(p.x) + ", y: " + std::to_string(p.y) +
      ", z: " + std::to_string(p.z));
}

GPoint3 Icosahedron::parse_hash(Icosahedron::hash_properties hash) const {
  if (hash.res <= 0) {
    throw std::invalid_argument("invalid hash -> resolution must be "
                                "greater than 0, provided res: " +
                                std::to_string(hash.res));
  }
  const int nd = hexmapf::num_divisions(hash.res);
  // check if row ok
  const int max_rows = nd * 3;
  if ((hash.row > max_rows || hash.row < 0)) {
    throw std::invalid_argument(
        "invalid hash -> row for res " + std::to_string(hash.res) +
        " must be within range [0, " + std::to_string(max_rows) +
        "], provided row: " + std::to_string(hash.row));
  }
  // check if col ok
  const int max_cols = nd * 5 - 1;
  if (hash.col > max_cols || hash.col < 0) {
    throw std::invalid_argument(
        "invalid hash -> col for res " + std::to_string(hash.res) +
        " must be within range [0, " + std::to_string(max_cols) +
        "], provided col: " + std::to_string(hash.col));
  }

  int tri_num = -1;
  int lower_horz = -1;
  int lower_vert = -1;
  int &row = hash.row;
  int &col = hash.col;

  if (row < nd) {
    // point in top tris
    tri_num = row != 0 ? trunc(col / row) : 0;
    lower_vert = row;
    lower_horz = col - tri_num * row;
  } else if (row <= nd * 2) {
    // point in center tris
    lower_vert = row - nd;
    const int tri_offs = trunc(col / nd);
    const int col_offs = tri_offs * nd + (nd - lower_vert);
    const bool pointing_up = col > col_offs;

    if (pointing_up) {
      tri_num = 5 + tri_offs * 2 + 1;
      lower_horz = col - col_offs;
    } else {
      tri_num = 5 + tri_offs * 2;
      lower_horz = col_offs - col;
    }
  } else {
    // point in bottom tris
    lower_vert = row - nd * 2;
    const int cols_per_row = nd - lower_vert;
    const int tri_offs = cols_per_row != 0 ? trunc(col / cols_per_row) : 0;
    lower_horz = cols_per_row - (col - tri_offs * cols_per_row);
    tri_num = 15 + tri_offs;
  }

  const Triangle tri = Icosahedron::triangle(tri_num);
  const Point3 p =
      tri.generate_point(hash.res, lower_vert, lower_horz, hash.rm);

  return GPoint3(p.x, p.y, p.z, hash.res, row, col, hash.mo, hash.rm,
                 GPoint3::is_phex_center(hash.res, row, col), tri.num);
}

/**
 * TODO: need to test, have funny feeling won't work
 **/
Icosahedron::all_icosahedron_points Icosahedron::all_points(int res) const {
  Icosahedron::all_icosahedron_points points;
  const int offset_amount = res * 3;
  for (const Triangle &t : this->tris) {
    int offset;
    int range;
    std::vector<std::vector<Point3>> ps = t.all_points(res, this->rm);
    switch (t.pos) {
    case tri::position::TOP: {
      offset = 0;
      range = ps.size() - 1;
      break;
    }
    case tri::position::CENTER: {
      offset = offset_amount;
      range = ps.size() - 1;
      break;
    }
    case tri::position::BOT: {
      offset = offset_amount * 2;
      range = ps.size();
      break;
    }
    case tri::position::NA: {
      throw std::logic_error(
          "Icosahedron::all_points failed -> tri from icosahedron->tris didn't "
          "have pos for some reason, this should never happen");
    }
    };
    for (int fl = 0; fl < range; fl++) {
      if ((signed)points.size() < offset + fl) {
        // since tris start from top to bottom, this shouldn't skip any indices
        // -> adding in order, should do the same as js version (original)
        points.push_back(std::vector<GPoint3>());
      }
      Point3 popped = ps[fl].back();
      ps[fl].pop_back();
      for (unsigned int sl = 0; sl < ps[fl].size(); sl++) {
        // adds gpoint3, storing indexes (for referencing other points)
        points[offset + fl].push_back(
            GPoint3(ps[fl][sl].x, ps[fl][sl].y, ps[fl][sl].z, res, offset + fl,
                    static_cast<int>(points[offset + fl].size()), this->mo,
                    this->rm, ps[fl][sl].is_pc));
      }
    }
  }
  return points;
}

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
}

/**
 * TODO: need to test of course
 **/
Phex Icosahedron::not_lazy_containing_phex(Point3 p, int res) const {
  const Icosahedron::all_icosahedron_points all_points = this->all_points(res);
  const std::vector<Phex> phexes = Phex::all_phexes(all_points);

  std::unique_ptr<Phex> closest_phex;
  long double dist;
  long double smallest_dist = constants::radius * 2;
  for (Phex phex : phexes) {
    dist = p.distance(phex.center);
    if (dist < smallest_dist) {
      smallest_dist = dist;
      // since phexes is created outside of loop, should remain in scope after
      // loop
      closest_phex.reset(&phex);
    }
  }
  // return closest phex value (copy)
  // need to check though, if RVO is applied here we're screwed
  return *closest_phex;
}
