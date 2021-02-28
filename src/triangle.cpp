#include "triangle.hpp"
#include "enums.hpp"

// #include <string>

using std::round;

Triangle::Triangle(Point3 A, Point3 B, Point3 C, tri::pointing direction,
                   tri::position pos, int num, int toAB, int toBC, int toCA)
    : A(A), B(B), C(C), direction(direction), pos(pos), num(num), toAB(toAB),
      toBC(toBC), toCA(toCA){};

Triangle::vec_side_components_result
Triangle::vec_side_components(const Triangle &tri, const Point3 &i) {
  Point3 u_CA = tri.A;
  u_CA.subtract(tri.C);
  u_CA.unit();
  Point3 u_CB = tri.B;
  u_CB.subtract(tri.C);
  u_CB.unit();
  Point3 CI = i;
  CI.subtract(tri.C);

  const long double beta = u_CA.angle_between(CI);
  const long double alpha = CI.angle_between(u_CB);
  const long double phi = constants::PI - beta - alpha;

  // law of sines to find missing magnitudes & lengths
  const long double mag_CI = CI.mag();
  const long double mag_CB = (mag_CI * sin(beta)) / sin(phi);
  const long double mag_CA = (mag_CI * sin(alpha)) / sin(phi);

  Point3 vec_CA = u_CA;
  vec_CA.mult_by(mag_CA);
  Point3 vec_CB = u_CB;
  vec_CB.mult_by(mag_CB);
  return {.vec_CA = vec_CA, .vec_CB = vec_CB};
}

Triangle::calc_percent_result
Triangle::calc_percent_gnomonic(const Point3 &p) const {
  // int precision = std::numeric_limits<long double>::max_digits10;

  const long double r = constants::radius;
  Point3 original_AB = this->B;
  original_AB.subtract(this->A);

  Point3 c_AB = original_AB;
  c_AB.div_by(2.0);
  c_AB.add(this->A);

  Point3 cent = c_AB;
  cent.subtract(this->C);
  cent.mult_by(2.0 / 3.0);
  cent.add(this->C);
  cent.unit();
  cent.mult_by(r);

  const long double alpha = this->C.angle_between(cent);
  const long double mag_cent = cent.mag();
  const long double mag_h = mag_cent / cos(alpha);
  Point3 A = this->A;
  A.unit();
  A.mult_by(mag_h);
  Point3 B = this->B;
  B.unit();
  B.mult_by(mag_h);
  Point3 C = this->C;
  C.unit();
  C.mult_by(mag_h);
  const Triangle projected_tri = Triangle(A, B, C);
  const Point3 projected_p = projected_tri.plane_intersection(p);

  // std::cout << "\nmag_h: " << mag_h << "\nmag_cent: " << mag_cent
  //           << "\nalpha: " << alpha << "\ncos(alpha): " << cos(alpha) <<
  //           "\n\n";

  Triangle::vec_side_components_result components =
      Triangle::vec_side_components(projected_tri, projected_p);

  const long double mag_comp_CA = components.vec_CA.mag();
  const long double mag_comp_CB = components.vec_CB.mag();
  Point3 mag_temp = B;
  mag_temp.subtract(A);
  const long double mag = mag_temp.mag();

  return {.percent_CA = mag_comp_CA / mag, .percent_CB = mag_comp_CB / mag};
}

Triangle::calc_percent_result
Triangle::calc_percent_quaternion(const Point3 &p) const {
  throw std::logic_error("CalcPercent->quaternion not ready yet");
  // so don't get warning
  return {.percent_CA = (p.x * 0) - 1, .percent_CB = -1};
}

std::vector<std::vector<Point3>>
Triangle::all_points(int res, ico::rotation_method rm) const {
  // empty 2d vec
  std::vector<std::vector<Point3>> points;

  const int max_divisions = hexmapf::num_divisions(res);
  const Point3 left_above = this->direction == tri::pointing::UP ? A : B;
  const Point3 left_below = this->direction == tri::pointing::UP ? C : A;
  std::vector<Point3> left_points =
      rm == ico::rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(left_above, left_below, res)
          : Point3::all_side_points_quaternion(left_above, left_below, res);

  const Point3 right_above = this->direction == tri::pointing::UP ? A : C;
  const Point3 right_below = this->direction == tri::pointing::UP ? B : A;
  std::vector<Point3> right_points =
      rm == ico::rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(right_above, right_below, res)
          : Point3::all_side_points_quaternion(right_above, right_below, res);

  for (int x = 0; x <= max_divisions; x++) {
    int num_divs = this->direction == tri::pointing::UP ? x : max_divisions - x;
    std::vector<Point3> new_points =
        rm == ico::rotation_method::gnomonic
            ? Point3::all_row_points_gnomonic(left_points[x], right_points[x],
                                              num_divs)
            : Point3::all_row_points_quaternion(left_points[x], right_points[x],
                                                num_divs);
    points.push_back(rm == ico::rotation_method::gnomonic
                         ? Point3::spherify1D(new_points)
                         : new_points);
  }
  return points;
}

Triangle::lazy_points_around_result
Triangle::lazy_points_around(Point3 &p, int res,
                             ico::rotation_method rotation) const {

  const int nd = hexmapf::num_divisions(res);
  // calc side percents
  const Triangle::calc_percent_result percents =
      rotation == ico::rotation_method::gnomonic
          ? this->calc_percent_gnomonic(p)
          : this->calc_percent_quaternion(p);

  // std::cout << "percents, percent_CA: " <<
  // percents.percent_CA
  //           << ", percent_CB: " << percents.percent_CB <<
  //           "\n";

  // calculate percent of intersect component from C to A
  const int estimated_vert_center = this->direction == tri::pointing::UP
                                        ? round(nd - percents.percent_CA * nd)
                                        : round(percents.percent_CA * nd);

  // lazy calculate points
  Point3::lazy_side_points_result side_point_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_side_points_gnomonic(*this, estimated_vert_center, res)
          : Point3::lazy_side_points_quaternion(*this, estimated_vert_center,
                                                res);

  std::vector<std::vector<Point3>> points;

  const int estimated_horz_center = round(percents.percent_CB * nd);
  // while vertical points exist, generate points for their rows in range
  int lower_horz_bound;
  // not hit or miss like js hexmap, here lazy range starts from vec[0]
  // accompanied by indx
  // TODO: need to test (check above comment)
  for (unsigned int i = 0; i < side_point_result.pointsL.size(); i++) {
    // generates points for range between left and right points along vertical
    // triangle sides (AB an AC)
    Point3 left = this->direction == tri::pointing::UP
                      ? side_point_result.pointsL[i]
                      : side_point_result.pointsR[i];
    Point3 right = this->direction == tri::pointing::UP
                       ? side_point_result.pointsR[i]
                       : side_point_result.pointsL[i];
    int num_div = this->direction == tri::pointing::UP
                      ? side_point_result.lower_indx + i
                      : nd - (side_point_result.lower_indx + i);

    Point3::lazy_row_points_result row_points_result =
        rotation == ico::rotation_method::gnomonic
            ? Point3::lazy_row_points_gnomonic(estimated_horz_center, left,
                                               right, num_div)
            : Point3::lazy_row_points_quaternion(estimated_horz_center, left,
                                                 right, num_div);

    points.push_back(rotation == ico::rotation_method::gnomonic
                         ? Point3::spherify1D(row_points_result.row_points)
                         : row_points_result.row_points);
    // better way to set lower_horz_bound? only need last value...
    lower_horz_bound = row_points_result.lower_indx;
  }

  return {.points = points,
          .start_vert = side_point_result.lower_indx,
          .start_horz = lower_horz_bound};
}

Point3 Triangle::generate_point(int res, int lower_vert, int lower_horz,
                                ico::rotation_method rotation) const {
  const int nd = hexmapf::num_divisions(res);
  // kind of hacky but works
  Point3::lazy_side_points_result vert_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_side_points_gnomonic(*this, lower_vert, res, 0,
                                              lower_vert, lower_vert)
          : Point3::lazy_side_points_quaternion(*this, lower_vert, res, 0,
                                                lower_vert, lower_vert);

  const Point3 left = this->direction == tri::pointing::UP
                          ? vert_result.pointsL[0]
                          : vert_result.pointsR[0];
  const Point3 right = this->direction == tri::pointing::UP
                           ? vert_result.pointsR[0]
                           : vert_result.pointsL[0];
  Point3::lazy_row_points_result horz_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_row_points_gnomonic(lower_horz, left, right,
                                             this->direction ==
                                                     tri::pointing::UP
                                                 ? vert_result.lower_indx
                                                 : nd - vert_result.lower_indx,
                                             0, lower_horz, lower_horz)
          : Point3::lazy_row_points_quaternion(
                lower_horz, left, right,
                this->direction == tri::pointing::UP
                    ? vert_result.lower_indx
                    : nd - vert_result.lower_indx,
                0, lower_horz, lower_horz);

  Point3 spherified = horz_result.row_points[0];
  if (rotation == ico::rotation_method::gnomonic) {
    spherified.spheriphy();
  }

  return spherified;
};

bool Triangle::contains_point(Point3 &point) const {
  // vec and tri intersection point
  const Point3 intersection = this->plane_intersection(point);

  if (intersection.on_opposite_side(point)) {
    return false;
  }
  // check if coords ok
  if (!intersection.is_valid()) {
    return false;
  }
  // calc tri area
  const long double tri_area = this->area();
  // if any sub tri area is bigger than thisArea it means point outside of
  // triangle
  const long double pAB_area = Triangle(this->A, this->B, intersection).area();
  if (pAB_area > tri_area + 0.01) {
    return false;
  }
  const long double pBC_area = Triangle(intersection, this->B, this->C).area();
  if (pBC_area > tri_area + 0.01) {
    return false;
  }
  const long double pCA_area = Triangle(this->A, intersection, this->C).area();
  if (pCA_area > tri_area + 0.01) {
    return false;
  }
  // round and check if equal enough
  const long double combined_area = pAB_area + pBC_area + pCA_area;
  const int equal_nuff = hexmapf::equal_enough(tri_area, combined_area);

  return equal_nuff;
};

Point3 Triangle::plane_intersection(Point3 vec) const {
  // x component
  const long double l = (this->A.y - this->B.y) * (this->C.z - this->B.z) -
                        (this->A.z - this->B.z) * (this->C.y - this->B.y);
  // y component
  const long double m = (this->A.z - this->B.z) * (this->C.x - this->B.x) -
                        (this->A.x - this->B.x) * (this->C.z - this->B.z);
  // z component
  const long double n = (this->A.x - this->B.x) * (this->C.y - this->B.y) -
                        (this->C.x - this->B.x) * (this->A.y - this->B.y);
  // finds v - variable used to find point along vector from origin to p on
  // plane
  const long double v_numer = l * this->A.x + m * this->A.y + n * this->A.z;
  const long double v_denom = l * vec.x + m * vec.y + n * vec.z;
  const long double v = v_numer / v_denom;
  // parametric equation for line along vec (from origin)
  // t_ is for temp
  const long double t_x = vec.x * v;
  const long double t_y = vec.y * v;
  const long double t_z = vec.z * v;
  return Point3(t_x, t_y, t_z);
};

long double Triangle::area() const {
  Point3 AB = this->B;
  AB.subtract(this->A);
  Point3 BC = this->C;
  BC.subtract(this->B);
  Point3 temp = AB;
  temp.cross(BC);

  const long double mag = temp.mag();

  return mag / 2.0;
}