#include "calc_percent.hpp"
#include "triangle.hpp"
#include <cmath>

#include <iostream>
#include <string>

using std::cos;
using std::sin;

CalcPercent::vec_side_components_result
CalcPercent::vec_side_components(const Triangle &tri, const Point3 &i) {
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

CalcPercent::calc_percent_result CalcPercent::gnomonic(const Triangle &tri,
                                                       const Point3 &p) {
  int precision = std::numeric_limits<long double>::max_digits10;

  const long double r = constants::radius;
  Point3 original_AB = tri.B;
  original_AB.subtract(tri.A);

  Point3 c_AB = original_AB;
  c_AB.div_by(2.0);
  c_AB.add(tri.A);

  Point3 cent = c_AB;
  cent.subtract(tri.C);
  cent.mult_by(2.0 / 3.0);
  cent.add(tri.C);
  cent.unit();
  cent.mult_by(r);

  const long double alpha = tri.C.angle_between(cent);
  const long double mag_cent = cent.mag();
  const long double mag_h = mag_cent / cos(alpha);
  Point3 A = tri.A;
  A.unit();
  A.mult_by(mag_h);
  Point3 B = tri.B;
  B.unit();
  B.mult_by(mag_h);
  Point3 C = tri.C;
  C.unit();
  C.mult_by(mag_h);
  const Triangle projected_tri = Triangle(A, B, C);
  const Point3 projected_p = projected_tri.plane_intersection(p);

  std::cout << "\nmag_h: " << mag_h << "\nmag_cent: " << mag_cent
            << "\nalpha: " << alpha << "\ncos(alpha): " << cos(alpha) << "\n\n";

  CalcPercent::vec_side_components_result components =
      CalcPercent::vec_side_components(projected_tri, projected_p);

  const long double mag_comp_CA = components.vec_CA.mag();
  const long double mag_comp_CB = components.vec_CB.mag();
  Point3 mag_temp = B;
  mag_temp.subtract(A);
  const long double mag = mag_temp.mag();

  return {.percent_CA = mag_comp_CA / mag, .percent_CB = mag_comp_CB / mag};
}

CalcPercent::calc_percent_result CalcPercent::quaternion(const Triangle &tri,
                                                         const Point3 &p) {
  // throw std::logic_error("CalcPercent->quaternion not ready yet");
  return {.percent_CA = -1, .percent_CB = -1};
}