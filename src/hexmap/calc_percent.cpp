#include "calc_percent.hpp"
#include "triangle.hpp"
#include <cmath>

using std::cos;
using std::sin;

CalcPercent::calc_percent_result CalcPercent::gnomonic(const Triangle &tri,
                                                       const Point3 &p) {
  const double r = constants::radius;
  Point3 original_AB = tri.B;
  original_AB.subtract(tri.A);
  Point3 c_AB = original_AB;
  c_AB.div_by(2);
  c_AB.add(tri.A);
  Point3 o_cent = c_AB;
  o_cent.subtract(tri.C);
  o_cent.mult_by(2 / 3);
  o_cent.add(tri.C);
  Point3 cent = o_cent;
  cent.unit();
  cent.mult_by(r);
  const double alpha = tri.C.angle_between(cent);
  const double mag_cent = cent.mag();
  const double mag_h = mag_cent / cos(alpha);
  Point3 A = tri.A;
  A.mult_by(mag_h);
  Point3 B = tri.B;
  B.mult_by(mag_h);
  Point3 C = tri.C;
  C.mult_by(mag_h);
  const Triangle projected_tri = Triangle(A, B, C);
  const Point3 projected_p = projected_tri.plane_intersection(p);

  CalcPercent::vec_side_components_result components =
      CalcPercent::vec_side_components(projected_tri, projected_p);

  const double mag_comp_CA = components.vec_CA.mag();
  const double mag_comp_CB = components.vec_CB.mag();
  Point3 mag_temp = B;
  mag_temp.subtract(A);
  const double mag = mag_temp.mag();

  return {.percent_CA = mag_comp_CA / mag, .percent_CB = mag_comp_CB / mag};
};

CalcPercent::calc_percent_result CalcPercent::quaternion(const Triangle &tri,
                                                         const Point3 &p) {
  throw std::logic_error("CalcPercent::quaternion not ready yet");
};

CalcPercent::vec_side_components_result
CalcPercent::vec_side_components(const Triangle &tri, const Point3 &i) {
  Point3 u_CA = tri.A;
  u_CA.subtract(tri.C);
  u_CA.unit();
  Point3 u_CB = tri.B;
  u_CB.subtract(tri.C);
  Point3 CI = i;
  CI.subtract(tri.C);

  const double beta = u_CA.angle_between(CI);
  const double alpha = CI.angle_between(u_CB);
  const double phi = constants::PI - beta - alpha;

  // law of sines to find missing magnitudes & lengths
  const double mag_CI = CI.mag();
  const double mag_CB = (mag_CI * sin(beta)) / sin(phi);
  const double mag_CA = (mag_CI * sin(alpha)) / sin(phi);

  Point3 vec_CA = u_CA;
  vec_CA.mult_by(mag_CA);
  Point3 vec_CB = u_CB;
  vec_CB.mult_by(mag_CB);
  return {.vec_CA = vec_CA, .vec_CB = vec_CB};
};