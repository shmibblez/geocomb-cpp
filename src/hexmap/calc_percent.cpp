#include "calc_percent.hpp"
#include "triangle.hpp"
#include <cmath>

#include <iomanip>
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

  const double beta = u_CA.angle_between(CI);
  const double alpha = CI.angle_between(u_CB);
  const double phi = constants::PI - beta - alpha;

  // law of sines to find missing magnitudes & lengths
  const double mag_CI = CI.mag();
  const double mag_CB = (mag_CI * sin(beta)) / sin(phi);
  const double mag_CA = (mag_CI * sin(alpha)) / sin(phi);

  std::cout << "u_CB coords:"
            << "\n  x: " << u_CB.x << "\n  y: " << u_CB.y << "\n  z: " << u_CB.z
            << "\n";
  std::cout << "mag_CB: " << mag_CB << "\n";

  Point3 vec_CA = u_CA;
  vec_CA.mult_by(mag_CA);
  Point3 vec_CB = u_CB;
  vec_CB.mult_by(mag_CB);
  return {.vec_CA = vec_CA, .vec_CB = vec_CB};
}

CalcPercent::calc_percent_result CalcPercent::gnomonic(const Triangle &tri,
                                                       const Point3 &p) {
  int precision = std::numeric_limits<double>::max_digits10;
  std::cout.precision(precision);
  // std::cout << "tri.A coords:"
  //           << "\n  x: " << std::fixed << tri.A.x << "\n  y: " << tri.A.y
  //           << "\n  z: " << std::fixed << tri.A.z << "\n";
  // std::cout << "tri.B coords:"
  //           << "\n  x: " << tri.B.x << "\n  y: " << tri.B.y
  //           << "\n  z: " << tri.B.z << "\n";
  // std::cout << "tri.C coords:"
  //           << "\n  x: " << tri.C.x << "\n  y: " << tri.C.y
  //           << "\n  z: " << tri.C.z << "\n";
  const double r = constants::radius;
  Point3 original_AB = tri.B;
  original_AB.subtract(tri.A);
  // std::cout << "original_AB coords:"
  //           << "\n  x: " << original_AB.x << "\n  y: " << original_AB.y
  //           << "\n  z: " << original_AB.z << "\n";
  Point3 c_AB = original_AB;
  c_AB.div_by(2.0);
  c_AB.add(tri.A);
  // std::cout << "c_AB coords:"
  //           << "\n  x: " << c_AB.x << "\n  y: " << c_AB.y << "\n  z: " <<
  //           c_AB.z
  //           << "\n";
  Point3 cent = c_AB;
  cent.subtract(tri.C);
  cent.mult_by(2.0 / 3.0);
  cent.add(tri.C);
  cent.unit();
  cent.mult_by(r);
  // std::cout << "cent coords:"
  //           << "\n  x: " << cent.x << "\n  y: " << cent.y << "\n  z: " <<
  //           cent.z
  //           << "\n";
  const double alpha = tri.C.angle_between(cent);
  const double mag_cent = cent.mag();
  const double mag_h = mag_cent / cos(alpha);
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

  std::cout << "alpha: " << alpha << ", mag_cent: " << mag_cent
            << ", mag_h: " << mag_cent << "\n";

  // std::cout << "p (received point) coords:"
  //           << "\n  x: " << p.x << "\n  y: " << p.y << "\n  z: " << p.z <<
  //           "\n";

  // std::cout << "projected_p coords:"
  //           << "\n  x: " << projected_p.x << "\n  y: " << projected_p.y
  //           << "\n  z: " << projected_p.z << "\n";

  // std::cout << "projected_tri coords:"
  //           << "\n--A coords:"
  //           << "\n  x: " << A.x << "\n  y: " << A.y << "\n  z: " << A.z
  //           << "\n--B coords:"
  //           << "\n  x: " << B.x << "\n  y: " << B.y << "\n  z: " << B.z
  //           << "\n--C coords:"
  //           << "\n  x: " << C.x << "\n  y: " << C.y << "\n  z: " << C.z <<
  //           "\n";

  CalcPercent::vec_side_components_result components =
      CalcPercent::vec_side_components(projected_tri, projected_p);

  const double mag_comp_CA = components.vec_CA.mag();
  const double mag_comp_CB = components.vec_CB.mag();
  Point3 mag_temp = B;
  mag_temp.subtract(A);
  const double mag = mag_temp.mag();

  std::cout << "mag_comp_CA: " << mag_comp_CA
            << ", mag_comp_CB: " << mag_comp_CB << "\n";
  std::cout << "vec_CA coords:"
            << "\n  x: " << components.vec_CA.x
            << "\n  y: " << components.vec_CA.y
            << "\n  z: " << components.vec_CA.z << "\n";
  std::cout << "vec_CB coords:"
            << "\n  x: " << components.vec_CB.x
            << "\n  y: " << components.vec_CB.y
            << "\n  z: " << components.vec_CB.z << "\n";

  return {.percent_CA = mag_comp_CA / mag, .percent_CB = mag_comp_CB / mag};
}

CalcPercent::calc_percent_result CalcPercent::quaternion(const Triangle &tri,
                                                         const Point3 &p) {
  throw std::logic_error("CalcPercent->quaternion not ready yet");
}