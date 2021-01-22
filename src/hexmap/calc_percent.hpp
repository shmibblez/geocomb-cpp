#include "constants.hpp"
#include "point3.hpp"
#include "triangle.hpp"

class CalcPercent {

public:
  struct calc_percent_result {
    double percent_CA;
    double percent_CB;
  };

  struct vec_side_components_result {
    Point3 vec_CA;
    Point3 vec_CB;
  };

  /**
   * calc percent of p along tri sides
   * @param tri triangle
   * @param p point inside [tri]
   * @returns percents along CA and CB
   **/
  static calc_percent_result gnomonic(const Triangle &tri, const Point3 &p);

  /**
   * not ready yet!!!!!!!!
   * @param tri triangle
   * @param p point inside [tri]
   * @returns percents along CA and CB
   **/
  static calc_percent_result quaternion(const Triangle &tri, const Point3 &p);

  /**
   * @param tri triangle
   * @param p point whose vector intersects tri plane and is within bounds
   * @param i intersection of p vector in triangle plane
   * @returns vectors that are on triangle sides and add up to point from tri.C,
   * [vec on CA, vec on CB]
   */
  static vec_side_components_result vec_side_components(const Triangle &tri,
                                                        const Point3 &i);
};