#include <vector>

class Point3 {

public:
  int x;
  int y;
  int z;
  int tri_num;
  bool is_vert;
  Point3(int x, int y, int z, bool is_vert = false, int tri_num = -1);

  // angle between vectors (origin, this) and (origin, p)
  double angle_between(const Point3 &p);
  /**
   * @param around vec to rotate around -> vec is from origin to point
   * @param rad rads to rotate
   * @note modifies obj **/
  void rotate(const Point3 &around, const double &rad);
  /**
   * magnitude from origin to point **/
  double mag();
  /**
   * add another vector
   * @note modifies obj */
  void add(const Point3 &p);
  /**
   * subtract another vector
   * @note modifies obj */
  void subtract(const Point3 &p);
  /**
   * dot product */
  double dot(const Point3 &p);
  /**
   * cross product
   * @note modifies obj **/
  void cross(const Point3 &p);
  /**
   * convert into unit vec
   * @note modifies obj **/
  void unit();
  /**
   * multiply by scalar
   * @note modifies obj **/
  void mult_by(const double num);
  /**
   * multiply by scalar
   * @note modifies obj **/
  void div_by(const double num);
  /**
   * distance between vectors **/
  double distance(const Point3 &p);
  /**
   * check if points on opposite sides of globe
   * @note loosely determined -> points don't need to be 180 deg apart, if 90
   * deg apart will return true **/
  bool on_opposite_side(const Point3 &p);
  /**
   * check if point is valid (all coords finite & valid numbers) **/
  bool is_valid();
  /**
   * @returns copy of closest point in point arr */
  Point3 closest_point(std::vector<Point3> points);

  /**
   * NOT IMPLEMENED (list functions that haven't been implemented yet & are in
   * hexmap js)
   *
   * pointNum(GPoint3 p)
   **/
};

class GPoint3 : public Point3 {

public:
  int res;
  int row;
  int col;
  GPoint3(double x, double y, double z, int res, int row, int col,
          bool is_vert = false, int tri_num = -1);
};

class Quaternion : public Point3 {

public:
  int w;
  Quaternion(double x, double y, double z, double w);

  /**
   * convert to unit quaternion */
  void unit();
  /**
   * get quaternion magnitude */
  double mag();
  /**
   * multiply with another quaternion
   * @note modifies obj */
  void multiply(const Quaternion &q);
};
