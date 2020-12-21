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
   * add to other vector **/
  Point3 add(const Point3 &p);
  /**
   * dot product **/
  double dot(const Point3 &p);
  /**
   * cross product
   * @note returns new obj, doesn't modify anything **/
  Point3 cross(const Point3 &p);
  /**
   * convert into unit vec
   * @note modifies obj **/
  Point3 unit();
  /**
   * scale vector (multiply by scalar)
   * @note modifies obj **/
  void scale(const double amount);
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
   * @returns closest point in point arr **/
  Point3 closest_point(std::vector<Point3> points);

  /**
   * NOT IMPLEMENED (list functions that haven't been implemented yet & are in
   *hexmap js)
   **/
};

class GPoint3 : public Point3 {

public:
  int res;
  int row;
  int col;
  GPoint3(int x, int y, int z, int res, int row, int col, bool is_vert = false,
          int tri_num = -1);
};

class Quaternion : public Point3 {

public:
  int w;
  Quaternion(int x, int y, int z, int w);

  // convert to unit quaternion
  void unit();
  // get magnitude
  double mag();
  // multiply with another quaternion
  Quaternion multiply(const Quaternion &q);
};
