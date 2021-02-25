#pragma once

#ifndef ICOSAHEDRON_HPP
#define ICOSAHEDRON_HPP

#include "enums.hpp"
#include "point3.hpp"
#include "triangle.hpp"
#include <vector>

class Phex;

class Icosahedron {

public:
  const std::vector<Triangle> tris;
  const ico::map_orientation mo;
  const ico::rotation_method rm;
  Icosahedron(ico::map_orientation orientation = ico::map_orientation::ECEF,
              ico::rotation_method rotation = ico::rotation_method::gnomonic);

  typedef std::vector<std::vector<GPoint3>> all_icosahedron_points;
  typedef std::vector<std::vector<GPoint3>> lazy_icosahedron_points;

  struct hash_properties {
    int res;
    int row;
    int col;
    ico::rotation_method rm;
    ico::map_orientation mo;
  };

  /**
   * @param mo map_orientation
   * @returns map_orientation key for mo
   **/
  static std::string map_orientation_key(ico::map_orientation mo);

  /**
   * @param rm rotation_method
   * @returns rotation_method key for rm
   **/
  static std::string rotation_method_key(ico::rotation_method rm);

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
  Point3 point_from_coords(long double lat, long double lon) const;

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
};

#endif