#pragma once

#ifndef ENUMS_HPP
#define ENUMS_HPP

class tri {

public:
  enum class pointing { UP, DOWN, NA };
  enum class position { TOP, CENTER, BOT, NA };

}; // namespace tri

class ico {

public:
  enum map_orientation { ECEF, dymaxion };
  enum rotation_method { gnomonic, quaternion };

}; // namespace ico

#endif