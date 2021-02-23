#pragma once

#ifndef ENUMS_HPP
#define ENUMS_HPP

namespace tri {

enum class pointing { UP, DOWN, NA };
enum class position { TOP, CENTER, BOT, NA };

}; // namespace tri

namespace ico {

enum map_orientation { ECEF, dymaxion };
enum rotation_method { gnomonic, quaternion };

}; // namespace ico

#endif