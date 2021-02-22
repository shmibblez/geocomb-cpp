#include "phex.hpp"

Phex::Phex(std::vector<GPoint3> points, GPoint3 center)
    : points(points), center(center){};

Phex::~Phex() {}

std::vector<Phex>
Phex::all_phexes(Icosahedron::all_icosahedron_points all_points) {
  // generate all points
  const std::vector<GPoint3> centers = Phex::all_phex_centers(all_points);
  // init phexes
  std::vector<Phex> phexes;
  // generate phexes from all points & phex centers
  for (const GPoint3 &c : centers) {
    // first hex center index is -> row_num % 3
    phexes.push_back(Phex(Phex::not_lazy_surrounding_points(all_points, c), c));
  }
  return phexes;
};

std::vector<GPoint3>
Phex::all_phex_centers(Icosahedron::all_icosahedron_points all_points) {
  const int tri_div_count = (all_points.size() - 1) / 3;
  std::vector<GPoint3> centers;

  for (unsigned int fl = 0; fl < all_points.size(); fl++) {
    // first hex center index is -> row_num % 3
    // const firstHexCenterIndx = fl % 3;
    int first_hex_center_indx = -1;
    if ((signed)fl < tri_div_count) {
      first_hex_center_indx = hexmapf::round_up(fl, 3) - fl;
      int count = 0;
      for (unsigned int sl = 0; sl < all_points[fl].size(); sl++) {
        if (sl % fl == 0) {
          // if at triangle edge, reset count
          count = first_hex_center_indx;
        }
        if (count == 0) {
          // if count === 0, it means at triangle center, add point and reset
          // counter
          centers.push_back(all_points[fl][sl]);
          count = 3;
        }
        count--;
      }
    } else if ((signed)fl > tri_div_count * 2) {
      first_hex_center_indx = fl % 3;
      int count = 0;
      for (unsigned int sl = 0; sl < all_points[fl].size(); sl++) {
        if (sl % (hexmapf::round_up(fl, tri_div_count) - fl) == 0) {
          count = first_hex_center_indx;
        }
        if (count == 0) {
          // if count === 0, it means at triangle center, add point and reset
          // counter
          centers.push_back(all_points[fl][sl]);
          count = 3;
        }
        count--;
      }
    } else {
      first_hex_center_indx = fl % 3;
      for (unsigned int sl = first_hex_center_indx; sl < all_points[fl].size();
           sl += 3) {
        centers.push_back(all_points[fl][sl]);
      }
    }
  }
  return centers;
};

// TODO: for lazy version, just generate points from row & col
// will also need res, & make sure p would be valid for res (within row & col
// bounds)
std::vector<GPoint3> Phex::not_lazy_surrounding_points(
    Icosahedron::all_icosahedron_points all_points, GPoint3 p) {
  if (p.is_pc) {
    // north, return row
    if (p.row == 0) {
      return all_points[1];
    }
    if (p.row == (signed)all_points.size() - 1) {
      return all_points[all_points.size() - 2];
    }
  }
  int left_indx = p.col - 1;
  int right_indx = p.col + 1;
  if ((signed)all_points[p.row].size() <= right_indx) {
    right_indx = 0;
  }
  if (left_indx < 0) {
    left_indx = all_points[p.row].size() - 1;
  }

  const int tri_div = p.res * 3;
  // replace with diy pointer if doesn't work
  std::unique_ptr<GPoint3> l_cent, l_abov, r_abov, r_cent, r_belo, l_belo;

  if (p.row < tri_div) {
    // top pent phexes
    int offset = trunc(p.col / p.row);
    if (p.col % p.row == 0) {
      // if point is on edge, special case
      int l_cent_indx = p.col + offset - 1;
      if (l_cent_indx < 0) {
        l_cent_indx = all_points[p.row + 1].size() - 1;
      }
      int l_abov_indx = p.col - 1;
      if (l_abov_indx < 0) {
        l_abov_indx = all_points[p.row].size() - 1;
      }
      l_cent.reset(&all_points[p.row + 1][l_cent_indx]);
      l_abov.reset(&all_points[p.row][l_abov_indx]);
      r_abov.reset(&all_points[p.row - 1][p.col - offset]);
      r_cent.reset(&all_points[p.row][right_indx]);
      r_belo.reset(&all_points[p.row + 1][p.col + offset + 1]);
      l_belo.reset(&all_points[p.row + 1][p.col + offset]);
    } else {
      // if point not on edge, do stuff
      int r_abov_indx = p.col - offset;
      if (r_abov_indx >= (signed)all_points[p.row - 1].size()) {
        r_abov_indx = 0;
      }
      l_cent.reset(&all_points[p.row][left_indx]);
      l_abov.reset(&all_points[p.row - 1][left_indx - offset]);
      r_abov.reset(&all_points[p.row - 1][r_abov_indx]);
      r_cent.reset(&all_points[p.row][right_indx]);
      r_belo.reset(&all_points[p.row + 1][p.col + offset + 1]);
      l_belo.reset(&all_points[p.row + 1][p.col + offset]);
    }
    return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo, *l_belo};
  } else if (p.row > tri_div && p.row < tri_div * 2) {
    // center phexes
    l_cent.reset(&all_points[p.row][left_indx]);
    l_abov.reset(&all_points[p.row - 1][p.col]);
    r_abov.reset(&all_points[p.row - 1][right_indx]);
    r_cent.reset(&all_points[p.row][right_indx]);
    r_belo.reset(&all_points[p.row + 1][p.col]);
    l_belo.reset(&all_points[p.row + 1][left_indx]);
    return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo, *l_belo};
  } else if (p.row > tri_div * 2) {
    // bottom pent phexes
    const int offset = trunc(p.col / (tri_div * 3 - p.row));
    if (p.col % (tri_div * 3 - p.row) == 0) {
      // if point is on edge, special case
      int l_cent_indx = p.col + offset + 1;
      if (l_cent_indx < 0) {
        l_cent_indx = all_points[p.row - 1].size() - 1;
      }
      int l_belo_indx = p.col - 1;
      if (l_belo_indx < 0) {
        l_belo_indx = all_points[p.row].size() - 1;
      }

      l_cent.reset(&all_points[p.row - 1][l_cent_indx]);
      l_abov.reset(&all_points[p.row - 1][p.col + offset]);
      r_abov.reset(&all_points[p.row - 1][p.col + offset + 1]);
      r_cent.reset(&all_points[p.row][right_indx]);
      r_belo.reset(&all_points[p.row + 1][p.col - offset]);
      l_belo.reset(&all_points[p.row][l_belo_indx]);
    } else {
      // if point not on edge, do stuff
      int r_belo_indx = p.col - offset;
      if (r_belo_indx >= (signed)all_points[p.row + 1].size()) {
        r_belo_indx = 0;
      }
      l_cent.reset(&all_points[p.row][left_indx]);
      l_abov.reset(&all_points[p.row - 1][p.col + offset]);
      r_abov.reset(&all_points[p.row - 1][p.col + offset + 1]);
      r_cent.reset(&all_points[p.row][right_indx]);
      r_belo.reset(&all_points[p.row + 1][r_belo_indx]);
      l_belo.reset(&all_points[p.row + 1][left_indx - offset]);
    }
    return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo, *l_belo};
  } else if (p.row / tri_div == 1) {
    // top edge phexes
    const int offset_top = trunc(trunc((p.col / all_points[p.row].size()) *
                                       all_points[p.row - 1].size()) /
                                 (p.row - 1));

    l_cent.reset(&all_points[p.row][left_indx]);
    l_abov.reset(&all_points[p.row - 1][left_indx - offset_top]);
    r_abov.reset(&all_points[p.row - 1][p.col - offset_top]);
    r_cent.reset(&all_points[p.row][right_indx]);
    r_belo.reset(&all_points[p.row + 1][p.col]);
    l_belo.reset(&all_points[p.row + 1][left_indx]);
    if (p.is_pc) {
      return {*l_cent, *r_abov, *r_cent, *r_belo, *l_belo};
    } else {
      return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo, *l_belo};
    }
  } else if (p.row / (tri_div * 2) == 1) {
    // bottom edge phexes
    const int offset_top = trunc(trunc((p.col / all_points[p.row].size()) *
                                       all_points[p.row + 1].size()) /
                                 (tri_div - 1));

    l_cent.reset(&all_points[p.row][left_indx]);
    l_abov.reset(&all_points[p.row - 1][p.col]);
    r_abov.reset(&all_points[p.row - 1][right_indx]);
    r_cent.reset(&all_points[p.row][right_indx]);
    r_belo.reset(&all_points[p.row + 1][p.col - offset_top]);
    l_belo.reset(&all_points[p.row + 1][left_indx - offset_top]);
    if (p.is_pc) {
      return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo};
    } else {
      return {*l_cent, *l_abov, *r_abov, *r_cent, *r_belo, *l_belo};
    }
  }
  throw std::logic_error("this should not happen");
};