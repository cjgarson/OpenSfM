#pragma once
#include <map/map.h>
#include <map/tracks_manager.h>

namespace sfm {
namespace retriangulation {
void RealignMaps(const sfmmap::Map& reference, sfmmap::Map& to_align,
                 bool update_points);
}  // namespace retriangulation
}  // namespace sfm
