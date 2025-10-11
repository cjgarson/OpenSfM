#pragma once

#include <foundation/types.h>
#include <geometry/camera.h>
#include <map/defines.h>
#include <map/observation.h>
#include <map/shot.h>
#include <map/tracks_manager.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace sfm {
namespace tracks_helpers {

std::unordered_map<sfmmap::map::ShotId, int> CountTracksPerShot(
    const sfmmap::map::TracksManager& manager,
    const std::vector<sfmmap::map::ShotId>& shots,
    const std::vector<sfmmap::map::TrackId>& tracks);

void AddConnections(sfmmap::map::TracksManager& manager,
                    const sfmmap::map::ShotId& shot_id,
                    const std::vector<sfmmap::map::TrackId>& connections);

void RemoveConnections(sfmmap::map::TracksManager& manager,
                       const sfmmap::map::ShotId& shot_id,
                       const std::vector<sfmmap::map::TrackId>& connections);

}  // namespace tracks_helpers
}  // namespace sfm
