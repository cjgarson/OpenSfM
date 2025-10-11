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

std::unordered_map<sfmmap::ShotId, int> CountTracksPerShot(
    const sfmmap::TracksManager& manager,
    const std::vector<sfmmap::ShotId>& shots,
    const std::vector<sfmmap::TrackId>& tracks);

void AddConnections(sfmmap::TracksManager& manager,
                    const sfmmap::ShotId& shot_id,
                    const std::vector<sfmmap::TrackId>& connections);

void RemoveConnections(sfmmap::TracksManager& manager,
                       const sfmmap::ShotId& shot_id,
                       const std::vector<sfmmap::TrackId>& connections);

}  // namespace tracks_helpers
}  // namespace sfm
