#include <sfm/tracks_helpers.h>
#include <map/tracks_manager.h>

#include <unordered_map>
#include <unordered_set>

namespace sfm {
namespace tracks_helpers {

std::unordered_map<sfmmap::ShotId, int> CountTracksPerShot(
    const sfmmap::TracksManager& manager,
    const std::vector<sfmmap::ShotId>& shots,
    const std::vector<sfmmap::TrackId>& tracks) {
  std::unordered_set<sfmmap::TrackId> tracks_set;
  for (const auto& track : tracks) {
    tracks_set.insert(track);
  }
  std::unordered_map<sfmmap::ShotId, int> counts;
  for (const auto& shot : shots) {
    const auto& observations = manager.GetShotObservations(shot);

    int sum = 0;
    for (const auto& obs : observations) {
      const auto& trackID = obs.first;
      if (tracks_set.find(trackID) == tracks_set.end()) {
        continue;
      }
      ++sum;
    }
    counts[shot] = sum;
  }
  return counts;
}

void AddConnections(sfmmap::TracksManager& manager, const sfmmap::ShotId& shot_id,
                    const std::vector<sfmmap::TrackId>& connections) {
  for (const auto& track_id : connections) {
    manager.AddObservation(shot_id, track_id, sfmmap::Observation());
  }
}

void RemoveConnections(sfmmap::TracksManager& manager, const sfmmap::ShotId& shot_id,
                       const std::vector<sfmmap::TrackId>& connections) {
  for (const auto& track_id : connections) {
    manager.RemoveObservation(shot_id, track_id);
  }
}

}  // namespace tracks_helpers
}  // namespace sfm
