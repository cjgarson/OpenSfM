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

  std::unordered_set<sfmmap::TrackId> tracks_set(tracks.begin(), tracks.end());

  std::unordered_map<sfmmap::ShotId, int> counts;
  for (const auto& shot : shots) {
    const auto& observations = manager.GetShotObservations(shot);
    int sum = 0;
    for (const auto& obs : observations) {
      const auto& track_id = obs.first;
      if (tracks_set.find(track_id) == tracks_set.end()) continue;
      ++sum;
    }
    counts[shot] = sum;
  }
  return counts;
}

void AddConnections(sfmmap::TracksManager& manager,
                    const sfmmap::ShotId& shot_id,
                    const std::vector<sfmmap::TrackId>& connections) {
  for (const auto& connection : connections) {
    manager.AddObservation(shot_id, connection, sfmmap::Observation());
  }
}

void RemoveConnections(sfmmap::TracksManager& manager,
                       const sfmmap::ShotId& shot_id,
                       const std::vector<sfmmap::TrackId>& connections) {
  for (const auto& connection : connections) {
    manager.RemoveObservation(shot_id, connection);
  }
}

}  // namespace tracks_helpers
}  // namespace sfm
