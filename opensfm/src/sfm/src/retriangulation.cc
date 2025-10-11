#include <sfm/tracks_helpers.h>
#include <map/tracks_manager.h>

#include <unordered_map>
#include <unordered_set>

namespace sfm {
namespace tracks_helpers {

std::unordered_map<sfmmap::map::ShotId, int> CountTracksPerShot(
    const sfmmap::map::TracksManager& manager,
    const std::vector<sfmmap::map::ShotId>& shots,
    const std::vector<sfmmap::map::TrackId>& tracks) {

  std::unordered_set<sfmmap::map::TrackId> tracks_set;
  for (const auto& track : tracks) {
    tracks_set.insert(track);
  }

  std::unordered_map<sfmmap::map::ShotId, int> counts;
  for (const auto& shot : shots) {
    const auto& observations = manager.GetShotObservations(shot);
    int sum = 0;
    for (const auto& obs : observations) {
      const auto& track_id = obs.first;
      if (tracks_set.find(track_id) == tracks_set.end()) {
        continue;
      }
      ++sum;
    }
    counts[shot] = sum;
  }
  return counts;
}

void AddConnections(sfmmap::map::TracksManager& manager,
                    const sfmmap::map::ShotId& shot_id,
                    const std::vector<sfmmap::map::TrackId>& connections) {
  sfmmap::map::Observation observation;
  for (const auto& connection : connections) {
    manager.AddObservation(shot_id, connection, observation);
  }
}

void RemoveConnections(sfmmap::map::TracksManager& manager,
                       const sfmmap::map::ShotId& shot_id,
                       const std::vector<sfmmap::map::TrackId>& connections) {
  for (const auto& connection : connections) {
    manager.RemoveObservation(shot_id, connection);
  }
}

}  // namespace tracks_helpers
}  // namespace sfm
