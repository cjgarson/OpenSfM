#include <map/defines.h>
#include <map/tracks_manager.h>
#include <map/map.h>
#include <sfm/retriangulation.h>

#include <limits>
#include <unordered_set>

namespace sfm {
namespace retriangulation {

void RealignMaps(const sfmmap::Map::Map& map_from,
                 sfmmap::Map::Map& map_to,
                 bool update_points) {
  const auto& map_from_shots = map_from.GetShots();

  const auto& from_ref = map_from.GetTopocentricConverter();
  const auto& to_ref   = map_to.GetTopocentricConverter();
  const auto  from_to_offset = to_ref.ToTopocentric(from_ref.GetLlaRef());

  // Transform from each shot in map_to to its corresponding shot in map_from
  std::unordered_map<sfmmap::Map::ShotId, ::geometry::Similarity> from_to_transforms;
  for (const auto& [shot_id, shot_to] : map_to.GetShots()) {
    if (!map_from.HasShot(shot_id)) continue;

    const auto& shot_from = map_from.GetShot(shot_id);
    auto shot_from_pose = *shot_from.GetPose();
    const auto shot_to_pose = *shot_to.GetPose();

    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    const double scale = (shot_from.scale != 0.0) ? (1.0 / shot_from.scale) : 1.0;
    const Mat3d R_to_from =
        shot_from_pose.RotationCameraToWorld() *
        shot_to_pose.RotationWorldToCamera();
    const Vec3d t_from_to =
        -scale * R_to_from * shot_to_pose.GetOrigin() +
        shot_from_pose.GetOrigin();

    from_to_transforms[shot_id] =
        ::geometry::Similarity(R_to_from, t_from_to, scale);
  }

  // Optionally realign all landmarks
  if (update_points) {
    constexpr auto max_dbl = std::numeric_limits<double>::max();
    for (auto& [lm_id, landmark] : map_to.GetLandmarks()) {
      const auto point = landmark.GetGlobalPos();
      std::pair<double, sfmmap::Map::ShotId> best_shot = {max_dbl, ""};

      for (const auto& [shot, _obs] : landmark.GetObservations()) {
        if (map_from_shots.find(shot->GetId()) == map_from_shots.end()) continue;
        const Vec3d ray = point - shot->GetPose()->GetOrigin();
        const double dist2 = ray.squaredNorm();
        if (dist2 < best_shot.first)
          best_shot = {dist2, shot->GetId()};
      }

      if (best_shot.first == max_dbl) continue;
      const auto& ref_shot = best_shot.second;
      const auto it = from_to_transforms.find(ref_shot);
      if (it == from_to_transforms.end()) continue;

      landmark.SetGlobalPos(it->second.Transform(landmark.GetGlobalPos()));
    }
  }

  // Sync cameras and shots
  std::unordered_set<sfmmap::Map::ShotId> to_delete;
  for (auto& [shot_id, shot_to] : map_to.GetShots()) {
    if (!map_from.HasShot(shot_id)) {
      to_delete.insert(shot_id);
      continue;
    }

    const auto& shot_from = map_from.GetShot(shot_id);
    auto& camera_to = map_to.GetCamera(shot_to.GetCamera()->id);

    camera_to.SetParametersValues(
        shot_from.GetCamera()->GetParametersValues());
    shot_to.scale    = shot_from.scale;
    shot_to.merge_cc = shot_from.merge_cc;
  }

  // Update rig instance poses
  for (auto& [instance_id, rig_instance_to] : map_to.GetRigInstances()) {
    for (const auto& [shot_id, _shot_ptr] : rig_instance_to.GetShots()) {
      if (map_from_shots.find(shot_id) == map_from_shots.end()) continue;
      const auto& shot_from = map_from_shots.at(shot_id);
      auto& to_pose = rig_instance_to.GetPose();
      to_pose = shot_from.GetRigInstance()->GetPose();
      to_pose.SetOrigin(to_pose.GetOrigin() + from_to_offset);
      break;
    }
  }

  for (const auto& shot_id : to_delete)
    map_to.RemoveShot(shot_id);
}

}  // namespace retriangulation
}  // namespace sfm
