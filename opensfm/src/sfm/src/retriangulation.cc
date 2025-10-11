#include "map/defines.h"
#include "map/tracks_manager.h"
#include "map/map.h"
#include "sfm/retriangulation.h"

#include <limits>
#include <unordered_set>

using namespace sfmmap;

namespace sfm {
namespace retriangulation {

void RealignMaps(const sfmmap::Map& map_from, sfmmap::Map& map_to,
                 bool update_points) {
  const auto& map_from_shots = map_from.GetShots();

  const auto& from_ref = map_from.GetTopocentricConverter();
  const auto& to_ref = map_to.GetTopocentricConverter();
  const auto from_to_offset = to_ref.ToTopocentric(from_ref.GetLlaRef());

  std::unordered_map<sfmmap::ShotId, geometry::Similarity> from_to_transforms;
  for (const auto& shot_to_pair : map_to.GetShots()) {
    const auto& shot_id = shot_to_pair.first;
    const auto& shot_to = shot_to_pair.second;

    if (!map_from.HasShot(shot_id)) continue;

    const auto& shot_from = map_from.GetShot(shot_id);
    auto shot_from_pose = *shot_from.GetPose();
    const auto shot_to_pose = *shot_to.GetPose();

    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    const double scale = (shot_from.scale != 0.0) ? (1.0 / shot_from.scale) : 1.0;
    const Mat3d R_to_from = shot_from_pose.RotationCameraToWorld() *
                            shot_to_pose.RotationWorldToCamera();
    const Vec3d t_from_to = -scale * R_to_from * shot_to_pose.GetOrigin() +
                            shot_from_pose.GetOrigin();

    from_to_transforms[shot_id] = geometry::Similarity(R_to_from, t_from_to, scale);
  }

  if (update_points) {
    constexpr auto max_dbl = std::numeric_limits<double>::max();
    for (auto& lm_pair : map_to.GetLandmarks()) {
      auto& landmark = lm_pair.second;
      const auto point = landmark.GetGlobalPos();

      std::pair<double, sfmmap::ShotId> best_shot = {max_dbl, ""};
      for (const auto& shot_obs_pair : landmark.GetObservations()) {
        const auto* shot = shot_obs_pair.first;
        if (map_from_shots.find(shot->GetId()) == map_from_shots.end()) continue;

        const Vec3d ray = point - shot->GetPose()->GetOrigin();
        const double dist2 = ray.squaredNorm();
        if (dist2 < best_shot.first) best_shot = {dist2, shot->GetId()};
      }

      if (best_shot.first == max_dbl) continue;
      const auto& ref_shot = best_shot.second;
      const auto transform_it = from_to_transforms.find(ref_shot);
      if (transform_it == from_to_transforms.end()) continue;

      landmark.SetGlobalPos(transform_it->second.Transform(landmark.GetGlobalPos()));
    }
  }

  std::unordered_set<sfmmap::ShotId> to_delete;
  for (auto& shot_to_pair : map_to.GetShots()) {
    const auto& shot_id = shot_to_pair.first;
    auto& shot_to = shot_to_pair.second;

    if (!map_from.HasShot(shot_id)) {
      to_delete.insert(shot_id);
      continue;
    }

    const auto& shot_from = map_from.GetShot(shot_id);
    auto& camera_to = map_to.GetCamera(shot_to.GetCamera()->id);

    camera_to.SetParametersValues(shot_from.GetCamera()->GetParametersValues());
    shot_to.scale = shot_from.scale;
    shot_to.merge_cc = shot_from.merge_cc;
  }

  for (auto& rig_instance_pair : map_to.GetRigInstances()) {
    auto& rig_instance_to = rig_instance_pair.second;
    for (const auto& shot_pair : rig_instance_to.GetShots()) {
      const auto& shot_id = shot_pair.first;
      if (map_from_shots.find(shot_id) != map_from_shots.end()) {
        const auto& shot_from = map_from_shots.at(shot_id);
        auto& to_pose = rig_instance_to.GetPose();

        to_pose = shot_from.GetRigInstance()->GetPose();
        to_pose.SetOrigin(to_pose.GetOrigin() + from_to_offset);
        break;
      }
    }
  }

  for (const auto& shot_id : to_delete) {
    map_to.RemoveShot(shot_id);
  }
}

}  // namespace retriangulation
}  // namespace sfm
