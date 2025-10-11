#include <map/defines.h>
#include <map/tracks_manager.h>
#include <map/map.h>
#include <sfm/retriangulation.h>

#include <limits>
#include <unordered_set>

namespace sfm {
namespace retriangulation {

// Rigid+scale mapping “from” shot frame into “to” shot frame, constructed
// with the elements we already have available in the codebase.
static inline geometry::Similarity MakeSimilarityFromTo(
    const geometry::Pose& shot_from_pose,  // already shifted to 'to' LLA
    const geometry::Pose& shot_to_pose,
    double inv_from_scale /* 1.0/shot_from.scale or 1.0 if zero */) {
  // R_to_from maps coordinates expressed in the “to” camera frame into the
  // “from” camera frame
  const Mat3d R_to_from =
      shot_from_pose.RotationCameraToWorld() * shot_to_pose.RotationWorldToCamera();

  // t_from_to is the “from” camera origin expressed in the “to” coordinates,
  // respecting the scale convention in the original implementation
  const Vec3d t_from_to =
      -inv_from_scale * R_to_from * shot_to_pose.GetOrigin() +
      shot_from_pose.GetOrigin();

  // The constructor geometry::Similarity(R, t, s) is available in-tree
  return geometry::Similarity(R_to_from, t_from_to, inv_from_scale);
}

void RealignMaps(const sfmmap::Map& map_from, sfmmap::Map& map_to,
                 bool update_points) {
  const auto& map_from_shots = map_from.GetShots();

  const auto& from_ref = map_from.GetTopocentricConverter();
  const auto& to_ref   = map_to.GetTopocentricConverter();
  const Vec3d from_to_offset = to_ref.ToTopocentric(from_ref.GetLlaRef());

  // record transforms that remap points of 'to' relative to 'from'
  std::unordered_map<sfmmap::ShotId, geometry::Similarity> from_to_transforms;
  for (const auto& shot_to_pair : map_to.GetShots()) {
    const auto& shot_id = shot_to_pair.first;
    const auto& shot_to = shot_to_pair.second;

    if (!map_from.HasShot(shot_id)) {
      continue;
    }

    const auto& shot_from = map_from.GetShot(shot_id);

    // clone then nudge “from” shot into “to” topocentric LLA
    geometry::Pose shot_from_pose = *shot_from.GetPose();
    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    const geometry::Pose shot_to_pose = *shot_to.GetPose();

    const double inv_scale =
        (shot_from.scale != 0.0) ? (1.0 / shot_from.scale) : 1.0;

    from_to_transforms[shot_id] =
        MakeSimilarityFromTo(shot_from_pose, shot_to_pose, inv_scale);
  }

  // remap points of 'to' using the computed transforms if requested
  if (update_points) {
    constexpr auto max_dbl = std::numeric_limits<double>::max();
    for (auto& lm_pair : map_to.GetLandmarks()) {
      auto& landmark = lm_pair.second;
      const Vec3d point = landmark.GetGlobalPos();

      std::pair<double, sfmmap::ShotId> best_shot = {max_dbl, ""};
      for (const auto& shot_obs_pair : landmark.GetObservations()) {
        const auto* shot = shot_obs_pair.first;
        if (map_from_shots.find(shot->GetId()) == map_from_shots.end()) {
          continue;
        }
        const Vec3d ray   = point - shot->GetPose()->GetOrigin();
        const double d2   = ray.squaredNorm();
        if (d2 < best_shot.first) best_shot = {d2, shot->GetId()};
      }

      if (best_shot.first == max_dbl) continue;

      const auto xform_it = from_to_transforms.find(best_shot.second);
      if (xform_it == from_to_transforms.end()) continue;

      landmark.SetGlobalPos(xform_it->second.Transform(landmark.GetGlobalPos()));
    }
  }

  // synchronize shots and cameras
  std::unordered_set<sfmmap::ShotId> to_delete;
  for (auto& shot_to_pair : map_to.GetShots()) {
    const auto& shot_id = shot_to_pair.first;
    auto& shot_to       = shot_to_pair.second;

    if (!map_from.HasShot(shot_id)) {
      to_delete.insert(shot_id);
      continue;
    }

    const auto& shot_from = map_from.GetShot(shot_id);
    auto& camera_to = map_to.GetCamera(shot_to.GetCamera()->id);

    // copy intrinsics from map_from
    camera_to.SetParametersValues(shot_from.GetCamera()->GetParametersValues());
    // copy ad-hoc metadata
    shot_to.scale    = shot_from.scale;
    shot_to.merge_cc = shot_from.merge_cc;
  }

  // map rig instances (rig cameras assumed unchanged)
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

  // remove any extra shots
  for (const auto& shot_id : to_delete) {
    map_to.RemoveShot(shot_id);
  }
}

}  // namespace retriangulation
}  // namespace sfm
