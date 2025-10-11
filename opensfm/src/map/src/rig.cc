#include <map/rig.h>
#include <map/shot.h>
#include <geometry/pose.h>
#include <stdexcept>

namespace sfmmap {

void RigInstance::AddShot(RigCamera* rig_camera, Shot* shot) {
  if (!rig_camera || !shot) {
    throw std::invalid_argument("RigInstance::AddShot received null pointers.");
  }

  const ShotId& sid = shot->GetId();
  if (shots_.count(sid)) {
    throw std::runtime_error("RigInstance::AddShot: shot already exists: " + sid);
  }

  shots_[sid] = shot;
  shot_to_rig_[sid] = rig_camera;
}

void RigInstance::RemoveShot(const ShotId& shot_id) {
  shots_.erase(shot_id);
  shot_to_rig_.erase(shot_id);
}

std::set<ShotId> RigInstance::GetShotIDs() const {
  std::set<ShotId> ids;
  for (const auto& kv : shots_) ids.insert(kv.first);
  return ids;
}

void RigInstance::UpdateInstancePoseWithShot(const ShotId& shot_id,
                                             const geometry::Pose& new_shot_pose) {
  // If this rig has exactly one shot, sync rig pose with that shot’s pose
  if (shots_.size() == 1) {
    pose_ = new_shot_pose;
    return;
  }

  auto it = shot_to_rig_.find(shot_id);
  if (it == shot_to_rig_.end()) return;

  RigCamera* rc = it->second;
  if (!rc) return;

  // world←instance = world←camera * (instance←camera)
  pose_ = new_shot_pose.Compose(rc->pose.Inverse());
}

void RigInstance::UpdateRigCameraPose(const RigCameraId& rig_camera_id,
                                      const geometry::Pose& new_pose) {
  for (auto& kv : shot_to_rig_) {
    RigCamera* rc = kv.second;
    if (rc && rc->id == rig_camera_id) {
      rc->pose = new_pose;
      return;
    }
  }
}

}  // namespace map
