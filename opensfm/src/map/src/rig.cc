#include <map/rig.h>
#include <map/shot.h>   // for map::Shot (forward decl in header; include in .cc)
#include <stdexcept>

namespace map {

std::set<ShotId> RigInstance::GetShotIDs() const {
  std::set<ShotId> ids;
  ids.reserve(shots_.size());
  for (const auto& kv : shots_) {
    ids.insert(kv.first);
  }
  return ids;
}

void RigInstance::AddShot(RigCamera* rig_camera, Shot* shot) {
  if (!rig_camera || !shot) {
    throw std::invalid_argument("RigInstance::AddShot received null pointer.");
  }
  const ShotId& sid = shot->GetId();
  shots_[sid] = shot;
  shot_to_rig_[sid] = rig_camera;

  // If your Shot class tracks its rig linkage, set it here (safe if methods exist).
  // shot->SetRigInstanceId(id_);
  // shot->SetRigCameraId(rig_camera->id);
}

void RigInstance::RemoveShot(const ShotId& shot_id) {
  shots_.erase(shot_id);
  shot_to_rig_.erase(shot_id);
}

void RigInstance::UpdateInstancePoseWithShot(const ShotId& shot_id,
                                             const geometry::Pose& new_shot_pose) {
  // Minimal behavior: set rig instance pose to provided pose.
  // If you need a stricter relation (instance pose from shot + lever-arm inverse),
  // change to: pose_ = new_shot_pose * (shot_to_rig_[shot_id]->pose).Inverse();
  (void)shot_id; // not used in the minimal implementation
  pose_ = new_shot_pose;
}

void RigInstance::UpdateRigCameraPose(const RigCameraId& rig_camera_id,
                                      const geometry::Pose& new_pose) {
  // Find any entry using this rig camera id and update its pose.
  // (We only store pointers keyed by shot_id, so scan the small map.)
  for (auto& kv : shot_to_rig_) {
    RigCamera* rc = kv.second;
    if (rc && rc->id == rig_camera_id) {
      rc->pose = new_pose;
      return;
    }
  }
  // Not found is not necessarily an error; make it explicit if you prefer:
  // throw std::runtime_error("Rig camera id not found in this instance.");
}

} // namespace map
