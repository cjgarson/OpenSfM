#pragma once

#include <map/defines.h>       // ShotId, RigCameraId, RigInstanceId
#include <geometry/camera.h>   // geometry::Pose (via camera or pose header in your tree)
#include <geometry/pose.h>
#include <unordered_map>
#include <set>
#include <string>

namespace sfmmap {

// Forward declarations to avoid heavy includes and circular deps.
class Shot;

/**
 * A rig camera (lever-arm) with a fixed pose relative to its rig instance.
 * Keep the ID in this struct; do not rely on external .id members.
 */
struct RigCamera {
  RigCameraId id;           // identifier (string)
  geometry::Pose pose;      // T_camera_in_instance (lever-arm)
};

/**
 * A rig instance that owns multiple shots (one per rig camera).
 * Stores a global pose and the association ShotId -> (RigCamera*, Shot*).
 */
class RigInstance {
 public:
  explicit RigInstance(const RigInstanceId& rid) : id_(rid) {}

  // Identity
  const RigInstanceId& GetId() const { return id_; }

  // Global pose of the rig instance
  const geometry::Pose& GetPose() const { return pose_; }
  geometry::Pose& GetPose() { return pose_; }
  void SetPose(const geometry::Pose& p) { pose_ = p; }

  // Association management
  void AddShot(RigCamera* rig_camera, Shot* shot);
  void RemoveShot(const ShotId& shot_id);

  // Accessors for callers that want the mapping
  // - Shots: ShotId -> Shot*
  const std::unordered_map<ShotId, Shot*>& GetShots() const { return shots_; }
  // - Rig cameras used by this instance: ShotId -> RigCamera*
  const std::unordered_map<ShotId, RigCamera*>& GetRigCameras() const { return shot_to_rig_; }

  // Convenience helpers frequently used by SFM and BA code
  std::set<ShotId> GetShotIDs() const;

  // Simple update helpers used by some call-sites
  void UpdateInstancePoseWithShot(const ShotId& shot_id, const geometry::Pose& new_shot_pose);
  void UpdateRigCameraPose(const RigCameraId& rig_camera_id, const geometry::Pose& new_pose);

 private:
  RigInstanceId id_;

  // Global pose of this physical rig instance (world←instance)
  geometry::Pose pose_;

  // Mappings for the photos belonging to this instance
  std::unordered_map<ShotId, Shot*> shots_;                 // shot id -> Shot*
  std::unordered_map<ShotId, RigCamera*> shot_to_rig_;      // shot id -> RigCamera*
};

} // namespace map
