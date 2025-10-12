#pragma once

#include <map/defines.h>       // ShotId, RigCameraId, RigInstanceId
#include <geometry/camera.h>   // ::geometry::Pose (via camera or pose header)
#include <geometry/pose.h>
#include <unordered_map>
#include <set>
#include <string>

namespace sfmmap {

// Forward declaration to avoid heavy includes
class Shot;

/**
 * A rig camera with a fixed pose relative to its rig instance.
 */
struct RigCamera {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RigCameraId id;
  ::geometry::Pose pose;
};

/**
 * A rig instance that owns multiple shots (one per rig camera).
 * Stores a global pose and maps ShotId -> (RigCamera*, Shot*).
 */
class RigInstance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RigInstance() : id_(), pose_() {}
  explicit RigInstance(const RigInstanceId& rid) : id_(rid) {}

  const RigInstanceId& GetId() const { return id_; }

  // Global pose of this rig instance (world←instance)
  const ::geometry::Pose& GetPose() const { return pose_; }
  ::geometry::Pose& GetPose() { return pose_; }
  void SetPose(const ::geometry::Pose& p) { pose_ = p; }

  // Manage association of shots to this rig instance
  void AddShot(RigCamera* rig_camera, Shot* shot);
  void RemoveShot(const ShotId& shot_id);

  // Accessors
  const std::unordered_map<ShotId, Shot*>& GetShots() const { return shots_; }
  const std::unordered_map<ShotId, RigCamera*>& GetRigCameras() const { return shot_to_rig_; }

  // Utility methods
  std::set<ShotId> GetShotIDs() const;
  void UpdateInstancePoseWithShot(const ShotId& shot_id, const ::geometry::Pose& new_shot_pose);
  void UpdateRigCameraPose(const RigCameraId& rig_camera_id, const ::geometry::Pose& new_pose);

 private:
  RigInstanceId id_;
  ::geometry::Pose pose_;
  std::unordered_map<ShotId, Shot*> shots_;            // shot ID -> Shot*
  std::unordered_map<ShotId, RigCamera*> shot_to_rig_; // shot ID -> RigCamera*
};

}  // namespace sfmmap
