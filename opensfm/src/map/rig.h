#pragma once

#include <foundation/optional.h>
#include <geometry/pose.h>
#include <geometry/camera.h>
#include <map/defines.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <exception>
#include <set>
#include <unordered_map>

namespace map {
class Shot;

struct RigCamera {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum RelativeType {
    FIXED = 0,    // All instances of this rig camera have fixed relative poses
    SHARED = 1,   // All instances share the same relative pose (optimized)
    VARIABLE = 2  // Each rig camera has its own pose (optimized)
  };

  RelativeType relative_type{RelativeType::SHARED};

  // Pose of the camera wrt. the rig coordinate frame
  ::geometry::Pose pose;

  // Unique identifier of this RigCamera
  map::RigCameraId id;

  RigCamera() = default;
  RigCamera(const ::geometry::Pose& pose, const map::RigCameraId& id)
      : pose(pose), id(id) {}
};

class RigInstance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  map::RigInstanceId id;

  RigInstance() = default;
  explicit RigInstance(const RigInstanceId instance_id) : id(instance_id) {}

  // Getters
  const std::unordered_map<map::ShotId, map::Shot*>& GetShots() const {
    return shots_;
  }
  std::unordered_map<map::ShotId, map::Shot*>& GetShots() { return shots_; }

  const std::unordered_map<map::ShotId, map::RigCamera*>& GetRigCameras() const {
    return shots_rig_cameras_;
  }
  std::unordered_map<map::ShotId, map::RigCamera*>& GetRigCameras() {
    return shots_rig_cameras_;
  }

  std::set<map::ShotId> GetShotIDs() const;
  size_t NumberOfShots() const;

  // Pose
  const ::geometry::Pose& GetPose() const { return pose_; }
  ::geometry::Pose& GetPose() { return pose_; }
  void SetPose(const ::geometry::Pose& pose) { pose_ = pose; }

  // Add a new shot to this instance
  void AddShot(map::RigCamera* rig_camera, map::Shot* shot);

  // Update instance pose and shot poses wrt. a given shot
  void UpdateInstancePoseWithShot(const map::ShotId& shot_id,
                                  const ::geometry::Pose& shot_pose);

  // Update pose of this instance's RigCamera
  void UpdateRigCameraPose(const map::RigCameraId& rig_camera_id,
                           const ::geometry::Pose& pose);

  void RemoveShot(const map::ShotId& shot_id);

 private:
  // Actual instantiation of a rig: each shot maps to a RigCamera
  std::unordered_map<map::ShotId, map::Shot*> shots_;
  std::unordered_map<map::ShotId, map::RigCamera*> shots_rig_cameras_;

  // Pose of this rig coordinate frame wrt. world coordinates
  ::geometry::Pose pose_;

  // For VARIABLE rigs, each instance has its own copy of RigCameras
  std::unordered_map<
      map::ShotId,
      foundation::OptionalValue<map::RigCamera>,
      std::hash<map::ShotId>,
      std::equal_to<map::ShotId>,
      Eigen::aligned_allocator<
          std::pair<const map::ShotId,
                    foundation::OptionalValue<map::RigCamera>>>> own_rig_cameras_;
};

}  // namespace map
