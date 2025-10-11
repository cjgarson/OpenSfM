#pragma once

#include <foundation/optional.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
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
    // All instances of this rig camera have their relative poses fixed
    FIXED = 0,
    // All instances share the same relative pose which is optimized
    SHARED = 1,
    // Each rig camera has its own pose which is optimized
    VARIABLE = 2,
  };

  // Specify how to optimize rig relatives
  RelativeType relative_type{RelativeType::SHARED};

  /* Pose of the camera wrt. the rig coordinate frame */
  ::geometry::Pose pose;

  /* Unique identifier of this RigCamera */
  map::Map::RigCameraId id;

  RigCamera() = default;
  RigCamera(const ::geometry::Pose& p, const map::Map::RigCameraId& rid)
      : pose(p), id(rid) {}
};

class RigInstance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  map::Map::RigInstanceId id;

  RigInstance() = default;
  explicit RigInstance(const RigInstanceId instance_id) : id(instance_id) {}

  // Getters
  const std::unordered_map<map::Map::ShotId, map::Map::Shot*>& GetShots() const {
    return shots_;
  }
  std::unordered_map<map::Map::ShotId, map::Map::Shot*>& GetShots() { return shots_; }

  std::unordered_map<map::Map::ShotId, map::Map::RigCamera*>& GetRigCameras() {
    return shots_rig_cameras_;
  }
  const std::unordered_map<map::Map::ShotId, map::Map::RigCamera*>& GetRigCameras() const {
    return shots_rig_cameras_;
  }

  std::set<map::Map::ShotId> GetShotIDs() const;
  size_t NumberOfShots() const;

  // Pose
  const ::geometry::Pose& GetPose() const { return pose_; }
  ::geometry::Pose& GetPose() { return pose_; }
  void SetPose(const ::geometry::Pose& pose) { pose_ = pose; }

  // Add a new shot to this instance
  void AddShot(map::Map::RigCamera* rig_camera, map::Map::Shot* shot);

  // Update instance pose and shot's poses wrt. to a given shot of the instance
  void UpdateInstancePoseWithShot(const map::Map::ShotId& shot_id,
                                  const ::geometry::Pose& shot_pose);

  // Update pose of this instance's RigCamera
  void UpdateRigCameraPose(const map::Map::RigCameraId& rig_camera_id,
                           const ::geometry::Pose& pose);

  // Removal
  void RemoveShot(const map::Map::ShotId& shot_id);

 private:
  // Actual instantiation of a rig : each shot gets mapped to some RigCamera
  std::unordered_map<map::Map::ShotId, map::Map::Shot*> shots_;
  std::unordered_map<map::Map::ShotId, map::Map::RigCamera*> shots_rig_cameras_;

  // Pose of this rig coordinate frame wrt. world coordinates
  ::geometry::Pose pose_;

  // For VARIABLE rigs, each instance has its own copy of RigCameras
  std::unordered_map<
      map::Map::ShotId,
      foundation::OptionalValue<map::Map::RigCamera>,
      std::hash<map::Map::ShotId>,
      std::equal_to<map::Map::ShotId>,
      Eigen::aligned_allocator<
          std::pair<const map::Map::ShotId,
                    foundation::OptionalValue<map::Map::RigCamera>>>>
      own_rig_cameras_;
};

}  // namespace map
