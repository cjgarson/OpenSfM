#pragma once
#include <foundation/optional.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/observation.h>
#include <map/rig.h>

#include <Eigen/Eigen>
#include <iostream>
#include <unordered_map>

namespace map {

class Map;

struct ShotMesh {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void SetVertices(const MatXd& vertices) { vertices_ = vertices; }
  void SetFaces(const MatXd& faces) { faces_ = faces; }
  MatXd GetFaces() const { return faces_; }
  MatXd GetVertices() const { return vertices_; }
  MatXd vertices_;
  MatXd faces_;
};

struct ShotMeasurements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  foundation::OptionalValue<double> capture_time_;
  foundation::OptionalValue<Vec3d> gps_position_;
  foundation::OptionalValue<double> gps_accuracy_;
  foundation::OptionalValue<double> compass_accuracy_;
  foundation::OptionalValue<double> compass_angle_;
  foundation::OptionalValue<Vec3d> gravity_down_;
  foundation::OptionalValue<double> opk_accuracy_;
  foundation::OptionalValue<Vec3d> opk_angles_;
  foundation::OptionalValue<int> orientation_;
  foundation::OptionalValue<std::string> sequence_key_;
  void Set(const ShotMeasurements& other) { *this = other; }

  std::map<std::string, std::string> attributes_;
  const auto& GetAttributes() const { return attributes_; }
  auto& GetMutableAttributes() { return attributes_; }
  void SetAttributes(const std::map<std::string, std::string>& a) {
    attributes_ = a;
  }
};

class Shot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Shot(const ShotId& shot_id, const ::geometry::Camera* const shot_camera,
       RigInstance* rig_instance, RigCamera* rig_camera,
       const ::geometry::Pose& pose);
  Shot(const ShotId& shot_id, const ::geometry::Camera* const shot_camera,
       RigInstance* rig_instance, RigCamera* rig_camera);
  Shot(const ShotId& shot_id, const ::geometry::Camera& shot_camera,
       const ::geometry::Pose& pose);

  ShotId GetId() const { return id_; }

  bool IsInRig() const;
  void SetRig(RigInstance* rig_instance, RigCamera* rig_camera);
  RigInstance* GetRigInstance() const { return rig_instance_; }
  const RigCamera* GetRigCamera() const { return rig_camera_; }
  const RigInstanceId& GetRigInstanceId() const;
  const RigCameraId& GetRigCameraId() const;

  void SetPose(const ::geometry::Pose& pose);
  const ::geometry::Pose* const GetPose() const;
  ::geometry::Pose* const GetPose();
  Mat4d GetWorldToCam() const { return GetPose()->WorldToCamera(); }
  Mat4d GetCamToWorld() const { return GetPose()->CameraToWorld(); }

  const std::map<Landmark*, Observation, KeyCompare,
                 Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>&
  GetLandmarkObservations() const {
    return landmark_observations_;
  }
  std::map<Landmark*, Observation, KeyCompare,
           Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>&
  GetLandmarkObservations() {
    return landmark_observations_;
  }

  std::vector<Landmark*> ComputeValidLandmarks() {
    std::vector<Landmark*> v;
    v.reserve(landmark_observations_.size());
    for (auto& lm_obs : landmark_observations_) v.push_back(lm_obs.first);
    return v;
  }

  const Observation& GetObservation(const FeatureId id) const {
    return landmark_observations_.at(landmark_id_.at(id));
  }
  void CreateObservation(Landmark* lm, const Observation& obs) {
    landmark_observations_.insert({lm, obs});
    landmark_id_.insert({obs.feature_id, lm});
  }
  Observation* GetLandmarkObservation(Landmark* lm) {
    return &landmark_observations_.at(lm);
  }
  Landmark* GetObservationLandmark(const FeatureId id) {
    auto it = landmark_id_.find(id);
    return it == landmark_id_.end() ? nullptr : it->second;
  }
  void RemoveLandmarkObservation(const FeatureId id);

  const ShotMeasurements& GetShotMeasurements() const { return shot_measurements_; }
  ShotMeasurements& GetShotMeasurements() { return shot_measurements_; }
  void SetShotMeasurements(const ShotMeasurements& other) {
    shot_measurements_.Set(other);
  }

  bool operator==(const Shot& s) const { return id_ == s.id_; }
  bool operator!=(const Shot& s) const { return !(*this == s); }

  const ::geometry::Camera* const GetCamera() const { return shot_camera_; }

  Vec2d Project(const Vec3d& global_pos) const;
  MatX2d ProjectMany(const MatX3d& points) const;
  Vec3d Bearing(const Vec2d& point) const;
  MatX3d BearingMany(const MatX2d& points) const;

  MatXd GetCovariance() const { return covariance_.Value(); }
  void SetCovariance(const MatXd& cov) { covariance_.SetValue(cov); }

 public:
  const ShotId id_;
  ShotMesh mesh;
  long int merge_cc{0};
  double scale{1.0};

 private:
  ::geometry::Pose GetPoseInRig() const;

  mutable std::unique_ptr<::geometry::Pose> pose_;
  foundation::OptionalValue<MatXd> covariance_;

  foundation::OptionalValue<RigInstance> own_rig_instance_;
  foundation::OptionalValue<RigCamera> own_rig_camera_;
  RigInstance* rig_instance_{nullptr};
  RigCamera* rig_camera_{nullptr};

  foundation::OptionalValue<::geometry::Camera> own_camera_;
  const ::geometry::Camera* const shot_camera_;

  ShotMeasurements shot_measurements_;

  std::map<Landmark*, Observation, KeyCompare,
           Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>
      landmark_observations_;
  std::unordered_map<FeatureId, Landmark*> landmark_id_;
};

}  // namespace map
