#include <geometry/pose.h>
#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <string>

namespace {
bool IsSingleShotRig(const map::RigInstance* rig_instance,
                     const map::RigCamera* rig_camera) {
  if (!rig_instance || !rig_camera) return false;
  const bool has_identity_rig_camera = rig_camera->pose.IsIdentity();
  const bool is_single_shot_instance = rig_instance->GetShots().size() == 1;
  return has_identity_rig_camera && is_single_shot_instance;
}
}  // namespace

namespace map {

Shot::Shot(const ShotId& shot_id,
           const geometry::Camera* const shot_camera,
           RigInstance* rig_instance,
           RigCamera* rig_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      rig_instance_(rig_instance),
      rig_camera_(rig_camera),
      shot_camera_(shot_camera) {
  if (rig_instance_) {
    rig_instance_->AddShot(rig_camera_, this);
    rig_instance_->UpdateInstancePoseWithShot(shot_id, pose);
  }
}

Shot::Shot(const ShotId& shot_id,
           const geometry::Camera* const shot_camera,
           RigInstance* rig_instance,
           RigCamera* rig_camera)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(geometry::Pose())),
      rig_instance_(rig_instance),
      rig_camera_(rig_camera),
      shot_camera_(shot_camera) {
  if (rig_instance_) rig_instance_->AddShot(rig_camera_, this);
}

Shot::Shot(const ShotId& shot_id,
           const geometry::Camera& shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      own_rig_instance_(RigInstance(shot_id)),
      own_rig_camera_(RigCamera{shot_id, geometry::Pose()}),
      rig_instance_(&own_rig_instance_.Value()),
      rig_camera_(&own_rig_camera_.Value()),
      own_camera_(shot_camera),
      shot_camera_(&own_camera_.Value()) {
  rig_instance_->AddShot(rig_camera_, this);
  rig_instance_->SetPose(pose);
}

bool Shot::IsInRig() const { return rig_instance_ != nullptr; }

void Shot::SetRig(RigInstance* rig_instance, RigCamera* rig_camera) {
  rig_instance_ = rig_instance;
  rig_camera_ = rig_camera;
}

const RigInstanceId& Shot::GetRigInstanceId() const {
  if (!rig_instance_) throw std::runtime_error("Shot has no rig instance.");
  return rig_instance_->GetId();
}

const RigCameraId& Shot::GetRigCameraId() const {
  if (!rig_camera_) throw std::runtime_error("Shot has no rig camera.");
  return rig_camera_->id;
}

void ShotMeasurements::Set(const ShotMeasurements& other) {
  *this = other;  // Copy trivially: OptionalValues and map are value types
}

void Shot::RemoveLandmarkObservation(const FeatureId id) {
  const auto it = landmark_id_.find(id);
  if (it == landmark_id_.end()) {
    throw std::runtime_error("Can't find Feature ID " + std::to_string(id) +
                             " in Shot " + id_);
  }
  auto* lm = it->second;
  landmark_id_.erase(it);
  landmark_observations_.erase(lm);
}

void Shot::SetPose(const geometry::Pose& pose) {
  if (!rig_instance_ || !rig_camera_) {
    *pose_ = pose;
    return;
  }
  if (!IsSingleShotRig(rig_instance_, rig_camera_)) {
    throw std::runtime_error(
        "Can't set the pose of a shot belonging to a multi-shot rig instance");
  }
  rig_instance_->SetPose(pose);
  *pose_ = pose;
}

geometry::Pose Shot::GetPoseInRig() const {
  // world←camera = (world←instance) * (instance←camera)
  const auto& pose_instance = rig_instance_->GetPose();
  const auto& rig_camera_pose = rig_camera_->pose;
  return pose_instance.Compose(rig_camera_pose);
}

const geometry::Pose* Shot::GetPose() const {
  if (rig_instance_ && rig_camera_) {
    *pose_ = GetPoseInRig();
    if (IsSingleShotRig(rig_instance_, rig_camera_)) {
      return &rig_instance_->GetPose();
    }
  }
  return pose_.get();
}

geometry::Pose* Shot::GetPose() {
  if (rig_instance_ && rig_camera_) {
    *pose_ = GetPoseInRig();
    if (IsSingleShotRig(rig_instance_, rig_camera_)) {
      return &rig_instance_->GetPose();
    }
  }
  return pose_.get();
}

Vec2d Shot::Project(const Vec3d& global_pos) const {
  const auto* P = GetPose();
  return shot_camera_->Project(P->RotationWorldToCamera() * global_pos +
                               P->TranslationWorldToCamera());
}

MatX2d Shot::ProjectMany(const MatX3d& points) const {
  MatX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i)
    projected.row(i) = Project(points.row(i));
  return projected;
}

Vec3d Shot::Bearing(const Vec2d& point) const {
  return GetPose()->RotationCameraToWorld() * shot_camera_->Bearing(point);
}

MatX3d Shot::BearingMany(const MatX2d& points) const {
  MatX3d bearings(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i)
    bearings.row(i) = Bearing(points.row(i));
  return bearings;
}

}  // namespace map
