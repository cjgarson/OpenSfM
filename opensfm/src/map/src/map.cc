#include <geometry/pose.h>
#include <map/dataviews.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/rig.h>
#include <map/shot.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace {
void AssignShot(sfmmap::Shot& to, const sfmmap::Shot& from) {
  to.merge_cc = from.merge_cc;
  to.scale = from.scale;
  to.SetShotMeasurements(from.GetShotMeasurements());
  to.SetCovariance(from.GetCovariance());
}
}  // namespace

namespace sfmmap {

CameraView   Map::GetCameraView()   { return CameraView(*this); }
ShotView     Map::GetShotView()     { return ShotView(*this); }
PanoShotView Map::GetPanoShotView() { return PanoShotView(*this); }
LandmarkView Map::GetLandmarkView() { return LandmarkView(*this); }
BiasView     Map::GetBiasView()     { return BiasView(*this); }

std::unique_ptr<Map> Map::DeepCopy(const Map& map, bool copy_observations) {
  auto map_copy = std::make_unique<Map>();
  map_copy->topo_conv_ = map.topo_conv_;

  // Copy cameras
  for (const auto& camera_kv : map.GetCameras()) {
    map_copy->CreateCamera(camera_kv.second);
  }

  // Copy shots and pano shots, preserving rig relationships
  for (const auto& shot_kv : map.GetShots()) {
    if (!map_copy->HasShot(shot_kv.first)) {
      map_copy->UpdateShotWithRig(shot_kv.second, false);
    }
  }
  for (const auto& pano_kv : map.GetPanoShots()) {
    if (!map_copy->HasPanoShot(pano_kv.first)) {
      map_copy->UpdateShotWithRig(pano_kv.second, true);
    }
  }

  // Copy landmarks
  for (const auto& lm_kv : map.GetLandmarks()) {
    map_copy->CreateLandmark(lm_kv.first, lm_kv.second.GetGlobalPos());
  }

  // Copy observations if requested
  if (copy_observations) {
    for (const auto& shot_kv : map.GetShots()) {
      for (const auto& obs_kv : shot_kv.second.GetLandmarkObservations()) {
        map_copy->AddObservation(shot_kv.first, obs_kv.first->id_, obs_kv.second);
      }
    }
  }

  // Copy biases (if any)
  for (const auto& bias_kv : map.GetBiases()) {
    map_copy->SetBias(bias_kv.first, bias_kv.second);
  }

  return map_copy;
}

void Map::AddObservation(Shot* const shot, Landmark* const lm, const Observation& obs) {
  lm->AddObservation(shot, obs.feature_id);
  shot->CreateObservation(lm, obs);
}

void Map::AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                         const Observation& obs) {
  auto& shot = GetShot(shot_id);
  auto& lm = GetLandmark(lm_id);
  AddObservation(&shot, &lm, obs);
}

void Map::RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id) {
  auto& shot = GetShot(shot_id);
  auto& lm = GetLandmark(lm_id);
  shot.RemoveLandmarkObservation(lm.GetObservationIdInShot(&shot));
  lm.RemoveObservation(&shot);
}

const Shot& Map::GetShot(const ShotId& shot_id) const {
  auto it = shots_.find(shot_id);
  if (it == shots_.end()) {
    throw std::runtime_error("Invalid ShotID " + shot_id);
  }
  return it->second;
}

Shot& Map::GetShot(const ShotId& shot_id) {
  auto it = shots_.find(shot_id);
  if (it == shots_.end()) {
    throw std::runtime_error("Invalid ShotID " + shot_id);
  }
  return it->second;
}

Shot& Map::GetPanoShot(const ShotId& shot_id) {
  auto it = pano_shots_.find(shot_id);
  if (it == pano_shots_.end()) {
    throw std::runtime_error("Invalid PanoShotID " + shot_id);
  }
  return it->second;
}

const Shot& Map::GetPanoShot(const ShotId& shot_id) const {
  auto it = pano_shots_.find(shot_id);
  if (it == pano_shots_.end()) {
    throw std::runtime_error("Invalid PanoShotID " + shot_id);
  }
  return it->second;
}

const Landmark& Map::GetLandmark(const LandmarkId& lm_id) const {
  auto it = landmarks_.find(lm_id);
  if (it == landmarks_.end()) {
    throw std::runtime_error("Invalid LandmarkId " + lm_id);
  }
  return it->second;
}

Landmark& Map::GetLandmark(const LandmarkId& lm_id) {
  auto it = landmarks_.find(lm_id);
  if (it == landmarks_.end()) {
    throw std::runtime_error("Invalid LandmarkId " + lm_id);
  }
  return it->second;
}

void Map::ClearObservationsAndLandmarks() {
  for (auto& lm_kv : landmarks_) {
    auto& observations = lm_kv.second.GetObservations();
    for (const auto& obs : observations) {
      obs.first->RemoveLandmarkObservation(obs.second);
    }
    lm_kv.second.ClearObservations();
  }
  landmarks_.clear();
}

void Map::CleanLandmarksBelowMinObservations(size_t min_observations) {
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    if (it->second.NumberOfObservations() < min_observations) {
      for (const auto& obs : it->second.GetObservations()) {
        Shot* shot = obs.first;
        shot->RemoveLandmarkObservation(obs.second);
      }
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const RigCameraId& rig_camera_id,
                      const RigInstanceId& instance_id,
                      const ::geometry::Pose& pose) {
  if (shots_.find(shot_id) != shots_.end()) {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
  const auto& camera = GetCamera(camera_id);
  auto& rig_instance = GetRigInstance(instance_id);
  auto& rig_camera = GetRigCamera(rig_camera_id);
  auto it = shots_.emplace(std::piecewise_construct,
                           std::forward_as_tuple(shot_id),
                           std::forward_as_tuple(shot_id, &camera, &rig_instance, &rig_camera, pose));
  return it.first->second;
}

Shot& Map::CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                      const RigCameraId& rig_camera_id,
                      const RigInstanceId& instance_id) {
  if (shots_.find(shot_id) != shots_.end()) {
    throw std::runtime_error("Shot " + shot_id + " already exists.");
  }
  const auto& camera = GetCamera(camera_id);
  auto& rig_instance = GetRigInstance(instance_id);
  auto& rig_camera = GetRigCamera(rig_camera_id);
  auto it = shots_.emplace(std::piecewise_construct,
                           std::forward_as_tuple(shot_id),
                           std::forward_as_tuple(shot_id, &camera, &rig_instance, &rig_camera));
  return it.first->second;
}

void Map::RemoveShot(const ShotId& shot_id) {
  auto shot_it = shots_.find(shot_id);
  if (shot_it == shots_.end()) {
    throw std::runtime_error("Invalid ShotID " + shot_id);
  }
  Shot& shot = shot_it->second;
  shot.GetRigInstance()->RemoveShot(shot_id);
  for (auto& lm_obs : shot.GetLandmarkObservations()) {
    lm_obs.first->RemoveObservation(&shot);
  }
  shots_.erase(shot_it);
}

Shot& Map::CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id,
                          const RigCameraId& rig_camera_id,
                          const RigInstanceId& instance_id,
                          const ::geometry::Pose& pose) {
  if (pano_shots_.find(shot_id) != pano_shots_.end()) {
    throw std::runtime_error("PanoShot " + shot_id + " already exists.");
  }
  const auto& camera = GetCamera(camera_id);
  auto& rig_instance = GetRigInstance(instance_id);
  auto& rig_camera = GetRigCamera(rig_camera_id);
  auto it = pano_shots_.emplace(std::piecewise_construct,
                                std::forward_as_tuple(shot_id),
                                std::forward_as_tuple(shot_id, &camera, &rig_instance, &rig_camera, pose));
  return it.first->second;
}

void Map::RemovePanoShot(const ShotId& shot_id) {
  auto shot_it = pano_shots_.find(shot_id);
  if (shot_it == pano_shots_.end()) {
    throw std::runtime_error("Invalid PanoShotID " + shot_id);
  }
  const Shot& shot = shot_it->second;
  shot.GetRigInstance()->RemoveShot(shot_id);
  pano_shots_.erase(shot_it);
}

Landmark& Map::CreateLandmark(const LandmarkId& lm_id, const Vec3d& global_pos) {
  if (landmarks_.find(lm_id) != landmarks_.end()) {
    throw std::runtime_error("Landmark " + lm_id + " already exists.");
  }
  auto it = landmarks_.emplace(std::piecewise_construct,
                               std::forward_as_tuple(lm_id),
                               std::forward_as_tuple(lm_id, global_pos));
  return it.first->second;
}

void Map::RemoveLandmark(const Landmark* const lm) {
  if (!lm) {
    throw std::runtime_error("Nullptr landmark");
  }
  RemoveLandmark(lm->id_);
}

void Map::RemoveLandmark(const LandmarkId& lm_id) {
  auto lm_it = landmarks_.find(lm_id);
  if (lm_it == landmarks_.end()) {
    throw std::runtime_error("Invalid LandmarkId " + lm_id);
  }
  const Landmark& lm = lm_it->second;
  for (const auto& obs : lm.GetObservations()) {
    Shot* shot = obs.first;
    shot->RemoveLandmarkObservation(obs.second);
  }
  landmarks_.erase(lm_it);
}

::geometry::Camera& Map::CreateCamera(const ::geometry::Camera& cam) {
  auto it = cameras_.emplace(cam.id, cam);
  bias_.emplace(cam.id, ::geometry::Similarity());
  return it.first->second;
}

::geometry::Camera& Map::GetCamera(const CameraId& cam_id) {
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end()) {
    throw std::runtime_error("Invalid CameraId " + cam_id);
  }
  return it->second;
}

const ::geometry::Camera& Map::GetCamera(const CameraId& cam_id) const {
  auto it = cameras_.find(cam_id);
  if (it == cameras_.end()) {
    throw std::runtime_error("Invalid CameraId " + cam_id);
  }
  return it->second;
}

void Map::UpdateShotWithRig(const Shot& other_shot, bool is_panoshot) {
  RigInstance* rig_instance = other_shot.GetRigInstance();
  const RigInstanceId& instance_id = rig_instance->GetId();
  if (!HasRigInstance(instance_id)) {
    CreateRigInstance(instance_id);
  }
  for (const auto& shot_pair : rig_instance->GetShots()) {
    const ShotId& shot_id = shot_pair.first;
    Shot* shot = shot_pair.second;
    const auto camera = shot->GetCamera();
    const CameraId& camera_id = camera->id;
    if (!HasCamera(camera_id)) {
      CreateCamera(*camera);
    }
    const RigCamera* rig_camera = shot->GetRigCamera();
    const RigCameraId& rig_camera_id = rig_camera->id;
    if (!HasRigCamera(rig_camera_id)) {
      CreateRigCamera(*rig_camera);
    }
    bool exists = is_panoshot ? HasPanoShot(shot_id) : HasShot(shot_id);
    if (!exists) {
      Shot* new_shot = nullptr;
      if (is_panoshot) {
        new_shot = &CreatePanoShot(shot_id, camera_id, rig_camera_id, instance_id, *shot->GetPose());
      } else {
        new_shot = &CreateShot(shot_id, camera_id, rig_camera_id, instance_id, *shot->GetPose());
      }
      ::AssignShot(*new_shot, *shot);
    }
  }
  GetRigInstance(instance_id).UpdateInstancePoseWithShot(other_shot.GetId(), *other_shot.GetPose());
}

Shot& Map::UpdateShot(const Shot& other_shot) {
  auto it_exist = shots_.find(other_shot.GetId());
  if (it_exist == shots_.end()) {
    throw std::runtime_error("Shot " + other_shot.GetId() + " does not exist.");
  }
  Shot& shot = it_exist->second;
  UpdateShotWithRig(other_shot, false);
  ::AssignShot(shot, other_shot);
  return shot;
}

Shot& Map::UpdatePanoShot(const Shot& other_shot) {
  auto it_exist = pano_shots_.find(other_shot.GetId());
  if (it_exist == pano_shots_.end()) {
    throw std::runtime_error("Pano shot " + other_shot.GetId() + " does not exist.");
  }
  Shot& shot = it_exist->second;
  UpdateShotWithRig(other_shot, true);
  ::AssignShot(shot, other_shot);
  return shot;
}

RigCamera& Map::CreateRigCamera(const RigCamera& rig_camera) {
  if (rig_cameras_.find(rig_camera.id) != rig_cameras_.end()) {
    throw std::runtime_error("RigCamera " + rig_camera.id + " already exists.");
  }
  auto it = rig_cameras_.emplace(rig_camera.id, rig_camera);
  return it.first->second;
}

RigInstance& Map::CreateRigInstance(const RigInstanceId& instance_id) {
  if (rig_instances_.find(instance_id) != rig_instances_.end()) {
    throw std::runtime_error("RigInstance " + instance_id + " already exists.");
  }
  auto it = rig_instances_.emplace(std::piecewise_construct,
                                   std::forward_as_tuple(instance_id),
                                   std::forward_as_tuple(instance_id));
  return it.first->second;
}

void Map::RemoveRigInstance(const RigInstanceId& instance_id) {
  auto it_exist = rig_instances_.find(instance_id);
  if (it_exist == rig_instances_.end()) {
    throw std::runtime_error("Rig instance does not exist.");
  }
  rig_instances_.erase(it_exist);
}

// NEW: two-argument version (matches map.h)
RigInstance& Map::UpdateRigInstance(const RigInstance& other_rig_instance,
                                    const Map::RigCameraMap& rig_cameras) {
  // Ensure instance exists (create if needed)
  auto it_exist = rig_instances_.find(other_rig_instance.GetId());
  if (it_exist == rig_instances_.end()) {
    it_exist = rig_instances_.emplace(std::piecewise_construct,
                                      std::forward_as_tuple(other_rig_instance.GetId()),
                                      std::forward_as_tuple(other_rig_instance.GetId()))
                   .first;
  }
  RigInstance& rig_instance = it_exist->second;

  // Update pose (and any assignable state). We do NOT try to deep sync shot pointers here.
  rig_instance = other_rig_instance;

  // Copy provided rig cameras into our aligned container to keep memory stable for pointers.
  for (const auto& kv : rig_cameras) {
    const RigCameraId& rcid = kv.first;
    const RigCamera&   rc   = kv.second;
    rig_cameras_[rcid] = rc;
  }

  return rig_instance;
}

// Backward-compatibility wrapper for any old C++ callers
RigInstance& Map::UpdateRigInstance(const RigInstance& other_rig_instance) {
  static const Map::RigCameraMap kEmptyRigCameras;
  return UpdateRigInstance(other_rig_instance, kEmptyRigCameras);
}

size_t Map::NumberOfRigCameras() const {
  return rig_cameras_.size();
}

RigCamera& Map::GetRigCamera(const RigCameraId& rig_camera_id) {
  auto it = rig_cameras_.find(rig_camera_id);
  if (it == rig_cameras_.end()) {
    throw std::runtime_error("Invalid RigCameraID " + rig_camera_id);
  }
  return it->second;
}

bool Map::HasRigCamera(const RigCameraId& rig_camera_id) const {
  return rig_cameras_.find(rig_camera_id) != rig_cameras_.end();
}

size_t Map::NumberOfRigInstances() const {
  return rig_instances_.size();
}

RigInstance& Map::GetRigInstance(const RigInstanceId& instance_id) {
  auto it = rig_instances_.find(instance_id);
  if (it == rig_instances_.end()) {
    throw std::runtime_error("Invalid RigInstance index");
  }
  return it->second;
}

const RigInstance& Map::GetRigInstance(const RigInstanceId& instance_id) const {
  auto it = rig_instances_.find(instance_id);
  if (it == rig_instances_.end()) {
    throw std::runtime_error("Invalid RigInstance index");
  }
  return it->second;
}

bool Map::HasRigInstance(const RigInstanceId& instance_id) const {
  return rig_instances_.find(instance_id) != rig_instances_.end();
}

::geometry::Similarity& Map::GetBias(const CameraId& camera_id) {
  auto it = bias_.find(camera_id);
  if (it == bias_.end()) {
    throw std::runtime_error("Invalid CameraID " + camera_id);
  }
  return it->second;
}

void Map::SetBias(const CameraId& camera_id, const ::geometry::Similarity& transform) {
  auto it = bias_.find(camera_id);
  if (it == bias_.end()) {
    throw std::runtime_error("Invalid CameraID " + camera_id);
  }
  it->second = transform;
}

std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d>>
Map::ComputeReprojectionErrors(const TracksManager& tracks_manager, const ErrorType& error_type) const {
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d>> errors;
  for (const ShotId& shot_id : tracks_manager.GetShotIds()) {
    auto it_shot = shots_.find(shot_id);
    if (it_shot == shots_.end()) {
      continue;
    }
    const Shot& shot = it_shot->second;
    auto& per_shot = errors[shot_id];
    for (const auto& obs_kv : tracks_manager.GetShotObservations(shot_id)) {
      auto it_lm = landmarks_.find(obs_kv.first);
      if (it_lm == landmarks_.end()) {
        continue;
      }
      if (error_type == ErrorType::Pixel) {
        Vec2d err = obs_kv.second.point - shot.Project(it_lm->second.GetGlobalPos());
        per_shot[obs_kv.first] = err;
      }
      if (error_type == ErrorType::Normalized) {
        Vec2d err = obs_kv.second.point - shot.Project(it_lm->second.GetGlobalPos());
        per_shot[obs_kv.first] = err / obs_kv.second.scale;
      }
      if (error_type == ErrorType::Angular) {
        Vec3d pt_cam = (it_lm->second.GetGlobalPos() - shot.GetPose()->GetOrigin()).normalized();
        Vec3d bearing = shot.Bearing(obs_kv.second.point).normalized();
        double angle = std::acos(pt_cam.dot(bearing));
        per_shot[obs_kv.first] = Vec2d::Constant(angle);
      }
    }
  }
  return errors;
}

std::unordered_map<sfmmap::LandmarkId, sfmmap::Observation>
Map::GetShotObservations(const sfmmap::ShotId& shot_id) const {
    std::unordered_map<sfmmap::LandmarkId, sfmmap::Observation> result;
    const auto& shot = GetShot(shot_id);
    for (const auto& kv : shot.GetLandmarkObservations()) {
        result[kv.first] = kv.second;
    }
    return result;
}

std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation>>
Map::GetValidObservations(const TracksManager& tracks_manager) const {
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation>> valid_obs;
  for (const ShotId& shot_id : tracks_manager.GetShotIds()) {
    auto it_shot = shots_.find(shot_id);
    if (it_shot == shots_.end()) {
      continue;
    }
    auto& per_shot = valid_obs[shot_id];
    for (const auto& obs_kv : tracks_manager.GetShotObservations(shot_id)) {
      auto it_lm = landmarks_.find(obs_kv.first);
      if (it_lm == landmarks_.end()) {
        continue;
      }
      per_shot[obs_kv.first] = obs_kv.second;
    }
  }
  return valid_obs;
}

TracksManager Map::ToTracksManager() const {
  TracksManager manager;
  for (const auto& shot_kv : shots_) {
    for (const auto& lm_obs : shot_kv.second.GetLandmarkObservations()) {
      manager.AddObservation(shot_kv.first, lm_obs.first->id_, lm_obs.second);
    }
  }
  for (const auto& pano_kv : pano_shots_) {
    for (const auto& lm_obs : pano_kv.second.GetLandmarkObservations()) {
      manager.AddObservation(pano_kv.first, lm_obs.first->id_, lm_obs.second);
    }
  }
  return manager;
}

}  // namespace sfmmap
