#pragma once

#include <geo/geo.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <geometry/similarity.h>
#include <map/map_types.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>
#include <map/tracks_manager.h>
#include <map/dataviews.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>

namespace map {

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Aliases
  using CameraMap = map::CameraMap;
  using BiasMap = map::BiasMap;
  using ShotMap = map::ShotMap;
  using LandmarkMap = map::LandmarkMap;
  using RigCameraMap = map::RigCameraMap;
  using RigInstanceMap = map::RigInstanceMap;

  // Deep Copy
  static std::unique_ptr<Map> DeepCopy(const Map& map,
                                       bool copy_observations = false);

  // Camera
  ::geometry::Camera& GetCamera(const CameraId& cam_id);
  const ::geometry::Camera& GetCamera(const CameraId& cam_id) const;
  ::geometry::Camera& CreateCamera(const ::geometry::Camera& cam);
  CameraView GetCameraView() { return CameraView(*this); }
  bool HasCamera(const CameraId& cam_id) const { return cameras_.count(cam_id); }
  const CameraMap& GetCameras() const { return cameras_; }
  CameraMap& GetCameras() { return cameras_; }

  // Shots
  Shot& CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                   const RigCameraId& rig_camera_id,
                   const RigInstanceId& instance_id,
                   const ::geometry::Pose& pose);
  Shot& CreateShot(const ShotId& shot_id, const CameraId& camera_id,
                   const RigCameraId& rig_camera_id,
                   const RigInstanceId& instance_id);

  const Shot& GetShot(const ShotId& shot_id) const;
  Shot& GetShot(const ShotId& shot_id);
  bool HasShot(const ShotId& shot_id) const { return shots_.count(shot_id); }
  const ShotMap& GetShots() const { return shots_; }
  ShotMap& GetShots() { return shots_; }

  Shot& UpdateShot(const Shot& other_shot);
  void RemoveShot(const ShotId& shot_id);
  ShotView GetShotView() { return ShotView(*this); }

  // PanoShots
  Shot& CreatePanoShot(const ShotId& shot_id, const CameraId& camera_id,
                       const RigCameraId& rig_camera_id,
                       const RigInstanceId& instance_id,
                       const ::geometry::Pose& pose);

  Shot& GetPanoShot(const ShotId& shot_id);
  const Shot& GetPanoShot(const ShotId& shot_id) const;
  bool HasPanoShot(const ShotId& shot_id) const {
    return pano_shots_.count(shot_id);
  }
  const ShotMap& GetPanoShots() const { return pano_shots_; }
  ShotMap& GetPanoShots() { return pano_shots_; }

  void RemovePanoShot(const ShotId& shot_id);
  Shot& UpdatePanoShot(const Shot& other_shot);
  PanoShotView GetPanoShotView() { return PanoShotView(*this); }

  // Rigs
  RigCamera& CreateRigCamera(const map::RigCamera& rig_camera);
  RigInstance& CreateRigInstance(const map::RigInstanceId& instance_id);
  void RemoveRigInstance(const map::RigInstanceId& instance_id);
  RigInstance& UpdateRigInstance(const RigInstance& other_rig_instance);

  size_t NumberOfRigCameras() const;
  RigCamera& GetRigCamera(const RigCameraId& rig_camera_id);
  const RigCameraMap& GetRigCameras() const { return rig_cameras_; }
  RigCameraMap& GetRigCameras() { return rig_cameras_; }
  bool HasRigCamera(const RigCameraId& rig_camera_id) const;

  size_t NumberOfRigInstances() const;
  RigInstance& GetRigInstance(const RigInstanceId& instance_id);
  const RigInstance& GetRigInstance(const RigInstanceId& instance_id) const;
  const RigInstanceMap& GetRigInstances() const { return rig_instances_; }
  RigInstanceMap& GetRigInstances() { return rig_instances_; }
  bool HasRigInstance(const RigInstanceId& instance_id) const;

  // Landmarks
  Landmark& CreateLandmark(const LandmarkId& lm_id, const Vec3d& global_pos);
  const Landmark& GetLandmark(const LandmarkId& lm_id) const;
  Landmark& GetLandmark(const LandmarkId& lm_id);
  const LandmarkMap& GetLandmarks() const { return landmarks_; }
  LandmarkMap& GetLandmarks() { return landmarks_; }
  bool HasLandmark(const LandmarkId& lm_id) const {
    return landmarks_.count(lm_id);
  }
  LandmarkView GetLandmarkView() { return LandmarkView(*this); }

  void RemoveLandmark(const Landmark* const lm);
  void RemoveLandmark(const LandmarkId& lm_id);

  // Observations
  void AddObservation(Shot* const shot, Landmark* const lm,
                      const Observation& obs);
  void AddObservation(const ShotId& shot_id, const LandmarkId& lm_id,
                      const Observation& obs);
  void RemoveObservation(const ShotId& shot_id, const LandmarkId& lm_id);
  void ClearObservationsAndLandmarks();
  void CleanLandmarksBelowMinObservations(size_t min_observations);

  // Map info
  size_t NumberOfShots() const { return shots_.size(); }
  size_t NumberOfPanoShots() const { return pano_shots_.size(); }
  size_t NumberOfLandmarks() const { return landmarks_.size(); }
  size_t NumberOfCameras() const { return cameras_.size(); }
  size_t NumberOfBiases() const { return bias_.size(); }

  // Bias
  BiasView GetBiasView() { return BiasView(*this); }
  ::geometry::Similarity& GetBias(const CameraId& camera_id);
  void SetBias(const CameraId& camera_id,
               const ::geometry::Similarity& transform);
  bool HasBias(const CameraId& cam_id) const { return bias_.count(cam_id); }
  const BiasMap& GetBiases() const { return bias_; }
  BiasMap& GetBiases() { return bias_; }

  // Topocentric converter
  const geo::TopocentricConverter& GetTopocentricConverter() const {
    return topo_conv_;
  }
  void SetTopocentricConverter(double lat, double lon, double alt) {
    topo_conv_.lat_ = lat;
    topo_conv_.long_ = lon;
    topo_conv_.alt_ = alt;
  }

  TracksManager ToTracksManager() const;

  enum ErrorType { Pixel = 0x0, Normalized = 0x1, Angular = 0x2 };
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Vec2d>>
  ComputeReprojectionErrors(const TracksManager& tracks_manager,
                            const ErrorType& error_type) const;
  std::unordered_map<ShotId, std::unordered_map<LandmarkId, Observation>>
  GetValidObservations(const TracksManager& tracks_manager) const;

 private:
  void UpdateShotWithRig(const Shot& other_shot, bool is_panoshot = false);

  CameraMap cameras_;
  BiasMap bias_;
  ShotMap shots_;
  ShotMap pano_shots_;
  LandmarkMap landmarks_;
  RigInstanceMap rig_instances_;
  RigCameraMap rig_cameras_;
  geo::TopocentricConverter topo_conv_;
};

}  // namespace map
