#pragma once
#include <geometry/camera.h>
#include <geometry/similarity.h>
#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>

#include <deque>
#include <unordered_map>

namespace map {

class Map;

// ---------- ShotView ----------
class ShotView {
 public:
  explicit ShotView(Map& map);
  Shot& GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id) const;

  // IMPORTANT: return the alias types from Map, not raw unordered_map,
  // so the Eigen aligned_allocator matches exactly.
  const Map::ShotMap& GetShots() const;
  size_t NumberOfShots() const;

 private:
  Map& map_;
};

// ---------- PanoShotView ----------
class PanoShotView {
 public:
  explicit PanoShotView(Map& map);
  Shot& GetShot(const map::ShotId& shot_id);
  bool HasShot(const map::ShotId& shot_id) const;
  const Map::ShotMap& GetShots() const;
  size_t NumberOfShots() const;

 private:
  Map& map_;
};

// ---------- LandmarkView ----------
class LandmarkView {
 public:
  explicit LandmarkView(Map& map);
  Landmark& GetLandmark(const LandmarkId& lm_id);
  bool HasLandmark(const LandmarkId& lm_id) const;
  const Map::LandmarkMap& GetLandmarks() const;
  size_t NumberOfLandmarks() const;

 private:
  Map& map_;
};

// ---------- CameraView ----------
class CameraView {
 public:
  explicit CameraView(Map& map);
  size_t NumberOfCameras() const;
  geometry::Camera& GetCamera(const CameraId& cam_id);
  const Map::CameraMap& GetCameras() const;
  bool HasCamera(const CameraId& cam_id) const;

 private:
  Map& map_;
};

// ---------- RigCameraView ----------
class RigCameraView {
 public:
  explicit RigCameraView(Map& map);
  size_t NumberOfRigCameras() const;
  RigCamera& GetRigCamera(const RigCameraId& rig_camera_id);
  const Map::RigCameraMap& GetRigCameras() const;
  bool HasRigCamera(const RigCameraId& rig_camera_id) const;

 private:
  Map& map_;
};

// ---------- RigInstanceView ----------
class RigInstanceView {
 public:
  explicit RigInstanceView(Map& map);
  size_t NumberOfRigInstances() const;
  RigInstance& GetRigInstance(const RigInstanceId& instance_id);
  const Map::RigInstanceMap& GetRigInstances() const;
  bool HasRigInstance(const RigInstanceId& instance_id) const;

 private:
  Map& map_;
};

// ---------- BiasView ----------
class BiasView {
 public:
  explicit BiasView(Map& map);
  size_t NumberOfBiases() const;
  geometry::Similarity& GetBias(const CameraId& cam_id);
  const Map::BiasMap& GetBiases() const;
  bool HasBias(const CameraId& cam_id) const;

 private:
  Map& map_;
};

}  // namespace map
