#include <map/map.h>

namespace map {

ShotView::ShotView(Map& map) : map_(map) {}
Shot& ShotView::GetShot(const map::ShotId& shot_id) {
  return map_.GetShot(shot_id);
}
bool ShotView::HasShot(const map::ShotId& shot_id) const {
  return map_.HasShot(shot_id);
}
const Map::ShotMap& ShotView::GetShots() const {  // Updated type
  return map_.GetShots();
}
size_t ShotView::NumberOfShots() const { return map_.NumberOfShots(); }

PanoShotView::PanoShotView(Map& map) : map_(map) {}
Shot& PanoShotView::GetShot(const map::ShotId& shot_id) {
  return map_.GetPanoShot(shot_id);
}
bool PanoShotView::HasShot(const map::ShotId& shot_id) const {
  return map_.HasPanoShot(shot_id);
}
const Map::ShotMap& PanoShotView::GetShots() const {  // Updated type
  return map_.GetPanoShots();
}
size_t PanoShotView::NumberOfShots() const { return map_.NumberOfPanoShots(); }

LandmarkView::LandmarkView(Map& map) : map_(map) {}
Landmark& LandmarkView::GetLandmark(const LandmarkId& lm_id) {
  return map_.GetLandmark(lm_id);
}
bool LandmarkView::HasLandmark(const LandmarkId& lm_id) const {
  return map_.HasLandmark(lm_id);
}
const Map::LandmarkMap& LandmarkView::GetLandmarks() const {  // Updated type
  return map_.GetLandmarks();
}
size_t LandmarkView::NumberOfLandmarks() const {
  return map_.NumberOfLandmarks();
}

CameraView::CameraView(Map& map) : map_(map) {}
size_t CameraView::NumberOfCameras() const { return map_.NumberOfCameras(); }
geometry::Camera& CameraView::GetCamera(const CameraId& cam_id) {
  return map_.GetCamera(cam_id);
}
const Map::CameraMap& CameraView::GetCameras() const {  // Updated type
  return map_.GetCameras();
}
bool CameraView::HasCamera(const CameraId& cam_id) const {
  return map_.HasCamera(cam_id);
}

BiasView::BiasView(Map& map) : map_(map) {}
size_t BiasView::NumberOfBiases() const { return map_.NumberOfBiases(); }
geometry::Similarity& BiasView::GetBias(const CameraId& cam_id) {
  return map_.GetBias(cam_id);
}
const Map::BiasMap& BiasView::GetBiases() const {  // Updated type
  return map_.GetBiases();
}
bool BiasView::HasBias(const CameraId& cam_id) const {
  return map_.HasBias(cam_id);
}

RigCameraView::RigCameraView(Map& map) : map_(map) {}
size_t RigCameraView::NumberOfRigCameras() const {
  return map_.NumberOfRigCameras();
}
RigCamera& RigCameraView::GetRigCamera(const RigCameraId& rig_camera_id) {
  return map_.GetRigCamera(rig_camera_id);
}
const Map::RigCameraMap& RigCameraView::GetRigCameras() const {  // Updated type
  return map_.GetRigCameras();
}
bool RigCameraView::HasRigCamera(const RigCameraId& rig_camera_id) const {
  return map_.HasRigCamera(rig_camera_id);
}

RigInstanceView::RigInstanceView(Map& map) : map_(map) {}
size_t RigInstanceView::NumberOfRigInstances() const {
  return map_.NumberOfRigInstances();
}
RigInstance& RigInstanceView::GetRigInstance(const RigInstanceId& instance_id) {
  return map_.GetRigInstance(instance_id);
}
const Map::RigInstanceMap& RigInstanceView::GetRigInstances() const {  // Updated type
  return map_.GetRigInstances();
}
bool RigInstanceView::HasRigInstance(const RigInstanceId& instance_id) const {
  return map_.HasRigInstance(instance_id);
}

}  // namespace map
