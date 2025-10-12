// src/sfm/src/ba_helpers.cc
#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>

#include <map/ground_control_points.h>
#include <map/map.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geo/geo.h"
#include "map/defines.h"
#include <sfm/ba_helpers.h>

namespace py = pybind11;

namespace sfm {

// -----------------------------------------------------------------------------
// Neighborhood (IDs)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<map::ShotId>, std::unordered_set<map::ShotId>>
BAHelpers::ShotNeighborhoodIds(map::Map& map,
                               const map::ShotId& central_shot_id,
                               size_t radius,
                               size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points, max_interior_size);

  std::unordered_set<map::ShotId> interior;
  interior.reserve(res.first.size());
  for (map::Shot* s : res.first) interior.insert(s->GetId());

  std::unordered_set<map::ShotId> boundary;
  boundary.reserve(res.second.size());
  for (map::Shot* s : res.second) boundary.insert(s->GetId());

  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Neighborhood (pointers)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<map::Shot*>, std::unordered_set<map::Shot*>>
BAHelpers::ShotNeighborhood(map::Map& map,
                            const map::ShotId& central_shot_id,
                            size_t radius,
                            size_t min_common_points,
                            size_t max_interior_size) {
  constexpr size_t kMaxBoundarySize = 1000000;

  std::unordered_set<map::Shot*> interior;
  interior.reserve(max_interior_size);

  auto& central_shot = map.GetShot(central_shot_id);
  // If part of a rig, add all shots in the same instance
  const auto instance_shots = map.GetRigInstance(central_shot.GetRigInstanceId()).GetShotIDs();
  for (const auto& sid : instance_shots) {
    interior.insert(&map.GetShot(sid));
  }
  // and ensure the central itself is present
  interior.insert(&central_shot);

  for (size_t d = 1; d < radius && interior.size() < max_interior_size; ++d) {
    const auto remaining = max_interior_size - interior.size();
    auto neighbors = DirectShotNeighbors(map, interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }

  // boundary = direct neighbors with a low threshold
  auto boundary = DirectShotNeighbors(map, interior, /*min_common_points=*/1, kMaxBoundarySize);
  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Direct neighbors
// -----------------------------------------------------------------------------
std::unordered_set<map::Shot*>
BAHelpers::DirectShotNeighbors(map::Map& map,
                               const std::unordered_set<map::Shot*>& shot_ids,
                               const size_t min_common_points,
                               const size_t max_neighbors) {
  std::unordered_set<map::Landmark*> points;
  for (auto* shot : shot_ids) {
    // map<Landmark*, Observation>
    for (const auto& kv : shot->GetLandmarkObservations()) {
      points.insert(kv.first);
    }
  }

  std::unordered_map<map::Shot*, size_t> common_points;
  for (auto* lm : points) {
    // map<Shot*, FeatureId>
    for (const auto& obs : lm->GetObservations()) {
      auto* nshot = obs.first;
      if (shot_ids.find(nshot) == shot_ids.end()) {
        ++common_points[nshot];
      }
    }
  }

  std::vector<std::pair<map::Shot*, size_t>> ranked;
  ranked.reserve(common_points.size());
  for (const auto& kv : common_points) ranked.emplace_back(kv.first, kv.second);

  std::sort(ranked.begin(), ranked.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

  const size_t max_n = std::min(max_neighbors, ranked.size());
  std::unordered_set<map::Shot*> out;
  out.reserve(max_n);

  size_t idx = 0;
  for (const auto& p : ranked) {
    if (p.second >= min_common_points && idx < max_n) {
      // add whole rig instance for consistency
      const auto instance_shots =
          map.GetRigInstance(p.first->GetRigInstanceId()).GetShotIDs();
      for (const auto& sid : instance_shots) {
        out.insert(&map.GetShot(sid));
      }
      ++idx;
    } else {
      break;
    }
  }
  return out;
}

// -----------------------------------------------------------------------------
// BundleLocal
// -----------------------------------------------------------------------------
py::tuple BAHelpers::BundleLocal(
    map::Map& map,
    const std::unordered_map<map::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<map::RigCameraId, map::RigCamera>& rig_camera_priors,
    const AlignedVector<map::GroundControlPoint>& gcp,
    const map::ShotId& central_shot_id,
    const py::dict& config) {

  py::dict report;
  const auto start = std::chrono::high_resolution_clock::now();

  auto neighborhood = ShotNeighborhood(
      map,
      central_shot_id,
      config["local_bundle_radius"].cast<size_t>(),
      config["local_bundle_min_common_points"].cast<size_t>(),
      config["local_bundle_max_shots"].cast<size_t>());

  auto& interior = neighborhood.first;
  auto& boundary = neighborhood.second;

  bundle::BundleAdjuster ba;
  ba.SetUseAnalyticDerivatives(config["bundle_analytic_derivatives"].cast<bool>());

  // Cameras (fixed in local BA)
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    const auto& cam_prior = camera_priors.at(cam.id);
    constexpr bool fix_cameras{true};
    ba.AddCamera(cam.id, cam, cam_prior, fix_cameras);
  }

  // Collect involved rig cameras / instances
  std::unordered_set<map::RigCameraId> rig_cameras_ids;
  std::unordered_set<map::RigInstanceId> rig_instances_ids;
  for (auto* shot : interior) {
    rig_cameras_ids.insert(shot->GetRigCameraId());
    rig_instances_ids.insert(shot->GetRigInstanceId());
  }
  for (auto* shot : boundary) {
    rig_cameras_ids.insert(shot->GetRigCameraId());
    rig_instances_ids.insert(shot->GetRigInstanceId());
  }

  // Rig cameras are fixed here
  constexpr bool rig_camera_constant{true};
  for (const auto& rid : rig_cameras_ids) {
    const auto& rc = map.GetRigCamera(rid);
    ba.AddRigCamera(rid, rc.pose, rig_camera_priors.at(rid).pose, rig_camera_constant);
  }

  // Rig instances
  const std::string gps_scale_group = "dummy";
  for (const auto& iid : rig_instances_ids) {
    auto& instance = map.GetRigInstance(iid);
    std::unordered_map<std::string, std::string> shot_cameras;
    std::unordered_map<std::string, std::string> shot_rig_cameras;

    Vec3d avg_pos = Vec3d::Zero();
    double avg_std = 0.0;
    int gps_count = 0;
    bool fix_instance = false;

    for (const auto& shot_n_rc : instance.GetRigCameras()) {
      const auto shot_id = shot_n_rc.first;
      auto& shot = map.GetShot(shot_id);
      shot_cameras[shot_id] = shot.GetCamera()->id;
      shot_rig_cameras[shot_id] = shot_n_rc.second->id;

      const bool is_boundary = boundary.find(&shot) != boundary.end();
      if (is_boundary) fix_instance = true;

      if (!is_boundary && config["bundle_use_gps"].cast<bool>()) {
        const auto meas = shot.GetShotMeasurements();
        if (meas.gps_position_.HasValue()) {
          avg_pos += meas.gps_position_.Value();
          avg_std += meas.gps_accuracy_.Value();
          ++gps_count;
        }
      }
    }

    ba.AddRigInstance(iid, instance.GetPose(), shot_cameras, shot_rig_cameras, fix_instance);

    if (!fix_instance && gps_count > 0) {
      avg_pos /= gps_count;
      avg_std /= gps_count;
      ba.AddRigInstancePositionPrior(iid, avg_pos, Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // Points + observations
  std::unordered_set<map::Landmark*> points;
  py::list pt_ids;
  constexpr bool point_constant{false};

  for (auto* shot : interior) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.insert(lm).second) {
        pt_ids.append(lm->id_);
        ba.AddPoint(lm->id_, lm->GetGlobalPos(), point_constant);
      }
      const auto& obs = lm_obs.second;
      ba.AddPointProjectionObservation(shot->id_, lm->id_, obs.point, obs.scale);
    }
  }
  for (auto* shot : boundary) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.count(lm)) {
        const auto& obs = lm_obs.second;
        ba.AddPointProjectionObservation(shot->id_, lm->id_, obs.point, obs.scale);
      }
    }
  }

  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, map, gcp, config);
  }

  // Loss, priors, solver
  ba.SetPointProjectionLossFunction(
      config["loss_function"].cast<std::string>(),
      config["loss_function_threshold"].cast<double>());
  ba.SetInternalParametersPriorSD(
      config["exif_focal_sd"].cast<double>(),
      config["principal_point_sd"].cast<double>(),
      config["radial_distortion_k1_sd"].cast<double>(),
      config["radial_distortion_k2_sd"].cast<double>(),
      config["tangential_distortion_p1_sd"].cast<double>(),
      config["tangential_distortion_p2_sd"].cast<double>(),
      config["radial_distortion_k3_sd"].cast<double>(),
      config["radial_distortion_k4_sd"].cast<double>());
  ba.SetRigParametersPriorSD(
      config["rig_translation_sd"].cast<double>(),
      config["rig_rotation_sd"].cast<double>());

  ba.SetNumThreads(config["processes"].cast<int>());
  ba.SetMaxNumIterations(10);
  ba.SetLinearSolverType("DENSE_SCHUR");

  const auto t_setup = std::chrono::high_resolution_clock::now();
  { py::gil_scoped_release release; ba.Run(); }
  const auto t_run = std::chrono::high_resolution_clock::now();

  // Copy back (only things created in this BA)
  for (const auto& iid : rig_instances_ids) {
    auto& instance = map.GetRigInstance(iid);
    auto i = ba.GetRigInstance(iid);
    instance.SetPose(i.GetValue());
  }
  for (auto* point : points) {
    const auto& pt = ba.GetPoint(point->id_);
    point->SetGlobalPos(pt.GetValue());
    point->SetReprojectionErrors(pt.reprojection_errors);
  }

  const auto t_teardown = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_setup - start).count() / 1e6;
  report["wall_times"]["run"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_run - t_setup).count() / 1e6;
  report["wall_times"]["teardown"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_teardown - t_run).count() / 1e6;
  report["num_interior_images"] = interior.size();
  report["num_boundary_images"] = boundary.size();
  report["num_other_images"] = map.NumberOfShots() - interior.size() - boundary.size();

  return py::make_tuple(pt_ids, report);
}

// -----------------------------------------------------------------------------
// GCP triangulation helper
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(
    const map::GroundControlPoint& point,
    const std::unordered_map<map::ShotId, map::Shot>& shots,
    Vec3d& coordinates) {

  constexpr double reproj_threshold = 1.0;
  constexpr double min_angle = 0.1 * M_PI / 180.0;
  constexpr double max_angle = M_PI - min_angle;

  MatX3d bearings;   // unit vectors in world
  MatX3d centers;    // camera centers in world
  bearings.resize(point.observations_.size(), 3);
  centers.resize(point.observations_.size(), 3);

  size_t i = 0;
  for (const auto& o : point.observations_) {
    auto it = shots.find(o.shot_id_);
    if (it == shots.end()) continue;

    const map::Shot& shot = it->second;
    const Eigen::Vector3d b_cam = shot.GetCamera()->Bearing(o.projection_);
    const auto* pose = shot.GetPose();
    const Eigen::Vector3d b_world = pose->RotationCameraToWorld() * b_cam;

    bearings.row(i) = b_world;
    centers.row(i)  = pose->GetOrigin();
    ++i;
  }

  if (i < 2) return false;

  bearings.conservativeResize(i, Eigen::NoChange);
  centers.conservativeResize(i, Eigen::NoChange);

  std::vector<double> thresholds(i, reproj_threshold);

  auto res = ::geometry::TriangulateBearingsMidpoint(centers, bearings, thresholds,
                                                     min_angle, max_angle);
  if (!res.first) return false;

  coordinates = res.second;
  return true;
}

// -----------------------------------------------------------------------------
// Add GCP constraints to BA
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(
    bundle::BundleAdjuster& ba,
    const map::Map& map,
    const AlignedVector<map::GroundControlPoint>& gcp,
    const py::dict& config) {

  const auto& reference = map.GetTopocentricConverter();
  const auto& shots = map.GetShots();

  const auto dominant_terms = ba.GetRigInstances().size() +
                              ba.GetProjectionsCount() +
                              ba.GetRelativeMotionsCount();

  size_t total_terms = 0;
  for (const auto& point : gcp) {
    Vec3d coordinates;
    if (TriangulateGCP(point, shots, coordinates) || !point.lla_.empty()) {
      ++total_terms;
    }
    for (const auto& obs : point.observations_) {
      total_terms += (shots.count(obs.shot_id_) > 0);
    }
  }

  double global_weight = config["gcp_global_weight"].cast<double>() *
                         dominant_terms / std::max<size_t>(1, total_terms);

  size_t added = 0;
  for (const auto& point : gcp) {
    const auto pid = "gcp-" + point.id_;
    Vec3d coordinates;
    if (!TriangulateGCP(point, shots, coordinates)) {
      if (!point.lla_.empty()) {
        coordinates = reference.ToTopocentric(point.GetLlaVec3d());
      } else {
        continue;
      }
    }

    constexpr bool point_constant = false;
    ba.AddPoint(pid, coordinates, point_constant);

    if (!point.lla_.empty()) {
      const auto std_xyz = Vec3d(config["gcp_horizontal_sd"].cast<double>(),
                                 config["gcp_horizontal_sd"].cast<double>(),
                                 config["gcp_vertical_sd"].cast<double>());
      ba.AddPointPrior(pid, reference.ToTopocentric(point.GetLlaVec3d()),
                       std_xyz / global_weight, point.has_altitude_);
    }

    for (const auto& obs : point.observations_) {
      const auto& sid = obs.shot_id_;
      if (shots.count(sid) > 0) {
        constexpr double scale = 0.001;
        ba.AddPointProjectionObservation(sid, pid, obs.projection_, scale / global_weight);
        ++added;
      }
    }
  }
  return added;
}

// -----------------------------------------------------------------------------
// Bundle (global)
// -----------------------------------------------------------------------------
py::dict BAHelpers::Bundle(
    map::Map& map,
    const std::unordered_map<map::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<map::RigCameraId, map::RigCamera>& rig_camera_priors,
    const AlignedVector<map::GroundControlPoint>& gcp,
    const py::dict& config) {

  py::dict report;
  bundle::BundleAdjuster ba;

  const bool fix_cameras = !config["optimize_camera_parameters"].cast<bool>();
  ba.SetUseAnalyticDerivatives(config["bundle_analytic_derivatives"].cast<bool>());

  const auto start = std::chrono::high_resolution_clock::now();

  // Cameras
  const auto& all_cameras = map.GetCameras();
  for (const auto& c : all_cameras) {
    const auto& cam = c.second;
    const auto& cam_prior = camera_priors.at(cam.id);
    ba.AddCamera(cam.id, cam, cam_prior, fix_cameras);
  }

  // Points
  for (const auto& p : map.GetLandmarks()) {
    const auto& lm = p.second;
    ba.AddPoint(lm.id_, lm.GetGlobalPos(), false);
  }

  // Alignment mode
  auto align_method = config["align_method"].cast<std::string>();
  if (align_method == "auto") {
    align_method = DetectAlignmentConstraints(map, config, gcp);
  }

  bool add_up = false;
  Vec3d up = Vec3d::Zero();
  if (align_method == "orientation_prior") {
    const std::string prior = config["align_orientation_prior"].cast<std::string>();
    if (prior == "vertical") { add_up = true; up = Vec3d(0,0,-1); }
    else if (prior == "horizontal") { add_up = true; up = Vec3d(0,-1,0); }
  }

  // Rig cameras: lock lever-arm and optionally lock all if few instances
  constexpr size_t kMinRigInstanceForAdjust = 10;
  const bool lock_rig_camera = map.GetRigInstances().size() <= kMinRigInstanceForAdjust;
  for (const auto& rc_pair : map.GetRigCameras()) {
    const bool is_leverarm = all_cameras.find(rc_pair.first) != all_cameras.end();
    ba.AddRigCamera(rc_pair.first, rc_pair.second.pose,
                    rig_camera_priors.at(rc_pair.first).pose,
                    is_leverarm | lock_rig_camera);
  }

  // Rig instances with averaged GPS priors
  const std::string gps_scale_group = "dummy";
  for (auto inst_pair : map.GetRigInstances()) {
    auto& instance = inst_pair.second;

    Vec3d avg = Vec3d::Zero();
    double avg_std = 0.0;
    int gps_count = 0;

    std::unordered_map<std::string,std::string> shot_cams, shot_rigcams;
    for (const auto& s_rc : instance.GetRigCameras()) {
      const auto sid = s_rc.first;
      const auto& shot = map.GetShot(sid);
      shot_cams[sid] = shot.GetCamera()->id;
      shot_rigcams[sid] = s_rc.second->id;

      if (config["bundle_use_gps"].cast<bool>()) {
        const auto meas = shot.GetShotMeasurements();
        if (meas.gps_position_.HasValue() && meas.gps_accuracy_.HasValue()) {
          avg += meas.gps_position_.Value();
          avg_std += meas.gps_accuracy_.Value();
          ++gps_count;
        }
      }
    }

    ba.AddRigInstance(inst_pair.first, instance.GetPose(), shot_cams, shot_rigcams, false);

    if (config["bundle_use_gps"].cast<bool>() && gps_count > 0) {
      avg /= gps_count;
      avg_std /= gps_count;
      ba.AddRigInstancePositionPrior(inst_pair.first, avg, Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // Shots + observations + up-vector constraints
  for (const auto& sp : map.GetShots()) {
    const auto& shot = sp.second;

    if (add_up) {
      constexpr double std_dev = 1e-3;
      ba.AddAbsoluteUpVector(shot.id_, up, std_dev);
    }

    for (const auto& lm_obs : shot.GetLandmarkObservations()) {
      const auto& obs = lm_obs.second;
      ba.AddPointProjectionObservation(shot.id_, lm_obs.first->id_, obs.point, obs.scale);
    }
  }

  // Add GCP constraints
  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, map, gcp, config);
  }

  // Camera bias compensation (if enabled)
  if (config["bundle_compensate_gps_bias"].cast<bool>()) {
    const auto& biases = map.GetBiases();
    for (const auto& c : map.GetCameras()) {
      ba.SetCameraBias(c.first, biases.at(c.first));
    }
  }

  // Loss, priors, solver
  ba.SetPointProjectionLossFunction(
      config["loss_function"].cast<std::string>(),
      config["loss_function_threshold"].cast<double>());
  ba.SetInternalParametersPriorSD(
      config["exif_focal_sd"].cast<double>(),
      config["principal_point_sd"].cast<double>(),
      config["radial_distortion_k1_sd"].cast<double>(),
      config["radial_distortion_k2_sd"].cast<double>(),
      config["tangential_distortion_p1_sd"].cast<double>(),
      config["tangential_distortion_p2_sd"].cast<double>(),
      config["radial_distortion_k3_sd"].cast<double>(),
      config["radial_distortion_k4_sd"].cast<double>());
  ba.SetRigParametersPriorSD(
      config["rig_translation_sd"].cast<double>(),
      config["rig_rotation_sd"].cast<double>());

  ba.SetNumThreads(config["processes"].cast<int>());
  ba.SetMaxNumIterations(config["bundle_max_iterations"].cast<int>());
  ba.SetLinearSolverType("SPARSE_SCHUR");

  const auto t_setup = std::chrono::high_resolution_clock::now();
  { py::gil_scoped_release release; ba.Run(); }
  const auto t_run = std::chrono::high_resolution_clock::now();

  // Copy back whole BA to map
  BundleToMap(ba, map, !fix_cameras);

  const auto t_teardown = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_setup - start).count() / 1e6;
  report["wall_times"]["run"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_run - t_setup).count() / 1e6;
  report["wall_times"]["teardown"] =
      std::chrono::duration_cast<std::chrono::microseconds>(t_teardown - t_run).count() / 1e6;
  return report;
}

// -----------------------------------------------------------------------------
// Copy BA state back to the map
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& ba,
                            map::Map& out,
                            bool update_cameras) {
  // Cameras
  if (update_cameras) {
    for (auto& cam : out.GetCameras()) {
      const auto& ba_cam = ba.GetCamera(cam.first);
      for (const auto& p : ba_cam.GetParametersMap()) {
        cam.second.SetParameterValue(p.first, p.second);
      }
    }
  }

  // Bias
  for (auto& bias : out.GetBiases()) {
    const auto& new_bias = ba.GetBias(bias.first);
    if (!new_bias.IsValid()) {
      throw std::runtime_error("Bias " + bias.first + " has invalid values.");
    }
    bias.second = new_bias;
  }

  // Rig instances
  for (auto& inst : out.GetRigInstances()) {
    const auto new_val = ba.GetRigInstance(inst.first).GetValue();
    if (!new_val.IsValid()) {
      throw std::runtime_error("Rig Instance " + inst.first + " has invalid values.");
    }
    inst.second.SetPose(new_val);
  }

  // Rig cameras
  for (auto& rc : out.GetRigCameras()) {
    const auto new_val = ba.GetRigCamera(rc.first).GetValue();
    if (!new_val.IsValid()) {
      throw std::runtime_error("Rig Camera " + rc.first + " has invalid values.");
    }
    rc.second.pose = new_val;
  }

  // Points
  for (auto& p : out.GetLandmarks()) {
    const auto& pt = ba.GetPoint(p.first);
    if (!pt.GetValue().allFinite()) {
      throw std::runtime_error("Point " + p.first + " has invalid coordinates.");
    }
    p.second.SetGlobalPos(pt.GetValue());
    p.second.SetReprojectionErrors(pt.reprojection_errors);
  }
}

// -----------------------------------------------------------------------------
// Alignment helpers
// -----------------------------------------------------------------------------
void BAHelpers::AlignmentConstraints(
    const map::Map& map,
    const py::dict& config,
    const AlignedVector<map::GroundControlPoint>& gcp,
    MatX3d& Xp,
    MatX3d& X) {

  size_t reserve = 0;
  const auto& shots = map.GetShots();
  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) reserve += gcp.size();
  if (config["bundle_use_gps"].cast<bool>()) {
    for (const auto& s : shots) {
      if (s.second.GetShotMeasurements().gps_position_.HasValue()) reserve += 1;
    }
  }

  Xp.conservativeResize(reserve, Eigen::NoChange);
  X.conservativeResize(reserve, Eigen::NoChange);

  const auto& top = map.GetTopocentricConverter();
  size_t idx = 0;

  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) {
    for (const auto& pt : gcp) {
      if (pt.lla_.empty()) continue;
      Vec3d coords;
      if (TriangulateGCP(pt, shots, coords)) {
        Xp.row(idx) = top.ToTopocentric(pt.GetLlaVec3d());
        X.row(idx)  = coords;
        ++idx;
      }
    }
  }

  if (config["bundle_use_gps"].cast<bool>()) {
    for (const auto& sp : shots) {
      const auto& shot = sp.second;
      const auto pos = shot.GetShotMeasurements().gps_position_;
      if (pos.HasValue()) {
        Xp.row(idx) = pos.Value();
        X.row(idx)  = shot.GetPose()->GetOrigin();
        ++idx;
      }
    }
  }
}

std::string BAHelpers::DetectAlignmentConstraints(
    const map::Map& map,
    const py::dict& config,
    const AlignedVector<map::GroundControlPoint>& gcp) {

  MatX3d X, Xp;
  AlignmentConstraints(map, config, gcp, Xp, X);

  if (X.rows() < 3) {
    return "orientation_prior";
  }

  const Vec3d mean = X.colwise().mean();
  const MatX3d X0 = X.rowwise() - mean.transpose();

  const Mat3d cov = X0.transpose() * X0;
  Eigen::SelfAdjointEigenSolver<MatXd> es(cov, Eigen::EigenvaluesOnly);
  const Vec3d evals = es.eigenvalues();

  const double ratio = std::abs(evals[2] / evals[1]);
  constexpr double eps_abs = 1e-10;
  constexpr double eps_ratio = 5e3;

  int zeros = 0;
  for (int i = 0; i < 3; ++i) zeros += (evals[i] < eps_abs) ? 1 : 0;
  const bool is_line = zeros > 1 || ratio > eps_ratio;

  return is_line ? std::string("orientation_prior") : std::string("naive");
}

}  // namespace sfm
