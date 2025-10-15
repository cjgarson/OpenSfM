// src/sfm/src/ba_helpers.cc
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>

#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/shot.h>
#include <map/observation.h>

#include <sfm/ba_helpers.h>

namespace py = pybind11;
using std::size_t;

namespace sfm {

// -----------------------------------------------------------------------------
// Neighborhood (IDs)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::ShotId>, std::unordered_set<sfmmap::ShotId>>
BAHelpers::ShotNeighborhoodIds(sfmmap::Map& map,
                               const sfmmap::ShotId& central_shot_id,
                               size_t radius,
                               size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id,
                              radius, min_common_points, max_interior_size);
  std::unordered_set<sfmmap::ShotId> interior_ids, boundary_ids;
  for (sfmmap::Shot* s : res.first)   interior_ids.insert(s->id_);
  for (sfmmap::Shot* s : res.second)  boundary_ids.insert(s->id_);
  return {std::move(interior_ids), std::move(boundary_ids)};
}

// -----------------------------------------------------------------------------
// Reconstructed shots near a given shot
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::Shot*>, std::unordered_set<sfmmap::Shot*>>
BAHelpers::ShotNeighborhood(sfmmap::Map& map,
                            const sfmmap::ShotId& central_shot_id,
                            size_t radius,
                            size_t min_common_points,
                            size_t max_interior_size) {
  constexpr size_t kMaxBoundary{1000000};
  std::unordered_set<sfmmap::Shot*> interior;

  // seed with rig instance of central shot, if available
  auto& central = map.GetShot(central_shot_id);
  const auto inst_id = central.GetRigInstanceId();
  if (map.HasRigInstance(inst_id)) {
    for (const auto& sid : map.GetRigInstance(inst_id).GetShotIDs()) {
      interior.insert(&map.GetShot(sid));
    }
  }
  interior.insert(&central);

  for (size_t d = 1; d < radius && interior.size() < max_interior_size; ++d) {
    const auto remaining = max_interior_size - interior.size();
    auto neighbors = DirectShotNeighbors(map, interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }
  auto boundary = DirectShotNeighbors(map, interior, 1, kMaxBoundary);
  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Shots with many shared points to a set
// -----------------------------------------------------------------------------
std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& map,
                               const std::unordered_set<sfmmap::Shot*>& interior,
                               size_t min_common_points,
                               size_t max_neighbors) {
  // 1) collect landmarks seen by interior
  std::unordered_set<sfmmap::Landmark*> lms;
  for (auto* shot : interior) {
    for (const auto& kv : shot->GetLandmarkObservations()) {
      lms.insert(kv.first);
    }
  }

  // 2) count how many of those landmarks each outside shot also observes
  std::unordered_map<sfmmap::Shot*, size_t> counts;
  for (auto* lm : lms) {
    for (const auto& sh_obs : lm->GetObservations()) {
      auto* sh = sh_obs.first; // Shot*
      if (interior.find(sh) == interior.end()) ++counts[sh];
    }
  }

  // 3) sort by counts desc and expand to rig instances
  std::vector<std::pair<sfmmap::Shot*, size_t>> sorted(counts.begin(), counts.end());
  std::sort(sorted.begin(), sorted.end(),
            [](auto& a, auto& b) { return a.second > b.second; });

  const size_t take = std::min(max_neighbors, sorted.size());
  std::unordered_set<sfmmap::Shot*> out;
  size_t i = 0;
  for (auto& p : sorted) {
    if (p.second < min_common_points || i >= take) break;
    const auto rid = p.first->GetRigInstanceId();
    if (map.HasRigInstance(rid)) {
      for (const auto& sid : map.GetRigInstance(rid).GetShotIDs()) {
        out.insert(&map.GetShot(sid));
      }
    } else {
      out.insert(p.first);
    }
    ++i;
  }
  return out;
}

// -----------------------------------------------------------------------------
// Triangulate a GCP from its observations (bearing+centers midpoint)
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(const sfmmap::GroundControlPoint& point,
                               const sfmmap::Map::ShotMap& shots,
                               Vec3d& coordinates) {
  // observations_ items have fields: shot_id_, projection_ (pixel coords)
  if (point.observations_.size() < 2) return false;

  MatX3d bearings(point.observations_.size(), 3);
  MatX3d centers(point.observations_.size(), 3);
  size_t n = 0;

  for (const auto& obs : point.observations_) {
    auto it = shots.find(obs.shot_id_);
    if (it == shots.end()) continue;
    const sfmmap::Shot& shot = it->second;

    // pixel -> unit bearing in camera, then to world
    const Eigen::Vector3d b_cam = shot.GetCamera()->Bearing(obs.projection_);
    const auto* pose = shot.GetPose();
    const Eigen::Vector3d b_world = pose->RotationCameraToWorld() * b_cam;

    bearings.row(n) = b_world;
    centers.row(n)  = pose->GetOrigin();
    ++n;
  }

  if (n < 2) return false;
  bearings.conservativeResize(n, Eigen::NoChange);
  centers.conservativeResize(n, Eigen::NoChange);

  constexpr double min_angle = 0.1 * M_PI / 180.0;
  constexpr double max_angle = M_PI - min_angle;
  std::vector<double> thresholds(n, 1.0);

  const auto res = ::geometry::TriangulateBearingsMidpoint(
      centers, bearings, thresholds, min_angle, max_angle);
  if (!res.first) return false;
  coordinates = res.second;
  return true;
}

// -----------------------------------------------------------------------------
// Local BA around a central shot  -> (list(point_ids), report)
// -----------------------------------------------------------------------------
py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const sfmmap::Map::CameraMap& camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config) {

  py::dict report;
  const auto t0 = std::chrono::high_resolution_clock::now();

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

  // Cameras: add once, fixed (local BA just optimizes poses + points)
  {
    std::unordered_set<sfmmap::CameraId> added;
    for (auto* s : interior) added.insert(s->GetCamera()->id);
    for (auto* s : boundary) added.insert(s->GetCamera()->id);
    for (const auto& cid : added) {
      const auto& cam     = map.GetCamera(cid);
      const auto& cam_pri = camera_priors.at(cid);
      constexpr bool fix_cameras = true;
      ba.AddCamera(cid, cam, cam_pri, fix_cameras);
    }
  }

  // Collect rig cameras/instances from the neighborhood and add fixed rig cams
  std::unordered_set<sfmmap::RigCameraId>   rig_cam_ids;
  std::unordered_set<sfmmap::RigInstanceId> rig_inst_ids;
  auto collect_rig = [&](sfmmap::Shot* s){
    rig_cam_ids.insert(s->GetRigCameraId());
    rig_inst_ids.insert(s->GetRigInstanceId());
  };
  for (auto* s : interior) collect_rig(s);
  for (auto* s : boundary) collect_rig(s);

  for (const auto& rcid : rig_cam_ids) {
    const auto& rc  = map.GetRigCamera(rcid);
    const auto& rcp = rig_camera_priors.at(rcid);
    constexpr bool fix_rig_camera = true;
    ba.AddRigCamera(rcid, rc.pose, rcp.pose, fix_rig_camera);
  }

  // Add rig instances; fix instance if any of its shots is on the boundary
  const std::string gps_scale_group = "dummy";
  for (const auto& inst_id : rig_inst_ids) {
    auto& inst = map.GetRigInstance(inst_id);
    std::unordered_map<std::string, std::string> shot_cams, shot_rig_cams;

    bool fix_instance = false;
    Vec3d avg_pos = Vec3d::Zero();
    double avg_std = 0.0; int gps_n = 0;

    for (const auto& kv : inst.GetRigCameras()) {
      const auto& sid = kv.first;
      const auto& sh  = map.GetShot(sid);
      shot_cams[sid]       = sh.GetCamera()->id;
      shot_rig_cams[sid]   = kv.second->id;
      const bool is_boundary = boundary.find(&map.GetShot(sid)) != boundary.end();
      if (is_boundary) fix_instance = true;

      if (!is_boundary && config["bundle_use_gps"].cast<bool>()) {
        const auto meas = sh.GetShotMeasurements();
        if (meas.gps_position_.HasValue() && meas.gps_accuracy_.HasValue()) {
          avg_pos += meas.gps_position_.Value();
          avg_std += meas.gps_accuracy_.Value();
          ++gps_n;
        }
      }
    }

    ba.AddRigInstance(inst_id, inst.GetPose(), shot_cams, shot_rig_cams, fix_instance);
    if (!fix_instance && gps_n > 0) {
      avg_pos /= gps_n; avg_std /= gps_n;
      ba.AddRigInstancePositionPrior(inst_id, avg_pos,
                                     Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // Points: add interior points, and boundary projections only if already added
  std::unordered_set<sfmmap::Landmark*> pts;
  py::list adjusted_point_ids;

  constexpr bool kPointConstant = false;

  for (auto* s : interior) {
    for (const auto& kv : s->GetLandmarkObservations()) {
      auto* lm = kv.first;
      if (pts.insert(lm).second) { // newly inserted
        adjusted_point_ids.append(lm->id_);
        ba.AddPoint(lm->id_, lm->GetGlobalPos(), kPointConstant);
      }
      const auto& obs = kv.second; // Observation{point,scale}
      ba.AddPointProjectionObservation(s->id_, lm->id_, obs.point, obs.scale);
    }
  }
  for (auto* s : boundary) {
    for (const auto& kv : s->GetLandmarkObservations()) {
      auto* lm = kv.first;
      if (pts.find(lm) != pts.end()) {
        const auto& obs = kv.second;
        ba.AddPointProjectionObservation(s->id_, lm->id_, obs.point, obs.scale);
      }
    }
  }

  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, map, gcp, config);
  }

  // Loss + priors
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

  // write back instance poses and landmark states/reproj errors
  for (const auto& inst_id : rig_inst_ids) {
    auto inst = ba.GetRigInstance(inst_id);
    map.GetRigInstance(inst_id).SetPose(inst.GetValue());
  }
  for (auto* lm : pts) {
    const auto& p = ba.GetPoint(lm->id_);
    lm->SetGlobalPos(p.GetValue());
    lm->SetReprojectionErrors(p.reprojection_errors); // note plural
  }

  const auto t_done = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"]   = py::dict();
  report["wall_times"]["setup"]    = std::chrono::duration_cast<std::chrono::microseconds>(t_setup - t0).count() / 1e6;
  report["wall_times"]["run"]      = std::chrono::duration_cast<std::chrono::microseconds>(t_run   - t_setup).count() / 1e6;
  report["wall_times"]["teardown"] = std::chrono::duration_cast<std::chrono::microseconds>(t_done  - t_run).count() / 1e6;
  report["num_interior_images"]    = interior.size();
  report["num_boundary_images"]    = boundary.size();
  report["num_other_images"]       = map.NumberOfShots() - interior.size() - boundary.size();

  return py::make_tuple(adjusted_point_ids, report);
}

// -----------------------------------------------------------------------------
// Global BA
// -----------------------------------------------------------------------------
py::dict BAHelpers::Bundle(sfmmap::Map& map,
                           const sfmmap::Map::CameraMap& camera_priors,
                           const sfmmap::Map::RigCameraMap& rig_camera_priors,
                           const AlignedVector<sfmmap::GroundControlPoint>& gcp,
                           const py::dict& config) {
  py::dict report;

  bundle::BundleAdjuster ba;
  const bool fix_cameras = !config["optimize_camera_parameters"].cast<bool>();
  ba.SetUseAnalyticDerivatives(config["bundle_analytic_derivatives"].cast<bool>());
  const auto t0 = std::chrono::high_resolution_clock::now();

  // cameras
  for (const auto& kv : map.GetCameras()) {
    const auto& cam = kv.second;
    const auto& pri = camera_priors.at(cam.id);
    ba.AddCamera(cam.id, cam, pri, fix_cameras);
  }
  // points
  for (const auto& kv : map.GetLandmarks()) {
    const auto& lm = kv.second;
    ba.AddPoint(lm.id_, lm.GetGlobalPos(), false);
  }

  // alignment method
  std::string align = config["align_method"].cast<std::string>();
  if (align == "auto") align = DetectAlignmentConstraints(map, config, gcp);
  bool add_up = false; Vec3d up = Vec3d::Zero();
  if (align == "orientation_prior") {
    const std::string prior = config["align_orientation_prior"].cast<std::string>();
    if      (prior == "vertical")   { add_up = true; up = Vec3d(0, 0, -1); }
    else if (prior == "horizontal") { add_up = true; up = Vec3d(0, -1, 0); }
  }

  // rig cameras (mostly fixed unless lots of instances)
  constexpr size_t kMinRigInstanceForAdjust = 10;
  const bool lock_rig_camera = map.GetRigInstances().size() <= kMinRigInstanceForAdjust;
  for (const auto& kv : map.GetRigCameras()) {
    const auto& rc  = kv.second;
    const auto& prc = rig_camera_priors.at(kv.first);
    const bool is_leverarm = (map.GetCameras().find(kv.first) != map.GetCameras().end());
    ba.AddRigCamera(kv.first, rc.pose, prc.pose, is_leverarm || lock_rig_camera);
  }

  // rig instances (+ averaged GPS priors if requested)
  const std::string gps_scale_group = "dummy";
  for (const auto& kv : map.GetRigInstances()) {
    const auto& inst_id = kv.first;
    const auto& inst    = kv.second;

    Vec3d avg_pos = Vec3d::Zero(); double avg_std = 0.0; int gps_n = 0;
    std::unordered_map<std::string, std::string> shot_cams, shot_rig_cams;

    for (const auto& s_rc : inst.GetRigCameras()) {
      const auto sid = s_rc.first;
      const auto& sh = map.GetShot(sid);
      shot_cams[sid]     = sh.GetCamera()->id;
      shot_rig_cams[sid] = s_rc.second->id;

      if (config["bundle_use_gps"].cast<bool>()) {
        const auto meas = sh.GetShotMeasurements();
        if (meas.gps_position_.HasValue() && meas.gps_accuracy_.HasValue()) {
          avg_pos += meas.gps_position_.Value();
          avg_std += meas.gps_accuracy_.Value();
          ++gps_n;
        }
      }
    }

    ba.AddRigInstance(inst_id, inst.GetPose(), shot_cams, shot_rig_cams, false);
    if (config["bundle_use_gps"].cast<bool>() && gps_n > 0) {
      avg_pos /= gps_n; avg_std /= gps_n;
      ba.AddRigInstancePositionPrior(inst_id, avg_pos, Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // observations (and optional up-vector constraints)
  for (const auto& kvs : map.GetShots()) {
    const auto& s = kvs.second;
    if (add_up) {
      constexpr double std_dev = 1e-3;
      ba.AddAbsoluteUpVector(s.id_, up, std_dev);
    }
    for (const auto& kv : s.GetLandmarkObservations()) {
      const auto* lm = kv.first;
      const auto& ob = kv.second;
      ba.AddPointProjectionObservation(s.id_, lm->id_, ob.point, ob.scale);
    }
  }

  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, map, gcp, config);
  }

  if (config["bundle_compensate_gps_bias"].cast<bool>()) {
    const auto& biases = map.GetBiases();
    for (const auto& cam : map.GetCameras()) {
      ba.SetCameraBias(cam.first, biases.at(cam.first));
    }
  }

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
  const auto t_run   = std::chrono::high_resolution_clock::now();

  BundleToMap(ba, map, !fix_cameras);

  const auto t_done  = std::chrono::high_resolution_clock::now();
  report["brief_report"]            = ba.BriefReport();
  report["wall_times"]              = py::dict();
  report["wall_times"]["setup"]     = std::chrono::duration_cast<std::chrono::microseconds>(t_setup - t0).count() / 1e6;
  report["wall_times"]["run"]       = std::chrono::duration_cast<std::chrono::microseconds>(t_run   - t_setup).count() / 1e6;
  report["wall_times"]["teardown"]  = std::chrono::duration_cast<std::chrono::microseconds>(t_done  - t_run).count() / 1e6;
  return report;
}

// -----------------------------------------------------------------------------
// Copy BA state back to map
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& ba,
                            sfmmap::Map& output_map,
                            bool update_cameras) {
  // cameras
  if (update_cameras) {
    for (auto& kv : output_map.GetCameras()) {
      auto& cam = kv.second;
      const auto& bcam = ba.GetCamera(kv.first);
      for (const auto& p : bcam.GetParametersMap()) {
        cam.SetParameterValue(p.first, p.second);
      }
    }
  }
  // biases
  for (auto& kv : output_map.GetBiases()) {
    const auto& new_bias = ba.GetBias(kv.first);
    if (!new_bias.IsValid()) {
      throw std::runtime_error("Bias " + kv.first + " is invalid (NaN/INF).");
    }
    kv.second = new_bias;
  }
  // rig instances
  for (auto& kv : output_map.GetRigInstances()) {
    const auto new_pose = ba.GetRigInstance(kv.first).GetValue();
    if (!new_pose.IsValid()) {
      throw std::runtime_error("RigInstance " + kv.first + " is invalid (NaN/INF).");
    }
    kv.second.SetPose(new_pose);
  }
  // rig cameras
  for (auto& kv : output_map.GetRigCameras()) {
    const auto new_pose = ba.GetRigCamera(kv.first).GetValue();
    if (!new_pose.IsValid()) {
      throw std::runtime_error("RigCamera " + kv.first + " is invalid (NaN/INF).");
    }
    kv.second.pose = new_pose;
  }
  // landmarks
  for (auto& kv : output_map.GetLandmarks()) {
    auto& lm = kv.second;
    const auto& p = ba.GetPoint(lm.id_);
    if (!p.GetValue().allFinite()) {
      throw std::runtime_error("Point " + lm.id_ + " is invalid (NaN/INF).");
    }
    lm.SetGlobalPos(p.GetValue());
    lm.SetReprojectionErrors(p.reprojection_errors); // note plural
  }
}

// -----------------------------------------------------------------------------
// Alignment helpers
// -----------------------------------------------------------------------------
void BAHelpers::AlignmentConstraints(const sfmmap::Map& map,
                                     const py::dict& config,
                                     const AlignedVector<sfmmap::GroundControlPoint>& gcp,
                                     MatX3d& Xp, MatX3d& X) {
  size_t reserve = 0;
  const auto& shots = map.GetShots();
  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) {
    reserve += gcp.size();
  }
  if (config["bundle_use_gps"].cast<bool>()) {
    for (const auto& kv : shots) {
      const auto& sh = kv.second;
      if (sh.GetShotMeasurements().gps_position_.HasValue()) ++reserve;
    }
  }
  Xp.conservativeResize(reserve, Eigen::NoChange);
  X .conservativeResize(reserve, Eigen::NoChange);

  const auto& topo = map.GetTopocentricConverter();
  size_t idx = 0;

  if (!gcp.empty() && config["bundle_use_gcp"].cast<bool>()) {
    for (const auto& p : gcp) {
      if (p.lla_.empty()) continue;
      Vec3d coords;
      if (TriangulateGCP(p, shots, coords)) {
        Xp.row(idx) = topo.ToTopocentric(p.GetLlaVec3d());
        X .row(idx) = coords;
        ++idx;
      }
    }
  }
  if (config["bundle_use_gps"].cast<bool>()) {
    for (const auto& kv : shots) {
      const auto& sh = kv.second;
      const auto pos = sh.GetShotMeasurements().gps_position_;
      if (pos.HasValue()) {
        Xp.row(idx) = pos.Value();
        X .row(idx) = sh.GetPose()->GetOrigin();
        ++idx;
      }
    }
  }
}

std::string BAHelpers::DetectAlignmentConstraints(const sfmmap::Map& map,
                                                  const py::dict& config,
                                                  const AlignedVector<sfmmap::GroundControlPoint>& gcp) {
  MatX3d Xp, X;
  AlignmentConstraints(map, config, gcp, Xp, X);
  if (X.rows() < 3) return "orientation_prior";

  const Vec3d X_mean = X.colwise().mean();
  const MatX3d X0 = X.rowwise() - X_mean.transpose();
  const Mat3d C = X0.transpose() * X0;
  Eigen::SelfAdjointEigenSolver<MatXd> ses(C, Eigen::EigenvaluesOnly);
  const Vec3d evals = ses.eigenvalues();

  // line detection: extremely anisotropic covariance
  const double ratio = std::abs(evals[2] / std::max(1e-12, evals[1]));
  int small = 0; for (int i=0;i<3;++i) small += (evals[i] < 1e-10) ? 1 : 0;
  const bool is_line = small > 1 || ratio > 5e3;
  return is_line ? std::string("orientation_prior") : std::string("naive");
}

// -----------------------------------------------------------------------------
// Add GCP constraints
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(bundle::BundleAdjuster& ba,
                                 const sfmmap::Map& map,
                                 const AlignedVector<sfmmap::GroundControlPoint>& gcp,
                                 const py::dict& config) {
  const auto& topo  = map.GetTopocentricConverter();
  const auto& shots = map.GetShots();

  const auto dominant_terms = ba.GetRigInstances().size() +
                              ba.GetProjectionsCount() +
                              ba.GetRelativeMotionsCount();

  size_t total_terms = 0;
  for (const auto& p : gcp) {
    Vec3d coords;
    if (TriangulateGCP(p, shots, coords) || !p.lla_.empty()) ++total_terms;
    for (const auto& o : p.observations_) {
      total_terms += (shots.count(o.shot_id_) > 0);
    }
  }
  double global_weight = config["gcp_global_weight"].cast<double>() *
                         dominant_terms / std::max<size_t>(1, total_terms);

  size_t added = 0;
  for (const auto& p : gcp) {
    const std::string pid = "gcp-" + p.id_;
    Vec3d coords;
    if (!TriangulateGCP(p, shots, coords)) {
      if (!p.lla_.empty()) {
        coords = topo.ToTopocentric(p.GetLlaVec3d());
      } else {
        continue;
      }
    }
    constexpr bool kPointConstant = false;
    ba.AddPoint(pid, coords, kPointConstant);

    if (!p.lla_.empty()) {
      const Vec3d stdv(config["gcp_horizontal_sd"].cast<double>(),
                       config["gcp_horizontal_sd"].cast<double>(),
                       config["gcp_vertical_sd"].cast<double>());
      ba.AddPointPrior(pid, topo.ToTopocentric(p.GetLlaVec3d()),
                       stdv / global_weight, p.has_altitude_);
    }

    for (const auto& o : p.observations_) {
      if (shots.count(o.shot_id_) == 0) continue;
      constexpr double scale = 0.001;
      ba.AddPointProjectionObservation(o.shot_id_, pid, o.projection_,
                                       scale / global_weight);
      ++added;
    }
  }
  return added;
}

// -----------------------------------------------------------------------------
// Poses-only BA (fix points/cameras/rig cams)
// -----------------------------------------------------------------------------
py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<std::string>& shot_ids,
    const sfmmap::Map::CameraMap& camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const py::dict& config) {
  py::dict report;

  bundle::BundleAdjuster ba;
  ba.SetUseAnalyticDerivatives(config["bundle_analytic_derivatives"].cast<bool>());
  const auto t0 = std::chrono::high_resolution_clock::now();

  constexpr bool fix_cameras    = true;
  constexpr bool fix_points     = true;
  constexpr bool fix_rig_camera = true;

  // collect rigs involved
  std::unordered_set<sfmmap::RigInstanceId> inst_ids;
  for (const auto& sid : shot_ids) inst_ids.insert(map.GetShot(sid).GetRigInstanceId());
  std::unordered_set<sfmmap::RigCameraId> rc_ids;
  for (const auto& iid : inst_ids) {
    for (const auto& kv : map.GetRigInstance(iid).GetRigCameras()) {
      rc_ids.insert(kv.second->id);
    }
  }

  for (const auto& rcid : rc_ids) {
    const auto& rc  = map.GetRigCamera(rcid);
    const auto& rcp = rig_camera_priors.at(rcid);
    ba.AddRigCamera(rcid, rc.pose, rcp.pose, fix_rig_camera);
  }

  // cameras: add once
  std::unordered_set<sfmmap::CameraId> added_cams;
  for (const auto& sid : shot_ids) {
    const auto& sh = map.GetShot(sid);
    const auto& cam_id = sh.GetCamera()->id;
    if (!added_cams.insert(cam_id).second) continue;
    const auto& cam     = map.GetCamera(cam_id);
    const auto& cam_pri = camera_priors.at(cam_id);
    ba.AddCamera(cam_id, cam, cam_pri, fix_cameras);
  }

  // points observed by those shots (fixed)
  std::unordered_set<sfmmap::Landmark*> lms;
  for (const auto& sid : shot_ids) {
    const auto& sh = map.GetShot(sid);
    for (const auto& kv : sh.GetLandmarkObservations()) lms.insert(kv.first);
  }
  for (auto* lm : lms) {
    ba.AddPoint(lm->id_, lm->GetGlobalPos(), fix_points);
  }

  // rig instances + optional averaged GPS constraints
  const std::string gps_scale_group = "dummy";
  for (const auto& iid : inst_ids) {
    const auto& inst = map.GetRigInstance(iid);
    std::unordered_map<std::string, std::string> shot_cams, shot_rig_cams;

    Vec3d avg_pos = Vec3d::Zero(); double avg_std = 0.0; int gps_n = 0;
    bool fix_inst = false;

    for (const auto& kv : inst.GetRigCameras()) {
      const auto sid = kv.first;
      const auto& sh = map.GetShot(sid);
      shot_cams[sid]     = sh.GetCamera()->id;
      shot_rig_cams[sid] = kv.second->id;

      const bool this_shot_is_optimized = (shot_ids.find(sid) != shot_ids.end());
      if (this_shot_is_optimized) {
        fix_inst = true;  // if any shot is optimized, keep instance fixed (poses-only mode)
      } else if (config["bundle_use_gps"].cast<bool>()) {
        const auto meas = sh.GetShotMeasurements();
        if (meas.gps_position_.HasValue() && meas.gps_accuracy_.HasValue()) {
          avg_pos += meas.gps_position_.Value();
          avg_std += meas.gps_accuracy_.Value();
          ++gps_n;
        }
      }
    }

    ba.AddRigInstance(iid, inst.GetPose(), shot_cams, shot_rig_cams, fix_inst);
    if (!fix_inst && gps_n > 0) {
      avg_pos /= gps_n; avg_std /= gps_n;
      ba.AddRigInstancePositionPrior(iid, avg_pos, Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // observations for the selected shots
  for (const auto& sid : shot_ids) {
    const auto& sh = map.GetShot(sid);
    for (const auto& kv : sh.GetLandmarkObservations()) {
      const auto* lm = kv.first;
      const auto& ob = kv.second;
      ba.AddPointProjectionObservation(sh.id_, lm->id_, ob.point, ob.scale);
    }
  }


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

  ba.SetNumThreads(16); //cg
  ba.SetMaxNumIterations(7);
  ba.SetLinearSolverType("DENSE_QR"); //cg

  
  const auto t_setup = std::chrono::high_resolution_clock::now();
  
/*  if (shot_ids.size() <= 1) {
      report["brief_report"] = "Skipped pose-only BA (single shot)";
      report["num_shots"] = static_cast<int>(shot_ids.size());
      report["map_shots"] = static_cast<int>(map.GetShots().size());
      return report;
  } */
  
  { py::gil_scoped_release release; ba.Run(); }
  const auto t_run   = std::chrono::high_resolution_clock::now();

  // write back only instance poses (points are fixed)
  for (const auto& iid : inst_ids) {
    auto inst = ba.GetRigInstance(iid);
    map.GetRigInstance(iid).SetPose(inst.GetValue());
  }

  const auto t_done = std::chrono::high_resolution_clock::now();
  
  report["brief_report"]            = ba.BriefReport();
  report["wall_times"]              = py::dict();
  report["wall_times"]["setup"]     = std::chrono::duration_cast<std::chrono::microseconds>(t_setup - t0).count() / 1e6;
  report["wall_times"]["run"]       = std::chrono::duration_cast<std::chrono::microseconds>(t_run   - t_setup).count() / 1e6;
  report["wall_times"]["teardown"]  = std::chrono::duration_cast<std::chrono::microseconds>(t_done  - t_run).count() / 1e6;
  report["num_shots"]               = static_cast<int>(shot_ids.size());
  report["map_shots"]               = static_cast<int>(map.GetShots().size());
  return report;
}

} // namespace sfm
