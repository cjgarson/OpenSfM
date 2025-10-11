// src/sfm/src/ba_helpers.cc
#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>
#include <map/ground_control_points.h>
#include <map/map.h>
#include <sfm/ba_helpers.h>

#include <pybind11/pybind11.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "geo/geo.h"
#include "map/defines.h"

namespace py = pybind11;

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
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points, max_interior_size);

  std::unordered_set<sfmmap::ShotId> interior;
  for (sfmmap::Shot* shot : res.first) {
    interior.insert(shot->GetId());
  }

  std::unordered_set<sfmmap::ShotId> boundary;
  for (sfmmap::Shot* shot : res.second) {
    boundary.insert(shot->GetId());
  }

  return std::make_pair(std::move(interior), std::move(boundary));
}

// -----------------------------------------------------------------------------
// Neighborhood (pointers)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::Shot*>, std::unordered_set<sfmmap::Shot*>>
BAHelpers::ShotNeighborhood(sfmmap::Map& map,
                            const sfmmap::ShotId& central_shot_id,
                            size_t radius,
                            size_t min_common_points,
                            size_t max_interior_size) {
  constexpr size_t kMaxBoundarySize{1000000};

  std::unordered_set<sfmmap::Shot*> interior;

  auto& central_shot = map.GetShot(central_shot_id);
  const auto instance_shots =
      map.GetRigInstance(central_shot.GetRigInstanceId()).GetShotIDs();

  for (const auto& s : instance_shots) {
    interior.insert(&map.GetShot(s));
  }
  interior.insert(&central_shot);

  for (size_t distance = 1;
       distance < radius && interior.size() < max_interior_size; ++distance) {
    const auto remaining = max_interior_size - interior.size();
    const auto neighbors =
        DirectShotNeighbors(map, interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }

  const auto boundary = DirectShotNeighbors(map, interior, 1, kMaxBoundarySize);
  return std::make_pair(std::move(interior), std::move(const_cast<std::unordered_set<sfmmap::Shot*>&>(boundary)));
}

// -----------------------------------------------------------------------------
// Direct neighbors
// -----------------------------------------------------------------------------
std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& map,
                               const std::unordered_set<sfmmap::Shot*>& shot_ids,
                               const size_t min_common_points,
                               const size_t max_neighbors) {
  std::unordered_set<sfmmap::Landmark*> points;
  for (auto* shot : shot_ids) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      points.insert(lm_obs.first);
    }
  }

  std::unordered_map<sfmmap::Shot*, size_t> common_points;
  for (auto* pt : points) {
    for (const auto& neighbor_p : pt->GetObservations()) {
      auto* shot = neighbor_p.first;
      if (shot_ids.find(shot) == shot_ids.end()) {
        ++common_points[shot];
      }
    }
  }

  std::vector<std::pair<sfmmap::Shot*, size_t>> pairs(common_points.begin(), common_points.end());
  std::sort(pairs.begin(), pairs.end(),
            [](const std::pair<sfmmap::Shot*, size_t>& a,
               const std::pair<sfmmap::Shot*, size_t>& b) {
              return a.second > b.second;
            });

  const size_t max_n = std::min<size_t>(max_neighbors, pairs.size());
  std::unordered_set<sfmmap::Shot*> neighbors;

  size_t idx = 0;
  for (auto& p : pairs) {
    if (p.second >= min_common_points && idx < max_n) {
      const auto instance_shots =
          map.GetRigInstance(p.first->GetRigInstanceId()).GetShotIDs();
      for (const auto& s : instance_shots) {
        neighbors.insert(&map.GetShot(s));
      }
    } else {
      break;
    }
    ++idx;
  }
  return neighbors;
}

// -----------------------------------------------------------------------------
// Local bundle
// -----------------------------------------------------------------------------
py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const sfmmap::ShotId& central_shot_id,
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

  // Cameras
  for (const auto& cam_pair : map.GetCameras()) {
    const auto& cam = cam_pair.second;
    const auto& cam_prior = camera_priors.at(cam.id);
    constexpr bool kFixCameras{true};
    ba.AddCamera(cam.id, cam, cam_prior, kFixCameras);
  }

  // Sets
  std::unordered_set<sfmmap::Shot*> int_and_bound(interior.cbegin(), interior.cend());
  int_and_bound.insert(boundary.cbegin(), boundary.cend());

  std::unordered_set<sfmmap::Landmark*> points;
  py::list pt_ids;

  constexpr bool kPointConstant{false};
  constexpr bool kRigCameraConstant{true};

  // Rigs present in neighborhood
  std::unordered_set<sfmmap::RigCameraId> rig_cameras_ids;
  std::unordered_set<sfmmap::RigInstanceId> rig_instances_ids;
  for (auto* shot : int_and_bound) {
    rig_cameras_ids.insert(shot->GetRigCameraId());
    rig_instances_ids.insert(shot->GetRigInstanceId());
  }

  // Rig cameras
  for (const auto& rig_camera_id : rig_cameras_ids) {
    const auto& rig_camera = map.GetRigCamera(rig_camera_id);
    ba.AddRigCamera(rig_camera_id,
                    rig_camera.pose,
                    rig_camera_priors.at(rig_camera_id).pose,
                    kRigCameraConstant);
  }

  // Rig instances and GPS priors
  const std::string gps_scale_group = "dummy";
  for (const auto& rig_instance_id : rig_instances_ids) {
    auto& instance = map.GetRigInstance(rig_instance_id);
    std::unordered_map<std::string, std::string> shot_cameras, shot_rig_cameras;

    Vec3d avg_pos = Vec3d::Zero();
    double avg_std = 0.0;
    int gps_count = 0;
    bool fix_instance = false;

    for (const auto& shot_n_rig_camera : instance.GetRigCameras()) {
      const auto shot_id = shot_n_rig_camera.first;
      auto& shot = map.GetShot(shot_id);
      shot_cameras[shot_id] = shot.GetCamera()->id;
      shot_rig_cameras[shot_id] = shot_n_rig_camera.second->id;

      const auto is_boundary = boundary.find(&shot) != boundary.end();
      if (!is_boundary) {
        const auto& m = shot.GetShotMeasurements();
        if (config["bundle_use_gps"].cast<bool>() && m.gps_position_.HasValue()) {
          avg_pos += m.gps_position_.Value();
          avg_std += m.gps_accuracy_.Value();
          ++gps_count;
        }
      } else {
        fix_instance = true;
      }
    }

    ba.AddRigInstance(rig_instance_id, instance.GetPose(),
                      shot_cameras, shot_rig_cameras, fix_instance);

    if (!fix_instance && gps_count > 0) {
      avg_pos /= gps_count;
      avg_std /= gps_count;
      ba.AddRigInstancePositionPrior(rig_instance_id, avg_pos,
                                     Vec3d::Constant(avg_std), gps_scale_group);
    }
  }

  // Points / projections (interior)
  for (auto* shot : interior) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.count(lm) == 0) {
        points.insert(lm);
        pt_ids.append(lm->id_);
        ba.AddPoint(lm->id_, lm->GetGlobalPos(), kPointConstant);
      }
      const auto& obs = lm_obs.second;
      ba.AddPointProjectionObservation(shot->id_, lm_obs.first->id_, obs.point, obs.scale);
    }
  }

  // Boundary projections to interior points
  for (auto* shot : boundary) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      if (points.count(lm) > 0) {
        const auto& obs = lm_obs.second;
        ba.AddPointProjectionObservation(shot->id_, lm_obs.first->id_, obs.point, obs.scale);
      }
    }
  }

  // GCP
  if (config["bundle_use_gcp"].cast<bool>() && !gcp.empty()) {
    AddGCPToBundle(ba, map, gcp, config);
  }

  // BA options
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
  ba.SetRigParametersPriorSD(config["rig_translation_sd"].cast<double>(),
                             config["rig_rotation_sd"].cast<double>());

  ba.SetNumThreads(config["processes"].cast<int>());
  ba.SetMaxNumIterations(10);
  ba.SetLinearSolverType("DENSE_SCHUR");

  const auto timer_setup = std::chrono::high_resolution_clock::now();

  {
    py::gil_scoped_release release;
    ba.Run();
  }

  const auto timer_run = std::chrono::high_resolution_clock::now();

  // Write back rig instances
  for (const auto& rig_instance_id : rig_instances_ids) {
    auto& instance = map.GetRigInstance(rig_instance_id);
    auto i = ba.GetRigInstance(rig_instance_id);
    instance.SetPose(i.GetValue());
  }

  // Write back points
  for (auto* point : points) {
    const auto& pt = ba.GetPoint(point->id_);
    point->SetGlobalPos(pt.GetValue());
    point->SetReprojectionErrors(pt.reprojection_errors);
  }

  const auto timer_teardown = std::chrono::high_resolution_clock::now();
  report["brief_report"] = ba.BriefReport();
  report["wall_times"] = py::dict();
  report["wall_times"]["setup"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start).count() / 1e6;
  report["wall_times"]["run"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_run - timer_setup).count() / 1e6;
  report["wall_times"]["teardown"] =
      std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown - timer_run).count() / 1e6;
  report["num_interior_images"] = interior.size();
  report["num_boundary_images"] = boundary.size();
  report["num_other_images"] = map.NumberOfShots() - interior.size() - boundary.size();

  return py::make_tuple(pt_ids, report);
}

// -----------------------------------------------------------------------------
// GCP Triangulation helper
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(
    const sfmmap::GroundControlPoint& point,
    const sfmmap::Map::ShotMap& shots,
    Vec3d& coordinates) {
  constexpr auto reproj_threshold{1.0};
  constexpr auto min_ray_angle = 0.1 * M_PI / 180.0;
  constexpr auto max_ray_angle = M_PI - min_ray_angle;

  MatX3d os, bs;
  size_t added = 0;
  coordinates = Vec3d::Zero();

  bs.conservativeResize(point.observations_.size(), Eigen::NoChange);
  os.conservativeResize(point.observations_.size(), Eigen::NoChange);

  for (const auto& obs : point.observations_) {
    const auto shot_it = shots.find(obs.shot_id_);
    if (shot_it != shots.end()) {
      const auto& shot = shot_it->second;
      const Vec3d bearing = shot.GetCamera()->Bearing(obs.projection_);
      const auto* shot_pose = shot.GetPose();
      bs.row(added) = shot_pose->RotationCameraToWorld() * bearing;
      os.row(added) = shot_pose->GetOrigin();
      ++added;
    }
  }

  bs.conservativeResize(added, Eigen::NoChange);
  os.conservativeResize(added, Eigen::NoChange);

  if (added >= 2) {
    const std::vector<double> thresholds(added, reproj_threshold);
    const auto& res = geometry::TriangulateBearingsMidpoint(
        os, bs, thresholds, min_ray_angle, max_ray_angle);
    coordinates = res.second;
    return res.first;
  }
  return false;
}

py::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const py::dict& config) {
  py::dict report;
  report["status"] = "Bundle not implemented";
  report["shots"] = static_cast<int>(map.NumberOfShots());
  return report;
}

void BAHelpers::BundleToMap(const bundle::BundleAdjuster& bundle_adjuster,
                            sfmmap::Map& output_map,
                            bool update_cameras) {
  // No-op stub
  (void)bundle_adjuster;
  (void)output_map;
  (void)update_cameras;
}

std::string BAHelpers::DetectAlignmentConstraints(
    const sfmmap::Map& map,
    const py::dict& config,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp) {
  (void)map;
  (void)config;
  (void)gcp;
  return "DetectAlignmentConstraints not implemented";
}

void BAHelpers::AlignmentConstraints(
    const sfmmap::Map& map,
    const py::dict& config,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    MatX3d& Xp,
    MatX3d& X) {
  (void)map;
  (void)config;
  (void)gcp;
  Xp.resize(0, 3);
  X.resize(0, 3);
}

// -----------------------------------------------------------------------------
// Add GCP to BA
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(
    bundle::BundleAdjuster& ba,
    const sfmmap::Map& map,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const py::dict& config) {

  const auto& reference = map.GetTopocentricConverter();
  const auto& shots = map.GetShots();

  const auto dominant_terms = ba.GetRigInstances().size()
                            + ba.GetProjectionsCount()
                            + ba.GetRelativeMotionsCount();

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

  const double global_weight =
      config["gcp_global_weight"].cast<double>() *
      dominant_terms / std::max<size_t>(1, total_terms);

  size_t added = 0;
  for (const auto& point : gcp) {
    const auto point_id = std::string("gcp-") + point.id_;
    Vec3d coordinates;

    if (!TriangulateGCP(point, shots, coordinates)) {
      if (!point.lla_.empty()) {
        coordinates = reference.ToTopocentric(point.GetLlaVec3d());
      } else {
        continue;
      }
    }

    constexpr auto kPointConstant{false};
    ba.AddPoint(point_id, coordinates, kPointConstant);

    if (!point.lla_.empty()) {
      const auto point_std = Vec3d(
          config["gcp_horizontal_sd"].cast<double>(),
          config["gcp_horizontal_sd"].cast<double>(),
          config["gcp_vertical_sd"].cast<double>());
      ba.AddPointPrior(point_id,
                       reference.ToTopocentric(point.GetLlaVec3d()),
                       point_std / global_weight,
                       point.has_altitude_);
    }

    for (const auto& obs : point.observations_) {
      const auto& shot_id = obs.shot_id_;
      if (shots.count(shot_id) > 0) {
        constexpr double scale{0.001};
        ba.AddPointProjectionObservation(shot_id, point_id, obs.projection_,
                                         scale / global_weight);
        ++added;
      }
    }
  }
  return added;
}

}  // namespace sfm
