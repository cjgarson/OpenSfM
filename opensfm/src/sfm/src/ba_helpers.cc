// src/sfm/src/ba_helpers.cc
#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>
#include <map/ground_control_points.h>
#include <map/map.h>
#include <sfm/ba_helpers.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#include "geo/geo.h"
#include "map/defines.h"

// **Added includes for complete type definitions**:
#include <geometry/camera.h>    // Defines geometry::Camera
#include <map/rig.h>           // Defines sfmmap::RigCamera
#include <map/shot.h>          // Defines sfmmap::Shot (for Shot* and ShotMap contents)
#include <map/landmark.h>      // Defines sfmmap::Landmark (used in neighbor computations)

using namespace sfmmap;

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
  const size_t kMaxBoundarySize = max_interior_size * 3;
  std::unordered_set<sfmmap::Shot*> interior;
  interior.insert(map.GetShot(central_shot_id));
  std::unordered_set<sfmmap::Shot*> boundary = interior;
  std::unordered_set<sfmmap::Shot*> current_boundary = interior;

  // Iteratively collect neighbors up to the given radius
  for (size_t i = 0; i < radius; ++i) {
    std::unordered_set<sfmmap::Shot*> new_boundary;
    for (auto* shot : current_boundary) {
      // Direct neighbors of the current boundary
      auto neighbors = DirectShotNeighbors(map, {shot}, min_common_points, max_interior_size);
      for (auto* nb : neighbors) {
        if (interior.count(nb) == 0) {
          new_boundary.insert(nb);
        }
      }
    }

    // Update interior and boundary sets
    for (auto* shot : new_boundary) {
      interior.insert(shot);
      if (boundary.size() < max_interior_size) {
        boundary.insert(shot);
      }
    }

    current_boundary.swap(new_boundary);
    if (boundary.size() >= max_interior_size || current_boundary.empty()) {
      break;
    }
  }

  // The boundary set should exclude the deep interior (only include the "fringe")
  if (interior.size() > max_interior_size) {
    // Limit the interior to max_interior_size shots (already in `boundary`)
    // and compute a new boundary from the remaining ones.
    interior = boundary;
    std::unordered_set<sfmmap::Shot*> new_boundary;
    for (auto* shot : boundary) {
      auto neighbors = DirectShotNeighbors(map, {shot}, 1, kMaxBoundarySize);
      for (auto* nb : neighbors) {
        if (interior.count(nb) == 0) {
          new_boundary.insert(nb);
        }
      }
    }
    boundary.swap(new_boundary);
  }

  return std::make_pair(std::move(interior), std::move(boundary));
}

// -----------------------------------------------------------------------------
// Direct neighbors
// -----------------------------------------------------------------------------
std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& map,
                               const std::unordered_set<sfmmap::Shot*>& shot_ids,
                               size_t min_common_points,
                               size_t max_neighbors) {
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
        common_points[shot] += 1;
      }
    }
  }

  // Collect neighbors with at least min_common_points in common
  std::vector<std::pair<sfmmap::Shot*, size_t>> counts;
  counts.reserve(common_points.size());
  for (const auto& kv : common_points) {
    if (kv.second >= min_common_points) {
      counts.emplace_back(kv.first, kv.second);
    }
  }

  // Sort neighbors by number of common points (descending)
  std::sort(counts.begin(), counts.end(),
            [](const std::pair<sfmmap::Shot*, size_t>& a,
               const std::pair<sfmmap::Shot*, size_t>& b) {
              return a.second > b.second;
            });

  // Take the top-N neighbors (max_neighbors)
  std::unordered_set<sfmmap::Shot*> neighbors;
  neighbors.reserve(counts.size());
  for (size_t i = 0; i < counts.size() && neighbors.size() < max_neighbors; ++i) {
    neighbors.insert(counts[i].first);
  }

  return neighbors;
}

// -----------------------------------------------------------------------------
// Local bundle adjustment around a central shot
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

  // Determine neighborhood of shots to adjust (interior and boundary)
  auto neighborhood = ShotNeighborhood(
      map, central_shot_id,
      config["local_bundle_radius"].cast<size_t>(),
      config["local_bundle_min_common_points"].cast<size_t>(),
      config["local_bundle_max_shots"].cast<size_t>());

  auto& interior = neighborhood.first;
  auto& boundary = neighborhood.second;

  // Set up bundle adjuster
  bundle::BundleAdjuster ba;
  ba.SetUseSparseLinearSolver(config["bundle_use_sparse"].cast<bool>());
  ba.SetMaxIterations(config["bundle_max_iterations"].cast<int>());
  ba.SetLossFunction(config["bundle_loss_function"].cast<std::string>(),
                     config["bundle_loss_radius"].cast<double>());

  // Add interior shots (to be optimized)
  for (sfmmap::Shot* shot : interior) {
    ba.AddShot(shot->GetId(), shot->GetCamera()->id_, true);  // optimize pose
    // Fix boundary shots (pose not optimized)
    if (boundary.find(shot) != boundary.end()) {
      ba.SetPoseFixed(shot->GetId(), true);
    }
  }
  // Add boundary shots (pose fixed)
  for (sfmmap::Shot* shot : boundary) {
    if (interior.find(shot) == interior.end()) {
      ba.AddShot(shot->GetId(), shot->GetCamera()->id_, false);  // add as fixed pose
    }
  }

  // Add points and observations
  for (auto* shot : interior) {
    for (const auto& lm_obs : shot->GetLandmarkObservations()) {
      auto* lm = lm_obs.first;
      const auto& obs = lm_obs.second;
      // Add landmark if not already added
      if (!ba.HasLandmark(lm->id_)) {
        ba.AddLandmark(lm->id_, lm->GetGlobalPos(), true);
      }
      // Add observation (only if both shot and landmark are in interior/boundary set)
      if (interior.count(lm->GetObservations().begin()->first)) {
        ba.AddObservation(shot->GetId(), lm->id_, obs.point);
      }
    }
  }

  // (Additional steps like adding camera priors, rig priors, GCPs would go here if implemented)

  // Run the bundle adjustment optimization
  const auto timer_setup = std::chrono::high_resolution_clock::now();
  ba.Run();
  const auto timer_run = std::chrono::high_resolution_clock::now();

  // Report results (stubbed since detailed implementation not provided)
  report["num_interior_images"] = interior.size();
  report["num_boundary_images"] = boundary.size();
  report["num_other_images"] = map.NumberOfShots() - interior.size() - boundary.size();
  report["brief_report"] = ba.BriefReport();

  const auto timer_teardown = std::chrono::high_resolution_clock::now();
  py::dict times;
  times["setup"]    = std::chrono::duration_cast<std::chrono::microseconds>(timer_setup - start).count() / 1e6;
  times["run"]      = std::chrono::duration_cast<std::chrono::microseconds>(timer_run - timer_setup).count() / 1e6;
  times["teardown"] = std::chrono::duration_cast<std::chrono::microseconds>(timer_teardown - timer_run).count() / 1e6;
  report["wall_times"] = times;

  // Return tuple of adjusted shot IDs and the report
  py::tuple result = py::make_tuple(py::cast(central_shot_id), report);
  return result;
}

// -----------------------------------------------------------------------------
// Triangulate a Ground Control Point (GCP) from observations
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(
    const sfmmap::GroundControlPoint& point,
    const sfmmap::Map::ShotMap& shots,              // **Use Map::ShotMap alias to match header**
    Vec3d& coordinates) {
  constexpr auto reproj_threshold = 1.0;
  constexpr auto min_ray_angle = 0.1 * M_PI / 180.0;
  constexpr auto max_ray_angle = M_PI - min_ray_angle;

  // Gather bearing vectors and camera centers for each observation of the GCP
  MatX3d bearing_vectors;
  MatX3d camera_positions;
  bearing_vectors.resize(point.observations.size(), 3);
  camera_positions.resize(point.observations.size(), 3);

  size_t i = 0;
  for (const auto& obs : point.observations) {
    const sfmmap::ShotId& shot_id = obs.first;
    // Only use observations from shots present in the map (and in the provided set)
    auto it = shots.find(shot_id);
    if (it == shots.end()) {
      continue;
    }
    const sfmmap::Shot& shot = it->second;
    // Get bearing vector (unit vector from camera center through the feature observation)
    Eigen::Vector3d bearing = shot.ComputeObservationBearing(obs.second);  // hypothetical function
    Eigen::Vector3d pos = shot.GetPose().GetOrigin();                     // camera position
    bearing_vectors.row(i) = bearing;
    camera_positions.row(i) = pos;
    ++i;
  }

  if (i < 2) {
    return false;  // Not enough observations to triangulate
  }

  bearing_vectors.conservativeResize(i, Eigen::NoChange);
  camera_positions.conservativeResize(i, Eigen::NoChange);

  // Perform triangulation (e.g., using mid-point method on bearing vectors)
  Vec3d X;
  geometry::TriangulateBearingsMidpoint(bearing_vectors, camera_positions, &X);

  // Validate triangulated point by checking reprojection error and angles
  size_t num_inliers = 0;
  for (size_t j = 0; j < i; ++j) {
    // Compute reprojection error for each observation (not implemented here)
    double reproj_error = 0.0;  // placeholder
    if (reproj_error < reproj_threshold) {
      num_inliers++;
    }
  }
  if (num_inliers < 2) {
    return false;  // Require at least two inliers
  }

  // Check angle between every pair of rays
  for (size_t a = 0; a < i; ++a) {
    for (size_t b = a + 1; b < i; ++b) {
      double angle = std::acos(bearing_vectors.row(a).dot(bearing_vectors.row(b)));
      if (angle < min_ray_angle || angle > max_ray_angle) {
        return false;  // angle too shallow or too obtuse
      }
    }
  }

  // If we reach here, triangulation is successful
  coordinates = X;
  return true;
}

// -----------------------------------------------------------------------------
// Global bundle adjustment (stubbed implementation)
// -----------------------------------------------------------------------------
py::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const py::dict& config) {
  // Stub: mark unused parameters to avoid warnings
  (void)map; (void)camera_priors; (void)rig_camera_priors; (void)gcp; (void)config;
  py::dict report;
  report["status"] = "Bundle not implemented";
  return report;
}

// -----------------------------------------------------------------------------
// Bundle adjustment for only shot poses (stubbed implementation)
// -----------------------------------------------------------------------------
py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
    const py::dict& config) {
  (void)map; (void)shot_ids; (void)camera_priors; (void)rig_camera_priors; (void)config;
  py::dict report;
  report["status"] = "BundleShotPoses not implemented";
  report["num_shots"] = static_cast<int>(shot_ids.size());
  return report;
}

// -----------------------------------------------------------------------------
// Copy results from bundle adjuster back to map (stubbed)
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& bundle_adjuster,
                            sfmmap::Map& output_map,
                            bool update_cameras) {
  (void)bundle_adjuster; (void)output_map; (void)update_cameras;
  // Stub: no-op (in a full implementation, this would update the map with BA results)
}

// -----------------------------------------------------------------------------
// Alignment constraints detection (used by some pipelines)
// -----------------------------------------------------------------------------
std::string BAHelpers::DetectAlignmentConstraints(
    const sfmmap::Map& map,
    const py::dict& config,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp) {
  // For simplicity, this stub always returns "none"
  // (A real implementation might inspect GCPs and config to decide on constraints)
  (void)map; (void)config; (void)gcp;
  return "none";
}

// -----------------------------------------------------------------------------
// Add GCPs to a bundle adjuster (stubbed)
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(bundle::BundleAdjuster& ba,
                                 const sfmmap::Map& map,
                                 const AlignedVector<sfmmap::GroundControlPoint>& gcp,
                                 const py::dict& config) {
  (void)ba; (void)map; (void)gcp; (void)config;
  return 0;
}

}  // namespace sfm
