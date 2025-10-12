#include <foundation/optional.h>
#include <foundation/types.h>
#include <geometry/camera.h>
#include <geometry/pose.h>

#include <map/dataviews.h>
#include <map/defines.h>
#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/pybind_utils.h>
#include <map/rig.h>
#include <map/shot.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <typeinfo>

namespace py = pybind11;

// -----------------------------------------------------------------------------
// OptionalValue<T> wrapper
// -----------------------------------------------------------------------------
template <typename T>
void DeclareShotMeasurement(py::module &m, const std::string &type_name) {
  using SM = foundation::OptionalValue<T>;
  const std::string class_name = std::string("ShotMeasurement") + type_name;

  py::class_<SM>(m, class_name.c_str())
      .def(py::init<>())
      .def_property_readonly("has_value", &SM::HasValue)
      .def_property("value",
                    py::overload_cast<>(&SM::Value, py::const_),
                    &SM::SetValue)
      .def("reset", &SM::Reset)
      .def(py::pickle(
          [](const SM &sm) { return py::make_tuple(sm.HasValue(), sm.Value()); },
          [](py::tuple p) {
            SM sm;
            const bool has_value = p[0].cast<bool>();
            if (has_value) sm.SetValue(p[1].cast<T>());
            return sm;
          }));
}

PYBIND11_MODULE(pymap, m) {
  // Ensure geometry modules are loaded first (some bindings depend on them)
  py::module::import("opensfm.pygeometry");
  py::module::import("opensfm.pygeo");

  // Forward declarations to break cycles
  py::class_<sfmmap::Shot> shotCls(m, "Shot");
  py::class_<sfmmap::Map>  mapCls(m, "Map");

  // Optional<T> for ShotMeasurements
  DeclareShotMeasurement<int>(m, "Int");
  DeclareShotMeasurement<double>(m, "Double");
  DeclareShotMeasurement<Vec3d>(m, "Vec3d");
  DeclareShotMeasurement<std::string>(m, "String");

  // ErrorType enum
  py::enum_<sfmmap::Map::ErrorType>(m, "ErrorType")
      .value("Pixel",      sfmmap::Map::Pixel)
      .value("Normalized", sfmmap::Map::Normalized)
      .value("Angular",    sfmmap::Map::Angular)
      .export_values();

  // -----------------------------------------------------------------------------
  // Observation
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::Observation>(m, "Observation")
      .def(py::init<double,double,double,int,int,int,int,int,int>(),
           py::arg("x"), py::arg("y"), py::arg("s"),
           py::arg("r"), py::arg("g"), py::arg("b"),
           py::arg("feature"),
           py::arg("segmentation") = sfmmap::Observation::NO_SEMANTIC_VALUE,
           py::arg("instance")     = sfmmap::Observation::NO_SEMANTIC_VALUE)
      .def_readwrite("point",        &sfmmap::Observation::point)
      .def_readwrite("scale",        &sfmmap::Observation::scale)
      .def_readwrite("id",           &sfmmap::Observation::feature_id)
      .def_readwrite("color",        &sfmmap::Observation::color)
      .def_readwrite("segmentation", &sfmmap::Observation::segmentation_id)
      .def_readwrite("instance",     &sfmmap::Observation::instance_id)
      .def_readonly_static("NO_SEMANTIC_VALUE", &sfmmap::Observation::NO_SEMANTIC_VALUE)
      .def("copy", [](const sfmmap::Observation &o) { return sfmmap::Observation(o); });

  // -----------------------------------------------------------------------------
  // Landmark
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::Landmark>(m, "Landmark")
      .def(py::init<const sfmmap::LandmarkId&, const Vec3d&>())
      .def_readonly("id", &sfmmap::Landmark::id_)
      .def_property("coordinates",
                    &sfmmap::Landmark::GetGlobalPos,
                    &sfmmap::Landmark::SetGlobalPos)
      .def("get_observations", &sfmmap::Landmark::GetObservations,
           py::return_value_policy::reference_internal)
      .def("number_of_observations", &sfmmap::Landmark::NumberOfObservations)
      .def_property("reprojection_errors",
                    &sfmmap::Landmark::GetReprojectionErrors,
                    &sfmmap::Landmark::SetReprojectionErrors)
      .def_property("color",
                    &sfmmap::Landmark::GetColor,
                    &sfmmap::Landmark::SetColor);

  // -----------------------------------------------------------------------------
  // ShotMeasurements
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::ShotMeasurements>(m, "ShotMeasurements")
      .def(py::init<>())
      .def_readwrite("gps_accuracy",     &sfmmap::ShotMeasurements::gps_accuracy_)
      .def_readwrite("gps_position",     &sfmmap::ShotMeasurements::gps_position_)
      .def_readwrite("orientation",      &sfmmap::ShotMeasurements::orientation_)
      .def_readwrite("capture_time",     &sfmmap::ShotMeasurements::capture_time_)
      .def_readwrite("gravity_down",     &sfmmap::ShotMeasurements::gravity_down_)
      .def_readwrite("compass_angle",    &sfmmap::ShotMeasurements::compass_angle_)
      .def_readwrite("compass_accuracy", &sfmmap::ShotMeasurements::compass_accuracy_)
      .def_readwrite("opk_angles",       &sfmmap::ShotMeasurements::opk_angles_)
      .def_readwrite("opk_accuracy",     &sfmmap::ShotMeasurements::opk_accuracy_)
      .def_readwrite("sequence_key",     &sfmmap::ShotMeasurements::sequence_key_)
      .def_property("attributes",
                    &sfmmap::ShotMeasurements::GetAttributes,
                    &sfmmap::ShotMeasurements::SetAttributes)
      .def(py::pickle(
          [](const sfmmap::ShotMeasurements &s) {
            return py::make_tuple(
                s.gps_accuracy_, s.gps_position_, s.orientation_,
                s.capture_time_, s.gravity_down_, s.compass_angle_,
                s.compass_accuracy_, s.opk_angles_, s.opk_accuracy_,
                s.sequence_key_, s.GetAttributes());
          },
          [](py::tuple t) {
            sfmmap::ShotMeasurements sm;
            sm.gps_accuracy_     = t[0].cast<decltype(sm.gps_accuracy_)>();
            sm.gps_position_     = t[1].cast<decltype(sm.gps_position_)>();
            sm.orientation_      = t[2].cast<decltype(sm.orientation_)>();
            sm.capture_time_     = t[3].cast<decltype(sm.capture_time_)>();
            sm.gravity_down_     = t[4].cast<decltype(sm.gravity_down_)>();
            sm.compass_angle_    = t[5].cast<decltype(sm.compass_angle_)>();
            sm.compass_accuracy_ = t[6].cast<decltype(sm.compass_accuracy_)>();
            sm.opk_angles_       = t[7].cast<decltype(sm.opk_angles_)>();
            sm.opk_accuracy_     = t[8].cast<decltype(sm.opk_accuracy_)>();
            sm.sequence_key_     = t[9].cast<decltype(sm.sequence_key_)>();
            sm.GetMutableAttributes() = t[10].cast<decltype(sm.attributes_)>();
            return sm;
          }))
      .def("__copy__",
           [](const sfmmap::ShotMeasurements &src) {
             sfmmap::ShotMeasurements copy; copy.Set(src); return copy;
           },
           py::return_value_policy::copy)
      .def("set", &sfmmap::ShotMeasurements::Set);

  // -----------------------------------------------------------------------------
  // ShotMesh
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::ShotMesh>(m, "ShotMesh")
      .def_property("faces",    &sfmmap::ShotMesh::GetFaces,    &sfmmap::ShotMesh::SetFaces)
      .def_property("vertices", &sfmmap::ShotMesh::GetVertices, &sfmmap::ShotMesh::SetVertices);

  // -----------------------------------------------------------------------------
  // RigCamera (POD + pickle)  *** no (Pose, Id) ctor in this fork ***
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::RigCamera>(m, "RigCamera")
      .def(py::init<>())
      .def_readwrite("id",   &sfmmap::RigCamera::id)
      .def_readwrite("pose", &sfmmap::RigCamera::pose)
      .def(py::pickle(
          [](const sfmmap::RigCamera &rc) {         // __getstate__
            return py::make_tuple(rc.pose, rc.id);
          },
          [](py::tuple s) {                          // __setstate__
            if (s.size() != 2) {
              throw std::runtime_error("Invalid state for RigCamera: expected (pose, id)");
            }
            sfmmap::RigCamera rc;
            rc.pose = s[0].cast<geometry::Pose>();
            rc.id   = s[1].cast<sfmmap::RigCameraId>();
            return rc;
          }));

  // -----------------------------------------------------------------------------
  // RigInstance
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::RigInstance>(m, "RigInstance")
      .def(py::init<sfmmap::RigInstanceId>())
      .def_property_readonly("id", &sfmmap::RigInstance::GetId)
      .def_property("pose",
        [](const sfmmap::RigInstance& ri) -> const geometry::Pose& { return ri.GetPose(); },
        [](sfmmap::RigInstance& ri, const geometry::Pose& p) { ri.SetPose(p); },
        py::return_value_policy::reference_internal)
      .def_property_readonly("shots",
        [](const sfmmap::RigInstance& ri)
          -> const std::unordered_map<sfmmap::ShotId, sfmmap::Shot*>& { return ri.GetShots(); },
        py::return_value_policy::reference_internal)
      .def_property_readonly("rig_cameras",
        [](const sfmmap::RigInstance& ri)
          -> const std::unordered_map<sfmmap::ShotId, sfmmap::RigCamera*>& { return ri.GetRigCameras(); },
        py::return_value_policy::reference_internal)
      .def_property_readonly("rig_camera_ids",
        [](const sfmmap::RigInstance &ri) {
          std::map<sfmmap::ShotId, sfmmap::RigCameraId> ids;
          for (const auto &rc : ri.GetRigCameras()) ids[rc.first] = rc.second->id;
          return ids;
        })
      .def_property_readonly("camera_ids",
        [](const sfmmap::RigInstance &ri) {
          std::map<sfmmap::ShotId, sfmmap::CameraId> ids;
          for (const auto &sh : ri.GetShots()) ids[sh.first] = sh.second->GetCamera()->id;
          return ids;
        })
      .def("keys", &sfmmap::RigInstance::GetShotIDs)
      .def("add_shot",    &sfmmap::RigInstance::AddShot)
      .def("remove_shot", &sfmmap::RigInstance::RemoveShot)
      .def("update_instance_pose_with_shot", &sfmmap::RigInstance::UpdateInstancePoseWithShot)
      .def("update_rig_camera_pose",        &sfmmap::RigInstance::UpdateRigCameraPose);

  // -----------------------------------------------------------------------------
  // Shot
  // -----------------------------------------------------------------------------
  shotCls
      .def(py::init<const sfmmap::ShotId&, const geometry::Camera&, const geometry::Pose&>())
      .def_readonly("id", &sfmmap::Shot::id_)
      .def_readwrite("mesh", &sfmmap::Shot::mesh)
      .def_property("covariance", &sfmmap::Shot::GetCovariance, &sfmmap::Shot::SetCovariance)
      .def_readwrite("merge_cc", &sfmmap::Shot::merge_cc)
      .def_readwrite("scale",    &sfmmap::Shot::scale)
      .def_property_readonly("rig_instance",    &sfmmap::Shot::GetRigInstance)
      .def_property_readonly("rig_camera",      &sfmmap::Shot::GetRigCamera)
      .def_property_readonly("rig_instance_id", &sfmmap::Shot::GetRigInstanceId)
      .def_property_readonly("rig_camera_id",   &sfmmap::Shot::GetRigCameraId)
      .def("set_rig",             &sfmmap::Shot::SetRig)
      .def("get_observation",     &sfmmap::Shot::GetObservation,
                                   py::return_value_policy::reference_internal)
      .def("get_valid_landmarks", &sfmmap::Shot::ComputeValidLandmarks)
      .def("remove_observation",  &sfmmap::Shot::RemoveLandmarkObservation)
      .def_property("metadata",
                    py::overload_cast<>(&sfmmap::Shot::GetShotMeasurements),
                    &sfmmap::Shot::SetShotMeasurements,
                    py::return_value_policy::reference_internal)
      .def_property("pose",
                    py::overload_cast<>(&sfmmap::Shot::GetPose),
                    &sfmmap::Shot::SetPose,
                    py::return_value_policy::reference_internal)
      .def_property_readonly("camera", &sfmmap::Shot::GetCamera,
                             py::return_value_policy::reference_internal)
      .def("get_landmark_observation", &sfmmap::Shot::GetLandmarkObservation,
           py::return_value_policy::reference_internal)
      .def("get_observation_landmark", &sfmmap::Shot::GetObservationLandmark,
           py::return_value_policy::reference_internal)
      .def("project",      &sfmmap::Shot::Project)
      .def("project_many", &sfmmap::Shot::ProjectMany)
      .def("bearing",      &sfmmap::Shot::Bearing)
      .def("bearing_many", &sfmmap::Shot::BearingMany);

  // -----------------------------------------------------------------------------
  // GroundControlPointObservation
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::GroundControlPointObservation>(m, "GroundControlPointObservation")
      .def(py::init())
      .def(py::init<const sfmmap::ShotId&, const Vec2d&>())
      .def_readwrite("shot_id",    &sfmmap::GroundControlPointObservation::shot_id_)
      .def_readwrite("uid",        &sfmmap::GroundControlPointObservation::uid_)
      .def_readwrite("projection", &sfmmap::GroundControlPointObservation::projection_);

  // -----------------------------------------------------------------------------
  // GroundControlPoint
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::GroundControlPoint>(m, "GroundControlPoint")
      .def(py::init())
      .def_readwrite("id",             &sfmmap::GroundControlPoint::id_)
      .def_readwrite("survey_point_id",&sfmmap::GroundControlPoint::survey_point_id_)
      .def_readwrite("has_altitude",   &sfmmap::GroundControlPoint::has_altitude_)
      .def_readwrite("lla",            &sfmmap::GroundControlPoint::lla_)
      .def_property("lla_vec", &sfmmap::GroundControlPoint::GetLlaVec3d,
                               &sfmmap::GroundControlPoint::SetLla)
      .def_property("observations",
                    &sfmmap::GroundControlPoint::GetObservations,
                    &sfmmap::GroundControlPoint::SetObservations)
      .def("add_observation", &sfmmap::GroundControlPoint::AddObservation);

  // -----------------------------------------------------------------------------
  // TracksManager
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::TracksManager>(m, "TracksManager")
      .def(py::init())
      .def_static("instanciate_from_file",   &sfmmap::TracksManager::InstanciateFromFile,
                  py::call_guard<py::gil_scoped_release>())
      .def_static("instanciate_from_string", &sfmmap::TracksManager::InstanciateFromString,
                  py::call_guard<py::gil_scoped_release>())
      .def_static("merge_tracks_manager",    &sfmmap::TracksManager::MergeTracksManager)
      .def("add_observation",        &sfmmap::TracksManager::AddObservation)
      .def("remove_observation",     &sfmmap::TracksManager::RemoveObservation)
      .def("num_shots",              &sfmmap::TracksManager::NumShots)
      .def("num_tracks",             &sfmmap::TracksManager::NumTracks)
      .def("get_shot_ids",           &sfmmap::TracksManager::GetShotIds)
      .def("get_track_ids",          &sfmmap::TracksManager::GetTrackIds)
      .def("get_observation",        &sfmmap::TracksManager::GetObservation)
      .def("get_shot_observations",  &sfmmap::TracksManager::GetShotObservations)
      .def("get_track_observations", &sfmmap::TracksManager::GetTrackObservations)
      .def("construct_sub_tracks_manager",
           &sfmmap::TracksManager::ConstructSubTracksManager)
      .def("write_to_file",          &sfmmap::TracksManager::WriteToFile)
      .def("as_string",              &sfmmap::TracksManager::AsString)
      .def("get_all_common_observations",
           &sfmmap::TracksManager::GetAllCommonObservations,
           py::call_guard<py::gil_scoped_release>())
      .def("get_all_pairs_connectivity",
           &sfmmap::TracksManager::GetAllPairsConnectivity,
           py::arg("shots")  = std::vector<sfmmap::ShotId>(),
           py::arg("tracks") = std::vector<sfmmap::TrackId>(),
           py::call_guard<py::gil_scoped_release>());

  // -----------------------------------------------------------------------------
  // PanoShotView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::PanoShotView>(m, "PanoShotView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::PanoShotView::NumberOfShots)
      .def("items",
           [](sfmmap::PanoShotView &sv) {
             py::list out;
             for (auto &kv : sv.GetShots()) out.append(py::make_tuple(kv.first, kv.second));
             return out;
           })
      .def("values",
           [](sfmmap::PanoShotView &sv) {
             py::list out;
             for (auto &kv : sv.GetShots()) out.append(kv.second);
             return out;
           })
      .def("__iter__",
           [](const sfmmap::PanoShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::PanoShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::PanoShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::PanoShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::PanoShotView::HasShot);

  // -----------------------------------------------------------------------------
  // ShotView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::ShotView>(m, "ShotView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::ShotView::NumberOfShots)
      .def("items",
           [](sfmmap::ShotView &sv) {
             py::list out;
             for (auto &kv : sv.GetShots()) out.append(py::make_tuple(kv.first, kv.second));
             return out;
           })
      .def("values",
           [](sfmmap::ShotView &sv) {
             py::list out;
             for (auto &kv : sv.GetShots()) out.append(kv.second);
             return out;
           })
      .def("__iter__",
           [](const sfmmap::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::ShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::ShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::ShotView::HasShot);

  // -----------------------------------------------------------------------------
  // LandmarkView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::LandmarkView>(m, "LandmarkView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::LandmarkView::NumberOfLandmarks)
      .def("items",
           [](sfmmap::LandmarkView &sv) {
             py::list out;
             for (auto &kv : sv.GetLandmarks()) out.append(py::make_tuple(kv.first, kv.second));
             return out;
           })
      .def("values",
           [](sfmmap::LandmarkView &sv) {
             py::list out;
             for (auto &kv : sv.GetLandmarks()) out.append(kv.second);
             return out;
           })
      .def("__iter__",
           [](const sfmmap::LandmarkView &sv) {
             const auto &lms = sv.GetLandmarks();
             return py::make_key_iterator(lms.begin(), lms.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::LandmarkView &sv) {
             const auto &lms = sv.GetLandmarks();
             return py::make_key_iterator(lms.begin(), lms.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::LandmarkView::GetLandmark,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::LandmarkView::GetLandmark,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::LandmarkView::HasLandmark);

  // -----------------------------------------------------------------------------
  // CameraView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::CameraView>(m, "CameraView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::CameraView::NumberOfCameras)
      .def("items",
           [](sfmmap::CameraView &sv) {
             py::list out;
             for (const auto &kv : sv.GetCameras())
               out.append(py::make_tuple(kv.first, &sv.GetCamera(kv.first)));
             return out;
           })
      .def("values",
           [](sfmmap::CameraView &sv) {
             py::list out;
             for (const auto &kv : sv.GetCameras())
               out.append(&sv.GetCamera(kv.first));
             return out;
           }, py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const sfmmap::CameraView &sv) {
             const auto &cams = sv.GetCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::CameraView &sv) {
             const auto &cams = sv.GetCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::CameraView::GetCamera,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::CameraView::GetCamera,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::CameraView::HasCamera);

  // -----------------------------------------------------------------------------
  // BiasView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::BiasView>(m, "BiasView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::BiasView::NumberOfBiases)
      .def("items",
           [](sfmmap::BiasView &sv) {
             py::list out;
             for (const auto &kv : sv.GetBiases())
               out.append(py::make_tuple(kv.first, &sv.GetBias(kv.first)));
             return out;
           })
      .def("values",
           [](sfmmap::BiasView &sv) {
             py::list out;
             for (const auto &kv : sv.GetBiases())
               out.append(&sv.GetBias(kv.first));
             return out;
           }, py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const sfmmap::BiasView &sv) {
             const auto &biases = sv.GetBiases();
             return py::make_key_iterator(biases.begin(), biases.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::BiasView &sv) {
             const auto &biases = sv.GetBiases();
             return py::make_key_iterator(biases.begin(), biases.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::BiasView::GetBias,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::BiasView::GetBias,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::BiasView::HasBias);

  // -----------------------------------------------------------------------------
  // RigCameraView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::RigCameraView>(m, "RigCameraView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::RigCameraView::NumberOfRigCameras)
      .def("items",
           [](sfmmap::RigCameraView &sv) {
             py::list out;
             for (const auto &kv : sv.GetRigCameras())
               out.append(py::make_tuple(kv.first, &sv.GetRigCamera(kv.first)));
             return out;
           })
      .def("values",
           [](sfmmap::RigCameraView &sv) {
             py::list out;
             for (const auto &kv : sv.GetRigCameras())
               out.append(&sv.GetRigCamera(kv.first));
             return out;
           }, py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const sfmmap::RigCameraView &sv) {
             const auto &cams = sv.GetRigCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::RigCameraView &sv) {
             const auto &cams = sv.GetRigCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::RigCameraView::GetRigCamera,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::RigCameraView::GetRigCamera,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::RigCameraView::HasRigCamera);

  // -----------------------------------------------------------------------------
  // RigInstanceView
  // -----------------------------------------------------------------------------
  py::class_<sfmmap::RigInstanceView>(m, "RigInstanceView")
      .def(py::init<sfmmap::Map&>(), py::keep_alive<1, 2>())
      .def("__len__", &sfmmap::RigInstanceView::NumberOfRigInstances)
      .def("items",
           [](sfmmap::RigInstanceView &sv) {
             py::list out;
             for (const auto &kv : sv.GetRigInstances())
               out.append(py::make_tuple(kv.first, &sv.GetRigInstance(kv.first)));
             return out;
           })
      .def("values",
           [](sfmmap::RigInstanceView &sv) {
             py::list out;
             for (const auto &kv : sv.GetRigInstances())
               out.append(&sv.GetRigInstance(kv.first));
             return out;
           }, py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const sfmmap::RigInstanceView &sv) {
             const auto &instances = sv.GetRigInstances();
             return py::make_key_iterator(instances.begin(), instances.end());
           },
           py::keep_alive<0, 1>())
      .def("keys",
           [](const sfmmap::RigInstanceView &sv) {
             const auto &instances = sv.GetRigInstances();
             return py::make_key_iterator(instances.begin(), instances.end());
           },
           py::keep_alive<0, 1>())
      .def("get", &sfmmap::RigInstanceView::GetRigInstance,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &sfmmap::RigInstanceView::GetRigInstance,
           py::return_value_policy::reference_internal)
      .def("__contains__", &sfmmap::RigInstanceView::HasRigInstance);

  // -----------------------------------------------------------------------------
  // Map (full surface expected by Python)
  // -----------------------------------------------------------------------------
  mapCls
      .def(py::init())
      .def_static("deep_copy", &sfmmap::Map::DeepCopy,
                  py::return_value_policy::reference_internal,
                  py::call_guard<py::gil_scoped_release>())
      // Camera
      .def("create_camera", &sfmmap::Map::CreateCamera, py::arg("camera"),
           py::return_value_policy::reference_internal)
      .def("get_camera",
           [](sfmmap::Map &m, const sfmmap::CameraId &id) -> geometry::Camera& {
             return m.GetCamera(id);
           }, py::return_value_policy::reference_internal)
      // Bias
      .def("set_bias", &sfmmap::Map::SetBias,
           py::return_value_policy::reference_internal)
      .def("get_bias", &sfmmap::Map::GetBias,
           py::return_value_policy::reference_internal)
      // Rigs
      .def("create_rig_camera",  &sfmmap::Map::CreateRigCamera,
           py::return_value_policy::reference_internal)
      .def("create_rig_instance", &sfmmap::Map::CreateRigInstance,
           py::return_value_policy::reference_internal)
      .def("update_rig_instance", &sfmmap::Map::UpdateRigInstance,
           py::return_value_policy::reference_internal)
      .def("remove_rig_instance", &sfmmap::Map::RemoveRigInstance)
      // Landmark
      .def("create_landmark", &sfmmap::Map::CreateLandmark,
           py::arg("lm_id"), py::arg("global_position"),
           py::return_value_policy::reference_internal)
      .def("remove_landmark",
           (void (sfmmap::Map::*)(const sfmmap::Landmark* const)) &sfmmap::Map::RemoveLandmark)
      .def("remove_landmark",
           (void (sfmmap::Map::*)(const sfmmap::LandmarkId&))     &sfmmap::Map::RemoveLandmark)
      .def("has_landmark", &sfmmap::Map::HasLandmark)
      .def("get_landmark",
           [](sfmmap::Map &m, const sfmmap::LandmarkId &id) -> sfmmap::Landmark& {
             return m.GetLandmark(id);
           }, py::return_value_policy::reference_internal)
      .def("clear_observations_and_landmarks", &sfmmap::Map::ClearObservationsAndLandmarks)
      .def("clean_landmarks_below_min_observations",
           &sfmmap::Map::CleanLandmarksBelowMinObservations)
      // Shot
      .def("create_shot",
           (sfmmap::Shot& (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::CameraId&,
                                           const sfmmap::RigCameraId&, const sfmmap::RigInstanceId&,
                                           const geometry::Pose&)) &sfmmap::Map::CreateShot,
           py::return_value_policy::reference_internal)
      .def("create_shot",
           (sfmmap::Shot& (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::CameraId&,
                                           const sfmmap::RigCameraId&, const sfmmap::RigInstanceId&))
                                           &sfmmap::Map::CreateShot,
           py::return_value_policy::reference_internal)
      .def("remove_shot", &sfmmap::Map::RemoveShot)
      .def("get_shot",
           [](sfmmap::Map &m, const sfmmap::ShotId &id) -> sfmmap::Shot& {
             return m.GetShot(id);
           }, py::return_value_policy::reference_internal)
      .def("update_shot", &sfmmap::Map::UpdateShot,
           py::return_value_policy::reference_internal)
      // Pano Shot
      .def("create_pano_shot", &sfmmap::Map::CreatePanoShot,
           py::return_value_policy::reference_internal)
      .def("remove_pano_shot", &sfmmap::Map::RemovePanoShot)
      .def("get_pano_shot",
           [](sfmmap::Map &m, const sfmmap::ShotId &id) -> sfmmap::Shot& {
             return m.GetPanoShot(id);
           }, py::return_value_policy::reference_internal)
      .def("update_pano_shot", &sfmmap::Map::UpdatePanoShot,
           py::return_value_policy::reference_internal)
      // Observation
      .def("add_observation",
           (void (sfmmap::Map::*)(sfmmap::Shot* const, sfmmap::Landmark* const,
                                  const sfmmap::Observation&)) &sfmmap::Map::AddObservation,
           py::arg("shot"), py::arg("landmark"), py::arg("observation"))
      .def("add_observation",
           (void (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::LandmarkId&,
                                  const sfmmap::Observation&)) &sfmmap::Map::AddObservation,
           py::arg("shot_id"), py::arg("landmark_id"), py::arg("observation"))
      .def("remove_observation",
           (void (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::LandmarkId&)) &sfmmap::Map::RemoveObservation,
           py::arg("shot"), py::arg("landmark"))
      // Views
      .def("get_shots",         &sfmmap::Map::GetShotView)
      .def("get_pano_shots",    &sfmmap::Map::GetPanoShotView)
      .def("get_cameras",       &sfmmap::Map::GetCameraView)
      .def("get_biases",        &sfmmap::Map::GetBiasView)
      .def("get_camera_view",   &sfmmap::Map::GetCameraView)
      .def("get_landmarks",     &sfmmap::Map::GetLandmarkView)
      .def("get_landmark_view", &sfmmap::Map::GetLandmarkView)
      // Reference (Topocentric)
      .def("set_reference", &sfmmap::Map::SetTopocentricConverter)
      .def("get_reference",
           [](const sfmmap::Map &map) {
             py::module::import("opensfm.pygeo");  // ensure types are loaded
             return map.GetTopocentricConverter();
           })
      // QA / Tracks
      .def("compute_reprojection_errors", &sfmmap::Map::ComputeReprojectionErrors)
      .def("get_valid_observations",      &sfmmap::Map::GetValidObservations)
      .def("to_tracks_manager",           &sfmmap::Map::ToTracksManager);
}
