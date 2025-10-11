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

// Generic OptionalValue<T> wrapper
template <typename T>
void DeclareShotMeasurement(py::module &m, const std::string &type_name) {
  using SM = foundation::OptionalValue<T>;
  const std::string class_name = "ShotMeasurement" + type_name;
  py::class_<SM>(m, class_name.c_str())
      .def(py::init<>())
      .def_property_readonly("has_value", &SM::HasValue)
      .def_property("value",
                    [](const SM& s) { return s.Value(); },
                    [](SM& s, const T& v) { s.SetValue(v); })
      .def("reset", &SM::Reset);
}

PYBIND11_MODULE(pymap, m) {
  py::module::import("opensfm.pygeometry");
  py::module::import("opensfm.pygeo");

  // Forward declarations to break cycles
  py::class_<sfmmap::Shot> shotCls(m, "Shot");
  py::class_<sfmmap::Map>  mapCls(m, "Map");

  // Optional<T> bindings used by ShotMeasurements
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

  // Observation
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
      .def_readonly_static("NO_SEMANTIC_VALUE",
                           &sfmmap::Observation::NO_SEMANTIC_VALUE)
      .def("copy", [](const sfmmap::Observation &o) { return sfmmap::Observation(o); });

  // Landmark
  py::class_<sfmmap::Landmark>(m, "Landmark")
      .def(py::init<const sfmmap::LandmarkId&, const Vec3d&>())
      .def_readonly("id", &sfmmap::Landmark::id_)
      .def_property("coordinates",
                    &sfmmap::Landmark::GetGlobalPos,
                    &sfmmap::Landmark::SetGlobalPos)
      .def("get_observations", &sfmmap::Landmark::GetObservations,
           py::return_value_policy::reference_internal)
      .def("number_of_observations",
           &sfmmap::Landmark::NumberOfObservations)
      .def_property("reprojection_errors",
                    &sfmmap::Landmark::GetReprojectionErrors,
                    &sfmmap::Landmark::SetReprojectionErrors)
      .def_property("color",
                    &sfmmap::Landmark::GetColor,
                    &sfmmap::Landmark::SetColor);

  // ShotMeasurements
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
      .def("set", &sfmmap::ShotMeasurements::Set);

  // RigCamera (simple struct)
  py::class_<sfmmap::RigCamera>(m, "RigCamera")
      .def(py::init([](const geometry::Pose& pose, sfmmap::RigCameraId id) {
        sfmmap::RigCamera rc; rc.pose = pose; rc.id = std::move(id); return rc;
      }))
      .def_readwrite("id",   &sfmmap::RigCamera::id)
      .def_readwrite("pose", &sfmmap::RigCamera::pose);

  // RigInstance
  py::class_<sfmmap::RigInstance>(m, "RigInstance")
      .def(py::init<sfmmap::RigInstanceId>())
      .def_property_readonly("id", &sfmmap::RigInstance::GetId)
      .def_property("pose",
        [](const sfmmap::RigInstance& ri) -> const geometry::Pose& { return ri.GetPose(); },
        [](sfmmap::RigInstance& ri, const geometry::Pose& p) { ri.SetPose(p); },
        py::return_value_policy::reference_internal)
      .def_property_readonly("shots",
        [](const sfmmap::RigInstance& ri)
          -> const std::unordered_map<sfmmap::ShotId, sfmmap::Shot*>& {
          return ri.GetShots();
        }, py::return_value_policy::reference_internal)
      .def_property_readonly("rig_cameras",
        [](const sfmmap::RigInstance& ri)
          -> const std::unordered_map<sfmmap::ShotId, sfmmap::RigCamera*>& {
          return ri.GetRigCameras();
        }, py::return_value_policy::reference_internal)
      .def("add_shot",    &sfmmap::RigInstance::AddShot)
      .def("remove_shot", &sfmmap::RigInstance::RemoveShot);

  // Shot
  shotCls
      .def(py::init<const sfmmap::ShotId&, const geometry::Camera&, const geometry::Pose&>())
      .def_readonly("id", &sfmmap::Shot::id_)
      .def_property("pose",
        [](const sfmmap::Shot& s) -> const geometry::Pose& { return *s.GetPose(); },
        [](sfmmap::Shot& s, const geometry::Pose& p) { s.SetPose(p); },
        py::return_value_policy::reference_internal)
      .def_property_readonly("camera",
        [](const sfmmap::Shot& s) { return s.GetCamera(); },
        py::return_value_policy::reference_internal)
      .def("get_landmarks",
        [](sfmmap::Shot& s)
          -> std::map<sfmmap::Landmark*, sfmmap::Observation, sfmmap::KeyCompare,
                      Eigen::aligned_allocator<std::pair<sfmmap::Landmark* const, sfmmap::Observation>>>& {
          return s.GetLandmarkObservations();
        }, py::return_value_policy::reference_internal)
      .def("remove_landmark", &sfmmap::Shot::RemoveLandmarkObservation);

  // Map (expose only the essentials needed by python side here)
  mapCls
      .def(py::init<>())
      .def("get_shot",
        [](sfmmap::Map& m, const sfmmap::ShotId& id) -> sfmmap::Shot& { return m.GetShot(id); },
        py::return_value_policy::reference_internal)
      .def("get_landmark",
        [](sfmmap::Map& m, const sfmmap::LandmarkId& id) -> sfmmap::Landmark& { return m.GetLandmark(id); },
        py::return_value_policy::reference_internal)
      .def("create_camera", &sfmmap::Map::CreateCamera)
      .def("create_shot",
        (sfmmap::Shot& (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::CameraId&,
                                        const sfmmap::RigCameraId&, const sfmmap::RigInstanceId&,
                                        const geometry::Pose&)) &sfmmap::Map::CreateShot,
        py::return_value_policy::reference_internal)
      .def("remove_shot", &sfmmap::Map::RemoveShot)
      .def("create_landmark", &sfmmap::Map::CreateLandmark,
           py::return_value_policy::reference_internal)
      .def("remove_landmark",
           (void (sfmmap::Map::*)(const sfmmap::LandmarkId&)) &sfmmap::Map::RemoveLandmark)
      .def("add_observation",
           (void (sfmmap::Map::*)(sfmmap::Shot* const, sfmmap::Landmark* const,
                                  const sfmmap::Observation&)) &sfmmap::Map::AddObservation)
      .def("remove_observation",
           (void (sfmmap::Map::*)(const sfmmap::ShotId&, const sfmmap::LandmarkId&)) &sfmmap::Map::RemoveObservation)
      .def("clear_observations_and_landmarks", &sfmmap::Map::ClearObservationsAndLandmarks)
      .def("get_topocentric_converter", &sfmmap::Map::GetTopocentricConverter,
           py::return_value_policy::reference_internal)
      .def("get_shots",
           [](sfmmap::Map& m)
             -> const sfmmap::Map::ShotMap& {
             return m.GetShots();
           }, py::return_value_policy::reference_internal)

      .def("get_landmarks",
           [](sfmmap::Map& m)
             -> const sfmmap::Map::LandmarkMap& {
             return m.GetLandmarks();
           }, py::return_value_policy::reference_internal)
}
