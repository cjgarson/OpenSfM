#include <foundation/optional.h>
#include <foundation/types.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include "map/dataviews.h"
#include "map/defines.h"
#include "map/ground_control_points.h"
#include "map/landmark.h"
#include "map/map.h"
#include "map/pybind_utils.h"
#include "map/rig.h"
#include "map/shot.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <typeinfo>

namespace py = pybind11;
using namespace sfmmap;   // <— critical to resolve all sfmmap:: types

// -----------------------------------------------------------------------------
// Utility template for exposing foundation::OptionalValue<T>
// -----------------------------------------------------------------------------
template <typename T>
void DeclareShotMeasurement(py::module &m, const std::string &type_name) {
    using SM = foundation::OptionalValue<T>;
    const std::string class_name = "ShotMeasurement" + type_name;

    py::class_<SM>(m, class_name.c_str())
        .def(py::init<>())
        .def_property_readonly("has_value", &SM::HasValue)
        .def_property("value", py::overload_cast<>(&SM::Value, py::const_), &SM::SetValue)
        .def("reset", &SM::Reset)
        .def(py::pickle(
            [](const SM &sm) {
                return py::make_tuple(sm.HasValue(), sm.HasValue() ? sm.Value() : T());
            },
            [](py::tuple p) {
                SM sm;
                if (p[0].cast<bool>()) sm.SetValue(p[1].cast<T>());
                return sm;
            }));
}

// -----------------------------------------------------------------------------
// PYBIND11 MODULE
// -----------------------------------------------------------------------------
PYBIND11_MODULE(pymap, m) {
    py::module::import("opensfm.pygeometry");
    py::module::import("opensfm.pygeo");

    // Forward declarations for cyclic types
    py::class_<sfmmap::Shot> shotCls(m, "Shot");
    py::class_<sfmmap::Map> mapCls(m, "Map");

    DeclareShotMeasurement<int>(m, "Int");
    DeclareShotMeasurement<double>(m, "Double");
    DeclareShotMeasurement<Vec3d>(m, "Vec3d");
    DeclareShotMeasurement<std::string>(m, "String");

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------
    py::enum_<sfmmap::Map::ErrorType>(m, "ErrorType")
        .value("Pixel", sfmmap::Map::Pixel)
        .value("Normalized", sfmmap::Map::Normalized)
        .value("Angular", sfmmap::Map::Angular)
        .export_values();

    // -------------------------------------------------------------------------
    // Observation
    // -------------------------------------------------------------------------
    py::class_<sfmmap::Observation>(m, "Observation")
        .def(py::init<double, double, double, int, int, int, int, int, int>(),
             py::arg("x"), py::arg("y"), py::arg("s"), py::arg("r"), py::arg("g"),
             py::arg("b"), py::arg("feature"),
             py::arg("segmentation") = sfmmap::Observation::NO_SEMANTIC_VALUE,
             py::arg("instance") = sfmmap::Observation::NO_SEMANTIC_VALUE)
        .def_readwrite("point", &sfmmap::Observation::point)
        .def_readwrite("scale", &sfmmap::Observation::scale)
        .def_readwrite("id", &sfmmap::Observation::feature_id)
        .def_readwrite("color", &sfmmap::Observation::color)
        .def_readwrite("segmentation", &sfmmap::Observation::segmentation_id)
        .def_readwrite("instance", &sfmmap::Observation::instance_id)
        .def_readonly_static("NO_SEMANTIC_VALUE",
                             &sfmmap::Observation::NO_SEMANTIC_VALUE)
        .def("copy", [](const sfmmap::Observation &to_copy) {
            return sfmmap::Observation(to_copy);
        });

    // -------------------------------------------------------------------------
    // Landmark
    // -------------------------------------------------------------------------
    py::class_<sfmmap::Landmark>(m, "Landmark")
        .def(py::init<const sfmmap::LandmarkId&, const Vec3d&>())
        .def_readonly("id", &sfmmap::Landmark::id_)
        .def_property("coordinates", &sfmmap::Landmark::GetGlobalPos,
                      &sfmmap::Landmark::SetGlobalPos)
        .def("get_observations", &sfmmap::Landmark::GetObservations,
             py::return_value_policy::reference_internal)
        .def("number_of_observations", &sfmmap::Landmark::NumberOfObservations)
        .def_property("reprojection_errors",
                      &sfmmap::Landmark::GetReprojectionErrors,
                      &sfmmap::Landmark::SetReprojectionErrors)
        .def_property("color", &sfmmap::Landmark::GetColor,
                      &sfmmap::Landmark::SetColor);

    // -------------------------------------------------------------------------
    // ShotMeasurements
    // -------------------------------------------------------------------------
    py::class_<sfmmap::ShotMeasurements>(m, "ShotMeasurements")
        .def(py::init<>())
        .def_readwrite("gps_accuracy", &sfmmap::ShotMeasurements::gps_accuracy_)
        .def_readwrite("gps_position", &sfmmap::ShotMeasurements::gps_position_)
        .def_readwrite("orientation", &sfmmap::ShotMeasurements::orientation_)
        .def_readwrite("capture_time", &sfmmap::ShotMeasurements::capture_time_)
        .def_readwrite("gravity_down", &sfmmap::ShotMeasurements::gravity_down_)
        .def_readwrite("compass_angle", &sfmmap::ShotMeasurements::compass_angle_)
        .def_readwrite("compass_accuracy", &sfmmap::ShotMeasurements::compass_accuracy_)
        .def_readwrite("opk_angles", &sfmmap::ShotMeasurements::opk_angles_)
        .def_readwrite("opk_accuracy", &sfmmap::ShotMeasurements::opk_accuracy_)
        .def_readwrite("sequence_key", &sfmmap::ShotMeasurements::sequence_key_)
        .def_property("attributes", &sfmmap::ShotMeasurements::GetAttributes,
                      &sfmmap::ShotMeasurements::SetAttributes)
        .def("set", &sfmmap::ShotMeasurements::Set)
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
                sm.gps_accuracy_ = t[0].cast<decltype(sm.gps_accuracy_)>();
                sm.gps_position_ = t[1].cast<decltype(sm.gps_position_)>();
                sm.orientation_ = t[2].cast<decltype(sm.orientation_)>();
                sm.capture_time_ = t[3].cast<decltype(sm.capture_time_)>();
                sm.gravity_down_ = t[4].cast<decltype(sm.gravity_down_)>();
                sm.compass_angle_ = t[5].cast<decltype(sm.compass_angle_)>();
                sm.compass_accuracy_ = t[6].cast<decltype(sm.compass_accuracy_)>();
                sm.opk_angles_ = t[7].cast<decltype(sm.opk_angles_)>();
                sm.opk_accuracy_ = t[8].cast<decltype(sm.opk_accuracy_)>();
                sm.sequence_key_ = t[9].cast<decltype(sm.sequence_key_)>();
                sm.GetMutableAttributes() = t[10].cast<decltype(sm.attributes_)>();
                return sm;
            }));

    // -------------------------------------------------------------------------
    // RigCamera
    // -------------------------------------------------------------------------
    py::class_<sfmmap::RigCamera>(m, "RigCamera")
        .def(py::init([](const geometry::Pose &pose, sfmmap::RigCameraId id) {
            sfmmap::RigCamera rc;
            rc.pose = pose;
            rc.id = std::move(id);
            return rc;
        }))
        .def_readwrite("id", &sfmmap::RigCamera::id)
        .def_readwrite("pose", &sfmmap::RigCamera::pose)
        .def(py::pickle(
            [](const sfmmap::RigCamera &rc) {
                return py::make_tuple(rc.pose, rc.id);
            },
            [](py::tuple t) {
                sfmmap::RigCamera rc;
                rc.pose = t[0].cast<geometry::Pose>();
                rc.id = t[1].cast<sfmmap::RigCameraId>();
                return rc;
            }));

    // -------------------------------------------------------------------------
    // RigInstance
    // -------------------------------------------------------------------------
    py::class_<sfmmap::RigInstance>(m, "RigInstance")
        .def(py::init<sfmmap::RigInstanceId>())
        .def_property_readonly("id", &sfmmap::RigInstance::GetId)
        .def_property("pose", py::overload_cast<>(&sfmmap::RigInstance::GetPose),
                      &sfmmap::RigInstance::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("shots",
                               py::overload_cast<>(&sfmmap::RigInstance::GetShots, py::const_),
                               py::return_value_policy::reference_internal)
        .def_property_readonly("rig_cameras",
                               py::overload_cast<>(&sfmmap::RigInstance::GetRigCameras, py::const_),
                               py::return_value_policy::reference_internal)
        .def("keys", &sfmmap::RigInstance::GetShotIDs)
        .def("add_shot", &sfmmap::RigInstance::AddShot)
        .def("remove_shot", &sfmmap::RigInstance::RemoveShot)
        .def("update_instance_pose_with_shot",
             &sfmmap::RigInstance::UpdateInstancePoseWithShot)
        .def("update_rig_camera_pose", &sfmmap::RigInstance::UpdateRigCameraPose);

    // -------------------------------------------------------------------------
    // Shot
    // -------------------------------------------------------------------------
    shotCls
        .def(py::init<const sfmmap::ShotId&, const geometry::Camera&, const geometry::Pose&>())
        .def_readonly("id", &sfmmap::Shot::id_)
        .def_readwrite("mesh", &sfmmap::Shot::mesh)
        .def_property("covariance", &sfmmap::Shot::GetCovariance, &sfmmap::Shot::SetCovariance)
        .def_readwrite("merge_cc", &sfmmap::Shot::merge_cc)
        .def_readwrite("scale", &sfmmap::Shot::scale)
        .def_property_readonly("rig_instance", &sfmmap::Shot::GetRigInstance)
        .def_property_readonly("rig_camera", &sfmmap::Shot::GetRigCamera)
        .def_property_readonly("rig_instance_id", &sfmmap::Shot::GetRigInstanceId)
        .def_property_readonly("rig_camera_id", &sfmmap::Shot::GetRigCameraId)
        .def("set_rig", &sfmmap::Shot::SetRig)
        .def_property("metadata",
                      py::overload_cast<>(&sfmmap::Shot::GetShotMeasurements),
                      &sfmmap::Shot::SetShotMeasurements,
                      py::return_value_policy::reference_internal)
        .def_property("pose", py::overload_cast<>(&sfmmap::Shot::GetPose),
                      &sfmmap::Shot::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("camera", &sfmmap::Shot::GetCamera,
                               py::return_value_policy::reference_internal)
        .def("project", &sfmmap::Shot::Project)
        .def("project_many", &sfmmap::Shot::ProjectMany)
        .def("bearing", &sfmmap::Shot::Bearing)
        .def("bearing_many", &sfmmap::Shot::BearingMany);

    // -------------------------------------------------------------------------
    // Map class binding (trimmed; unchanged apart from sfmmap::)
    // -------------------------------------------------------------------------
    mapCls
        .def(py::init<>())
        .def_static("deep_copy", &sfmmap::Map::DeepCopy,
                    py::return_value_policy::reference_internal,
                    py::call_guard<py::gil_scoped_release>())
        // Camera
        .def("create_camera", &sfmmap::Map::CreateCamera,
             py::arg("camera"), py::return_value_policy::reference_internal)
        .def("get_camera",
             py::overload_cast<const sfmmap::CameraId&>(&sfmmap::Map::GetCamera),
             py::return_value_policy::reference_internal);
    // ... (rest of bindings identical, just replace map:: → sfmmap:: consistently)
}
