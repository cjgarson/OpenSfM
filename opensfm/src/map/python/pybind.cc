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

    // Forward declarations
    py::class_<sfmmap::Map::Shot> shotCls(m, "Shot");
    py::class_<sfmmap::Map::Map>  mapCls(m, "Map");

    DeclareShotMeasurement<int>(m, "Int");
    DeclareShotMeasurement<double>(m, "Double");
    DeclareShotMeasurement<Vec3d>(m, "Vec3d");
    DeclareShotMeasurement<std::string>(m, "String");

    py::enum_<sfmmap::Map::Map::ErrorType>(m, "ErrorType")
        .value("Pixel", sfmmap::Map::Map::Pixel)
        .value("Normalized", sfmmap::Map::Map::Normalized)
        .value("Angular", sfmmap::Map::Map::Angular)
        .export_values();

    py::class_<sfmmap::Map::Observation>(m, "Observation")
        .def(py::init<double,double,double,int,int,int,int,int,int>(),
             py::arg("x"), py::arg("y"), py::arg("s"),
             py::arg("r"), py::arg("g"), py::arg("b"),
             py::arg("feature"),
             py::arg("segmentation") = sfmmap::Map::Observation::NO_SEMANTIC_VALUE,
             py::arg("instance")     = sfmmap::Map::Observation::NO_SEMANTIC_VALUE)
        .def_readwrite("point",        &sfmmap::Map::Observation::point)
        .def_readwrite("scale",        &sfmmap::Map::Observation::scale)
        .def_readwrite("id",           &sfmmap::Map::Observation::feature_id)
        .def_readwrite("color",        &sfmmap::Map::Observation::color)
        .def_readwrite("segmentation", &sfmmap::Map::Observation::segmentation_id)
        .def_readwrite("instance",     &sfmmap::Map::Observation::instance_id)
        .def_readonly_static("NO_SEMANTIC_VALUE",
                             &sfmmap::Map::Observation::NO_SEMANTIC_VALUE)
        .def("copy", [](const sfmmap::Map::Observation &to_copy) {
            return sfmmap::Map::Observation(to_copy);
        });

    py::class_<sfmmap::Map::Landmark>(m, "Landmark")
        .def(py::init<const sfmmap::Map::LandmarkId&, const Vec3d&>())
        .def_readonly("id", &sfmmap::Map::Landmark::id_)
        .def_property("coordinates",
                      &sfmmap::Map::Landmark::GetGlobalPos,
                      &sfmmap::Map::Landmark::SetGlobalPos)
        .def("get_observations", &sfmmap::Map::Landmark::GetObservations,
             py::return_value_policy::reference_internal)
        .def("number_of_observations",
             &sfmmap::Map::Landmark::NumberOfObservations)
        .def_property("reprojection_errors",
                      &sfmmap::Map::Landmark::GetReprojectionErrors,
                      &sfmmap::Map::Landmark::SetReprojectionErrors)
        .def_property("color",
                      &sfmmap::Map::Landmark::GetColor,
                      &sfmmap::Map::Landmark::SetColor);

    py::class_<sfmmap::Map::ShotMeasurements>(m, "ShotMeasurements")
        .def(py::init<>())
        .def_readwrite("gps_accuracy",     &sfmmap::Map::ShotMeasurements::gps_accuracy_)
        .def_readwrite("gps_position",     &sfmmap::Map::ShotMeasurements::gps_position_)
        .def_readwrite("orientation",      &sfmmap::Map::ShotMeasurements::orientation_)
        .def_readwrite("capture_time",     &sfmmap::Map::ShotMeasurements::capture_time_)
        .def_readwrite("gravity_down",     &sfmmap::Map::ShotMeasurements::gravity_down_)
        .def_readwrite("compass_angle",    &sfmmap::Map::ShotMeasurements::compass_angle_)
        .def_readwrite("compass_accuracy", &sfmmap::Map::ShotMeasurements::compass_accuracy_)
        .def_readwrite("opk_angles",       &sfmmap::Map::ShotMeasurements::opk_angles_)
        .def_readwrite("opk_accuracy",     &sfmmap::Map::ShotMeasurements::opk_accuracy_)
        .def_readwrite("sequence_key",     &sfmmap::Map::ShotMeasurements::sequence_key_)
        .def_property("attributes",
                      &sfmmap::Map::ShotMeasurements::GetAttributes,
                      &sfmmap::Map::ShotMeasurements::SetAttributes)
        .def("set", &sfmmap::Map::ShotMeasurements::Set);

    py::class_<sfmmap::Map::RigCamera>(m, "RigCamera")
        .def(py::init([](const ::geometry::Pose &pose,
                         sfmmap::Map::RigCameraId id) {
            sfmmap::Map::RigCamera rc;
            rc.pose = pose;
            rc.id   = std::move(id);
            return rc;
        }))
        .def_readwrite("id",   &sfmmap::Map::RigCamera::id)
        .def_readwrite("pose", &sfmmap::Map::RigCamera::pose);

    py::class_<sfmmap::Map::RigInstance>(m, "RigInstance")
        .def(py::init<sfmmap::Map::RigInstanceId>())
        .def_property_readonly("id", &sfmmap::Map::RigInstance::GetId)
        .def_property("pose",
                      py::overload_cast<>(&sfmmap::Map::RigInstance::GetPose),
                      &sfmmap::Map::RigInstance::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("shots",
                      py::overload_cast<>(
                          &sfmmap::Map::RigInstance::GetShots, py::const_),
                      py::return_value_policy::reference_internal)
        .def("add_shot", &sfmmap::Map::RigInstance::AddShot)
        .def("remove_shot", &sfmmap::Map::RigInstance::RemoveShot);

    shotCls
        .def(py::init<const sfmmap::Map::ShotId&, const ::geometry::Camera&, const ::geometry::Pose&>())
        .def_readonly("id", &sfmmap::Map::Shot::id_)
        .def_property("pose", py::overload_cast<>(&sfmmap::Map::Shot::GetPose),
                      &sfmmap::Map::Shot::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("camera", &sfmmap::Map::Shot::GetCamera,
                               py::return_value_policy::reference_internal);

    mapCls.def(py::init<>());
}
