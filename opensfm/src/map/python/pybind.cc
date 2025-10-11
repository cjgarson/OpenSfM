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
    py::class_<sfmmap::map::Shot> shotCls(m, "Shot");
    py::class_<sfmmap::map::Map>  mapCls(m, "Map");

    DeclareShotMeasurement<int>(m, "Int");
    DeclareShotMeasurement<double>(m, "Double");
    DeclareShotMeasurement<Vec3d>(m, "Vec3d");
    DeclareShotMeasurement<std::string>(m, "String");

    py::enum_<sfmmap::map::Map::ErrorType>(m, "ErrorType")
        .value("Pixel", sfmmap::map::Map::Pixel)
        .value("Normalized", sfmmap::map::Map::Normalized)
        .value("Angular", sfmmap::map::Map::Angular)
        .export_values();

    py::class_<sfmmap::map::Observation>(m, "Observation")
        .def(py::init<double,double,double,int,int,int,int,int,int>(),
             py::arg("x"), py::arg("y"), py::arg("s"),
             py::arg("r"), py::arg("g"), py::arg("b"),
             py::arg("feature"),
             py::arg("segmentation") = sfmmap::map::Observation::NO_SEMANTIC_VALUE,
             py::arg("instance")     = sfmmap::map::Observation::NO_SEMANTIC_VALUE)
        .def_readwrite("point",        &sfmmap::map::Observation::point)
        .def_readwrite("scale",        &sfmmap::map::Observation::scale)
        .def_readwrite("id",           &sfmmap::map::Observation::feature_id)
        .def_readwrite("color",        &sfmmap::map::Observation::color)
        .def_readwrite("segmentation", &sfmmap::map::Observation::segmentation_id)
        .def_readwrite("instance",     &sfmmap::map::Observation::instance_id)
        .def_readonly_static("NO_SEMANTIC_VALUE",
                             &sfmmap::map::Observation::NO_SEMANTIC_VALUE)
        .def("copy", [](const sfmmap::map::Observation &to_copy) {
            return sfmmap::map::Observation(to_copy);
        });

    py::class_<sfmmap::map::Landmark>(m, "Landmark")
        .def(py::init<const sfmmap::map::LandmarkId&, const Vec3d&>())
        .def_readonly("id", &sfmmap::map::Landmark::id_)
        .def_property("coordinates",
                      &sfmmap::map::Landmark::GetGlobalPos,
                      &sfmmap::map::Landmark::SetGlobalPos)
        .def("get_observations", &sfmmap::map::Landmark::GetObservations,
             py::return_value_policy::reference_internal)
        .def("number_of_observations",
             &sfmmap::map::Landmark::NumberOfObservations)
        .def_property("reprojection_errors",
                      &sfmmap::map::Landmark::GetReprojectionErrors,
                      &sfmmap::map::Landmark::SetReprojectionErrors)
        .def_property("color",
                      &sfmmap::map::Landmark::GetColor,
                      &sfmmap::map::Landmark::SetColor);

    py::class_<sfmmap::map::ShotMeasurements>(m, "ShotMeasurements")
        .def(py::init<>())
        .def_readwrite("gps_accuracy",     &sfmmap::map::ShotMeasurements::gps_accuracy_)
        .def_readwrite("gps_position",     &sfmmap::map::ShotMeasurements::gps_position_)
        .def_readwrite("orientation",      &sfmmap::map::ShotMeasurements::orientation_)
        .def_readwrite("capture_time",     &sfmmap::map::ShotMeasurements::capture_time_)
        .def_readwrite("gravity_down",     &sfmmap::map::ShotMeasurements::gravity_down_)
        .def_readwrite("compass_angle",    &sfmmap::map::ShotMeasurements::compass_angle_)
        .def_readwrite("compass_accuracy", &sfmmap::map::ShotMeasurements::compass_accuracy_)
        .def_readwrite("opk_angles",       &sfmmap::map::ShotMeasurements::opk_angles_)
        .def_readwrite("opk_accuracy",     &sfmmap::map::ShotMeasurements::opk_accuracy_)
        .def_readwrite("sequence_key",     &sfmmap::map::ShotMeasurements::sequence_key_)
        .def_property("attributes",
                      &sfmmap::map::ShotMeasurements::GetAttributes,
                      &sfmmap::map::ShotMeasurements::SetAttributes)
        .def("set", &sfmmap::map::ShotMeasurements::Set);

    py::class_<sfmmap::map::RigCamera>(m, "RigCamera")
        .def(py::init([](const ::geometry::Pose &pose,
                         sfmmap::map::RigCameraId id) {
            sfmmap::map::RigCamera rc;
            rc.pose = pose;
            rc.id   = std::move(id);
            return rc;
        }))
        .def_readwrite("id",   &sfmmap::map::RigCamera::id)
        .def_readwrite("pose", &sfmmap::map::RigCamera::pose);

    py::class_<sfmmap::map::RigInstance>(m, "RigInstance")
        .def(py::init<sfmmap::map::RigInstanceId>())
        .def_property_readonly("id", &sfmmap::map::RigInstance::GetId)
        .def_property("pose",
                      py::overload_cast<>(&sfmmap::map::RigInstance::GetPose),
                      &sfmmap::map::RigInstance::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("shots",
                      py::overload_cast<>(
                          &sfmmap::map::RigInstance::GetShots, py::const_),
                      py::return_value_policy::reference_internal)
        .def("add_shot", &sfmmap::map::RigInstance::AddShot)
        .def("remove_shot", &sfmmap::map::RigInstance::RemoveShot);

    shotCls
        .def(py::init<const sfmmap::map::ShotId&, const ::geometry::Camera&, const ::geometry::Pose&>())
        .def_readonly("id", &sfmmap::map::Shot::id_)
        .def_property("pose", py::overload_cast<>(&sfmmap::map::Shot::GetPose),
                      &sfmmap::map::Shot::SetPose,
                      py::return_value_policy::reference_internal)
        .def_property_readonly("camera", &sfmmap::map::Shot::GetCamera,
                               py::return_value_policy::reference_internal);

    mapCls.def(py::init<>());
}
