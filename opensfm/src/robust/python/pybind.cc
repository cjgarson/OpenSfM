#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <robust/instanciations.h>
#include <robust/robust_estimator.h>
#include <robust/scorer.h>

template <class T>
void AddScoreType(py::module& m, const std::string& name) {
  py::class_<ScoreInfo<T>>(m, ("ScoreInfo" + name).c_str())
      .def(py::init())
      .def_readwrite("score", &ScoreInfo<T>::score)
      .def_readwrite("model", &ScoreInfo<T>::model)
      .def_readwrite("lo_model", &ScoreInfo<T>::lo_model)
      .def_readwrite("inliers_indices", &ScoreInfo<T>::inliers_indices);
}

PYBIND11_MODULE(pyrobust, m) {
  AddScoreType<Line::Type>(m, "Line");
  AddScoreType<Eigen::Matrix3d>(m, "Matrix3d");
  AddScoreType<Eigen::Matrix4d>(m, "Matrix4d");
  AddScoreType<Eigen::Matrix<double, 3, 4>>(m, "Matrix34d");
  AddScoreType<Eigen::Vector3d>(m, "Vector3d");

  py::class_<RobustEstimatorParams>(m, "RobustEstimatorParams")
      .def(py::init())
      .def_readwrite("iterations", &RobustEstimatorParams::iterations)
      .def_readwrite("probability", &RobustEstimatorParams::probability)
      .def_readwrite("use_local_optimization",
                     &RobustEstimatorParams::use_local_optimization)
      .def_readwrite("use_iteration_reduction",
                     &RobustEstimatorParams::use_iteration_reduction);

  m.def("ransac_line",
        [](const Eigen::MatrixXd& pts, double threshold,
           const robust::RobustEstimatorParams& params) {
          return robust::RANSACLine(pts, threshold, params);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_essential",
        [](const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2, double threshold,
           const robust::RobustEstimatorParams& params, const robust::RansacType& rtype) {
          return robust::RANSACEssential(p1, p2, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_relative_pose",
        [](const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2, double threshold,
           const robust::RobustEstimatorParams& params, const robust::RansacType& rtype) {
          return robust::RANSACRelativePose(p1, p2, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_relative_rotation",
        [](const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2, double threshold,
           const robust::RobustEstimatorParams& params, const robust::RansacType& rtype) {
          return robust::RANSACRelativeRotation(v1, v2, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_absolute_pose",
        [](const Eigen::MatrixXd& bearings,
           const Eigen::MatrixXd& points_world,
           double threshold,
           const robust::RobustEstimatorParams& params,
           const robust::RansacType& rtype) {
          return robust::RANSACAbsolutePose(bearings, points_world, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_absolute_pose_known_rotation",
        [](const Eigen::MatrixXd& bearings,
           const Eigen::MatrixXd& points_world,
           const Eigen::Matrix3d& R_w_c,
           double threshold,
           const robust::RobustEstimatorParams& params,
           const robust::RansacType& rtype) {
          return robust::RANSACAbsolutePoseKnownRotation(
              bearings, points_world, R_w_c, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  m.def("ransac_similarity",
        [](const Eigen::MatrixXd& x1, const Eigen::MatrixXd& x2, double threshold,
           const robust::RobustEstimatorParams& params, const robust::RansacType& rtype) {
          return robust::RANSACSimilarity(x1, x2, threshold, params, rtype);
        },
        py::call_guard<py::gil_scoped_release>());

  py::enum_<RansacType>(m, "RansacType")
      .value("RANSAC", RansacType::RANSAC)
      .value("MSAC", RansacType::MSAC)
      .value("LMedS", RansacType::LMedS)
      .export_values();
}
