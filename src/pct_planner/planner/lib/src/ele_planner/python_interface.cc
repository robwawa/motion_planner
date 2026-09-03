#include "ele_planner/offline_ele_planner.h"
#include <algorithm>

#include "pybind11/eigen.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(ele_planner, m) {
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      .def(py::init<double, bool>(), py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      .def("init_map", &OfflineElePlanner::InitMap)
      .def("plan", &OfflineElePlanner::Plan)
      .def(
          "set_dynamic_cost_map",
          [](OfflineElePlanner& planner,
             py::array_t<uint8_t,
                         py::array::c_style | py::array::forcecast> costs,
             uint8_t lethal_cost) {
            const py::buffer_info info = costs.request();
            if (info.ndim != 3) {
              throw py::value_error("dynamic cost map must be a 3-D array");
            }
            planner.SetDynamicCostMap(static_cast<const uint8_t*>(info.ptr),
                                      static_cast<size_t>(info.size),
                                      lethal_cost);
          },
          py::arg("costs"), py::arg("lethal_cost"))
      .def("clear_dynamic_cost_map", &OfflineElePlanner::ClearDynamicCostMap)
      .def(
          "get_dynamic_costs",
          [](const OfflineElePlanner& planner,
             py::array_t<uint64_t,
                         py::array::c_style | py::array::forcecast> indices) {
            const py::buffer_info info = indices.request();
            if (info.ndim != 1) {
              throw py::value_error(
                  "dynamic cost indices must be one-dimensional");
            }
            const auto* data = static_cast<const uint64_t*>(info.ptr);
            std::vector<size_t> native_indices;
            native_indices.reserve(static_cast<size_t>(info.size));
            for (py::ssize_t i = 0; i < info.size; ++i) {
              native_indices.push_back(static_cast<size_t>(data[i]));
            }
            const std::vector<uint8_t> values =
                planner.GetDynamicCosts(native_indices);
            py::array_t<uint8_t> output(values.size());
            py::buffer_info output_info = output.request();
            std::copy(values.begin(), values.end(),
                      static_cast<uint8_t*>(output_info.ptr));
            return output;
          },
          py::arg("flat_indices"))
      .def("debug", &OfflineElePlanner::Debug)
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight)
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations)
      .def("get_path_finder", &OfflineElePlanner::get_path_finder)
      .def("get_map", &OfflineElePlanner::get_map)
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer)
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj)
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath);
}
