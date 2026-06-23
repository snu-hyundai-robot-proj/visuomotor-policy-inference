#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <memory>

#include "dual_hdr_controller.h"

namespace py = pybind11;
using namespace dual_hdr_controller;

static void ros_init()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

static void ros_shutdown()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

static void ros_spin_once(const std::shared_ptr<rclcpp::Node>& node)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin_once();         // 여기서 블로킹
  exec.remove_node(node);
}

PYBIND11_MODULE(snu_hdr_controller_py, m)
{
    m.def("ros_init", &ros_init);
    m.def("ros_shutdown", &ros_shutdown);
    m.def("ros_spin_once", &ros_spin_once);

    py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node>>(m, "Node")
        .def(py::init<const std::string &>());

    py::class_<DualHdrController, std::shared_ptr<DualHdrController>>(m, "DualHdrController")
        .def(py::init<const std::string&, const rclcpp::Node::SharedPtr&>())        
        .def("initialize", &DualHdrController::initialize)
        .def("update", &DualHdrController::update)
        .def("validate", &DualHdrController::validate)
        .def("write", &DualHdrController::write)
        .def("getTimerReset", &DualHdrController::getTimerReset)
        .def("getFtNode", &DualHdrController::getFtNode);        

}