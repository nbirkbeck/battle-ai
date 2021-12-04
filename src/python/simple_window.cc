#include "src/simple_window.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(world_python, m) {
    py::class_<World>(m, "World")
      .def(py::init<>())
      .def("load_from_file", &World::LoadFromFile);
}
