#include <pybind11/pybind11.h>
namespace py = pybind11;

void rates_bind(py::module &);
void body_frame_bind(py::module &);
void lee_control_bind(py::module &);

PYBIND11_MODULE(quest_gncpy, m) {
  rates_bind(m);
  body_frame_bind(m);
  lee_control_bind(m);
}
