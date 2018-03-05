#include <pybind11/pybind11.h>
namespace py = pybind11;

void rates_bind(py::module &);
void body_frame_bind(py::module &);

PYBIND11_MODULE(voxbloxpy, m) {
  rates_bind(m);
  body_frame_bind(m);
}
