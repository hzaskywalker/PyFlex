//Written by Henry M. Clever. November 15, 2018.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include "main.cpp" //we can do this because of the FleX_INCLUDE_DIR in FindFleX.cmake
#include <stdio.h>

int add(int i, int j)
{
    return 100; //deadzones[1];
}

namespace py = pybind11;

py::array_t<double> make_array()
{
    // No pointer is passed, so NumPy will allocate the buffer
    printf("a;lsfjl;\n");
    auto myarray = py::array_t<double>(10);

    return myarray;
}

PYBIND11_MODULE(pyflex, m)
{
    m.doc() = "pybind11 passing plugin";

    m.def("update_frame", &UpdateFrame, "void update frame");
    m.def("sdl_main", &SDLMain, py::return_value_policy::automatic);
    m.def("initialize", &initialize, "float init");
    m.def("destroy_scene", &destroy_scene, py::return_value_policy::automatic);

    m.def("RandInit", &RandInit, "RandInit");
    m.def("add", &add, "Add two numbers");
    m.def("grab_z_pos_particle", &grab_z_pos_particle, py::return_value_policy::automatic);
    m.def("grab_y_pos_particle", &grab_y_pos_particle, py::return_value_policy::automatic);
    m.def("grab_x_pos_particle", &grab_x_pos_particle, py::return_value_policy::automatic);
    m.def("make_array", &make_array, py::return_value_policy::move); // Return policy can be left default, i.e. return_value_policy::automatic

    m.def(
        "subtract", [](int i, int j) { return i - j; }, "Subtract two numbers");

    auto PySimulator = py::class_<Simulator>(m, "Simulator");
    auto PyScene = py::class_<MyScene, ScenePtr>(m, "Scene");
    auto PyObject = py::class_<Object, ObjectPtr>(m, "Object");

    auto PyParticleObject = py::class_<ParticleObject>(m, "ParticleObject", PyObject);
    auto PyParticleShape = py::class_<ParticleShape, ParticleShapePtr>(m, "Shape", PyParticleObject);
    auto PyFluidGrid = py::class_<FluidGrid, FluidGridPtr>(m, "Fluid", PyParticleObject);

    auto PyKinematicObject = py::class_<KinematicObject>(m, "KObject", PyObject);
    auto PyKinematicBox = py::class_<KinematicBox, KinematicBoxPtr>(m, "KBox", PyKinematicObject);

    auto PyAgent = py::class_<Agent, AgentPtr>(m, "Agent");

    PySimulator.def(py::init<bool>(), py::arg("rendering") = false)
        .def("reset", &Simulator::reset, py::arg("center") = true)
        .def("get_scene", &Simulator::get_scene, py::return_value_policy::reference)
        .def("set_scene", &Simulator::set_scene, py::arg("name") = "empty")
        .def("render", &Simulator::render, py::return_value_policy::automatic)
        .def("get_agent", &Simulator::get_agent, py::return_value_policy::reference)
        .def("step", &Simulator::step);

    PyScene.def(py::init<char *>(), py::arg("name"))
        .def_readwrite("radius", &MyScene::_radius)
        .def_readwrite("dynamicFriction", &MyScene::_dynamicFriction)
        .def_readwrite("viscosity", &MyScene::_viscosity)
        .def_readwrite("numIterations", &MyScene::_numIterations)
        .def_readwrite("vorticityConfinement", &MyScene::_vorticityConfinement)
        .def_readwrite("fluidRestDistance", &MyScene::_fluidRestDistance)
        .def_readwrite("solidPressure", &MyScene::_solidPressure)
        .def_readwrite("relaxationFactor", &MyScene::_relaxationFactor)
        .def_readwrite("cohesion", &MyScene::_cohesion)
        .def_readwrite("collisionDistance", &MyScene::_collisionDistance)
        .def_readwrite("numPlanes", &MyScene::_numPlanes)
        .def_readwrite("camPos", &MyScene::camPos)
        .def_readwrite("camAngle", &MyScene::camAngle)
        .def("add", &MyScene::add_objects, py::arg("object") = ObjectPtr(0));

    /* --------------------- Particle Objects .......................*/

    PyObject.def_property_readonly("name", [](Object &shape) { return shape.mName; }).def_property("position", &Object::get_positions, &Object::set_positions)
    .def_property("velocity", &Object::get_velocities, &Object::set_velocities);

    PyParticleShape.def(py::init<string, string, XVec3, XVec3, float, XVec4, float, float>(), py::arg("name"), py::arg("path"), py::arg("lower") = XVec3({0, 0, 0}), py::arg("scale") = XVec3({1., 1., 1.}), py::arg("rotation") = 0., py::arg("color") = XVec4({0.0f, 0.0f, 0.0f, 0.0f}), py::arg("invMass") = 1.0f, py::arg("spacing") = 0.05)
        .def_property_readonly("filename", [](ParticleShape &shape) { return shape.filename; });

    PyFluidGrid.def(py::init<string, XVec3, int, int, int, float, XVec4, float, float>(), py::arg("name"), py::arg("lower") = XVec3({0, 0, 0}), py::arg("dimx") = 40, py::arg("dimy") = 40, py::arg("dimz") = 40, py::arg("radius") = 0.03f, py::arg("color") = XVec4({0.113f, 0.425f, 0.55f, 1.f}), py::arg("invMass") = 1.0f, py::arg("jitter") = 0.005f);


    /* ...................Kinematic Objects...................*/
    PyKinematicBox.def(py::init<string, XVec3, XVec3, XVec4, XVec4>(), py::arg("name"), py::arg("center")=XVec3({0, 0, 0}), py::arg("scale")=XVec3(0.1, 0.1, 0.1), py::arg("rotation")=XVec4({0, 0, 0, 1.}), py::arg("color")=XVec4({0.9, 0.9, 0.9, 1.}));


    PyAgent.def("add", &Agent::add_object, py::arg("object") = ObjectPtr(0))
    .def_readwrite("speed", &Agent::speed);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
