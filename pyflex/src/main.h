#pragma once
#include <Eigen/Dense>
#include <pybind11/numpy.h>
class MyScene;
class Object;
class Simulator;
class ParticleShape;
class FluidGrid;
class ParticleObject;
class KinematicObject;
class KinematicBox;
using namespace std;
namespace py=pybind11;
using ScenePtr = std::shared_ptr<MyScene>;
using ObjectPtr = std::shared_ptr<Object>;
using SimulatorPtr = std::shared_ptr<Simulator>;
using ParticleShapePtr = std::shared_ptr<ParticleShape>;
using FluidGridPtr = std::shared_ptr<FluidGrid>;
using ParticleObjectPtr = std::shared_ptr<ParticleObject>;
using KinematicObjectPtr = std::shared_ptr<KinematicObject>;
using KinematicBoxPtr = std::shared_ptr<KinematicBox>;
using XVec3 = Eigen::Vector3f;
using XVec4 = Eigen::Vector4f;

class Agent
{
public:
    // Agent is contorllered by
    bool w, a, s, d, j, k;
    float speed = 1.f;
    void reset();

    vector<ObjectPtr> objects;
    void update();
    void add_object(ObjectPtr obj);
};

using AgentPtr = std::shared_ptr<Agent>;