#include "demo.h"
#include <Eigen/Dense>

class MyScene;
class Object;
class Simulator;
class ParticleShape;
class FluidGrid;
using ScenePtr = std::shared_ptr<MyScene>;
using ObjectPtr = std::shared_ptr<Object>;
using SimulatorPtr = std::shared_ptr<Simulator>;
using ParticleShapePtr = std::shared_ptr<ParticleShape>;
using FluidGridPtr = std::shared_ptr<FluidGrid>;
using XVec3 = Eigen::Vector3f;
using XVec4 = Eigen::Vector4f;

class Object
{
public:
    Object(const string name, XVec4 color) : mName(name), color(color.data()) {}
    virtual void Initialize(int group) = 0;
    const string mName;
    Vec4 color;
};

class ParticleShape : public Object
{
public:
    ParticleShape(string _name, string _filename, XVec3 lower, XVec3 scale, float rotation, XVec4 color, float invMass, float spacing = 0.05f) : Object(_name, color), filename(_filename), lower(lower.data()), scale(scale.data()), rotation(rotation), spacing(spacing), velocity(Vec3(0.0f)), invMass(invMass), rigid(true), rigidStiffness(1.), skin(true), jitter(0.0f), skinOffset(0.0f), skinExpand(0.0f), springStiffness(0.0f)
    {
    }

    void Initialize(int group)
    {
        CreateParticleShape(GetFilePathByPlatform(filename.c_str()).c_str(), lower, scale, rotation, spacing, velocity, invMass, rigid, rigidStiffness, NvFlexMakePhase(group, 0), skin, jitter, skinOffset, skinExpand, color, springStiffness);
    }

    const string filename;
    Vec3 lower;
    Vec3 scale;
    float rotation;
    float spacing;
    Vec3 velocity;
    float invMass;
    bool rigid;
    float rigidStiffness;
    int phase;
    bool skin;
    float jitter;
    Vec3 skinOffset;
    float skinExpand;
    float springStiffness;
};

class FluidGrid : public Object
{
public:
    FluidGrid(string _name, XVec3 lower, int dimx, int dimy, int dimz, float radius, XVec4 color, float invMass, float jitter = 0.005f) : Object(_name, color), lower(lower.data()), dimx(dimx), dimy(dimy), dimz(dimz), radius(radius), velocity(Vec3(0.0f)), invMass(invMass), rigid(false), rigidStiffness(0), jitter(jitter)
    {
    }

    void Initialize(int group)
    {
        CreateParticleGrid(lower, dimx, dimy, dimz, radius, velocity, invMass, rigid, rigidStiffness, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), jitter);

        g_fluidColor = color;
    }

    Vec3 lower;
    int dimx, dimy, dimz;
    float radius;
    Vec3 velocity;
    float invMass;
    bool rigid;
    float rigidStiffness;
    int phase;
    float jitter;
};

class MyScene : public Scene
{
public:
    MyScene(const char *name) : Scene(name)
    {
    }

    void Initialize()
    {
        float radius = 0.1f;
        int group = 1;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            objects[i]->Initialize(group);
            group++;
        }

        g_sceneLower = Vec3(0.0f);
        g_numSubsteps = 2;
        float restDistance = radius * 0.55f;

        g_params.radius = radius;
        g_params.dynamicFriction = 0.01f;
        g_params.viscosity = 2.0f;
        g_params.numIterations = 4;
        g_params.vorticityConfinement = 40.0f;
        g_params.fluidRestDistance = restDistance;
        g_params.solidPressure = 0.f;
        g_params.relaxationFactor = 0.0f;
        g_params.cohesion = 0.02f;
        g_params.collisionDistance = 0.01f;

        g_maxDiffuseParticles = 64 * 1024;
        g_numExtraParticles = 48 * 1024;
        g_diffuseScale = 0.5f;

        /*
		Emitter e1;
		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e1.mPos = Vec3(radius, 1.f, 0.65f);
		e1.mSpeed = (restDistance/g_dt)*2.0f; // 2 particle layers per-frame
		e1.mEnabled = true;
		g_emitters.push_back(e1);
        */

        g_lightDistance = 1.8f;

        g_params.numPlanes = 5;

        g_waveFloorTilt = 0.0f;
        g_waveFrequency = 1.5f;
        g_waveAmplitude = 2.0f;

        g_warmup = true;

        // draw options
        g_drawPoints = false;
        g_drawEllipsoids = true;
        g_drawDiffuse = true;
    }

    void add_objects(ObjectPtr obj)
    {
        objects.push_back(obj);
    }

    vector<ObjectPtr> objects;
};

int num_sim = 0;

class Simulator
{
public:
    ScenePtr myscene;
    int scene_id;
    Simulator(bool rendering)
    {
        num_sim += 1;
        if (num_sim > 1)
        {
            throw runtime_error("You can create only one simulator per process...");
        }
        initialize(rendering);
        myscene = ScenePtr(new MyScene("empty"));
        g_scenes.push_back(myscene.get());
        scene_id = g_scenes.size() - 1;
    }

    bool step()
    {
        UpdateFrame();
        if (g_render)
        {
            return SDLMain();
        }
        else
        {
            return false;
        }
    }

    ScenePtr get_scene()
    {
        return myscene;
    }

    void set_scene(char *name)
    {
        for (int i = 0; i < g_scenes.size(); ++i)
        {
            if (strcmp(g_scenes[i]->mName, name) == 0)
            {
                scene_id = i;
                break;
            }
        }
    }

    void reset(bool centerCamera = false)
    {
        // reset the scene ...
        g_scene = scene_id;
        Reset(centerCamera);
    }

    ~Simulator()
    {
        destroy_scene();
    }
};