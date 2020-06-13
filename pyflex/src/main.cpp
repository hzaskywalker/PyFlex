#include "demo.h"

class MyScene;
class Object;
class Simulator;
class ParticleShape;
using ScenePtr=std::shared_ptr<MyScene>;
using ObjectPtr=std::shared_ptr<Object>;
using SimulatorPtr=std::shared_ptr<Simulator>;
using ParticleShapePtr=std::shared_ptr<ParticleShape>;

class Object
{
public:
    Object(const char *name) : mName(name) {}
    virtual void Initialize(int group) = 0;
    const char *mName;
};

class ParticleShape : public Object
{
public:
    ParticleShape(const char *name, const char *filename, Vec3 lower, Vec3 scale, float rotation, Vec4 color, float invMass) : Object(name), filename(filename), lower(lower), scale(scale), rotation(rotation), spacing(0.0f), velocity(Vec3(0.0f)), invMass(invMass), rigid(true), rigidStiffness(1.), skin(true), jitter(0.0f), skinOffset(0.0f), skinExpand(0.0f), color(color), springStiffness(0.0f) {}

    void Initialize(int group)
    {
        CreateParticleShape(GetFilePathByPlatform(filename).c_str(), lower, scale, rotation, spacing, velocity, invMass, rigid, rigidStiffness, NvFlexMakePhase(group, 0), skin, jitter, skinOffset, skinExpand, color, springStiffness);
    }

    const char *filename;
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
    Vec4 color;
    float springStiffness;
};

class MyScene : public Scene
{
public:
    MyScene(const char *name) : Scene(name)
    {
    }

    void Initialize(){
		float radius = 0.1f;
        int group = 1;
        for(size_t i=0;i<objects.size();++i){
            objects[i]->Initialize(group);
            group++;
        }


		g_sceneLower = Vec3(0.0f);
		g_numSubsteps = 2;
		float restDistance = radius*0.55f;

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

		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.5f;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.f);

		Emitter e1;
		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e1.mPos = Vec3(radius, 1.f, 0.65f);
		e1.mSpeed = (restDistance/g_dt)*2.0f; // 2 particle layers per-frame
		e1.mEnabled = true;

		g_emitters.push_back(e1);

		g_numExtraParticles = 48*1024;

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


int num_sim=0;

class Simulator{
public:
    ScenePtr myscene;
    int scene_id;
    Simulator(bool rendering){
        num_sim += 1;
        if(num_sim>1){
            throw runtime_error("You can create only one simulator per process...");
        }
        initialize(rendering);
        myscene = ScenePtr(new MyScene("empty"));
        g_scenes.push_back(myscene.get());
        scene_id = g_scenes.size()-1;
    }

    bool step(){
        UpdateFrame();
        if(g_render){
            return SDLMain();
        } else {
            return false;
        }
    }

    ScenePtr get_scene(){
        return myscene;
    }

    void set_scene(char* name){
        for(int i =0;i<g_scenes.size();++i){
            cout<<g_scenes[i]->mName<<" "<<name<<endl;
            if(strcmp(g_scenes[i]->mName, name)==0){
                scene_id = i;
                break;
            }
        }
    }

    void reset(bool centerCamera=false){
        // reset the scene ...
        g_scene = scene_id;
        Reset(centerCamera);
    }

    ~Simulator(){
        destroy_scene();
    }
};