#include "demo.h"
#include "main.h"
#include "Eigen/Geometry"

class Object
{
public:
    Object(const string name, XVec4 color) : mName(name), color(color.data()) {}
    virtual void Initialize(int group) = 0;
    virtual void set_positions(const Eigen::MatrixXf &position) = 0;
    virtual Eigen::MatrixXf get_positions() = 0;
    virtual void set_velocities(const Eigen::MatrixXf &position) = 0;
    virtual Eigen::MatrixXf get_velocities() = 0;
    virtual void update() {} //self update
    virtual void update_rigid_rotations() {} //self update
    const string mName;
    Vec4 color;
};

class KinematicObject : public Object
{
public:
    int shape_id;

    XVec3 velocity;
    float moveTime;

    KinematicObject(const string name, XVec4 color) : Object(name, color), moveTime(0), velocity({0., 0., 0.}) {}

    void update()
    {
        if (moveTime > 0)
        {
            auto pos = get_positions();
            pos(0) += velocity(0) * g_dt;
            pos(1) += velocity(1) * g_dt;
            pos(2) += velocity(2) * g_dt;
            set_positions(pos);
            moveTime -= g_dt;
            UpdateShapes();
        }
    }

    void set_positions(const Eigen::MatrixXf &position)
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped

        if (position.rows() == 14)
        {
            //only render the prev positions ...
            g_buffers->shapePrevPositions[shape_id].x = position(7);
            g_buffers->shapePrevPositions[shape_id].y = position(8);
            g_buffers->shapePrevPositions[shape_id].z = position(9);
            g_buffers->shapePrevRotations[shape_id].x = position(10);
            g_buffers->shapePrevRotations[shape_id].y = position(11);
            g_buffers->shapePrevRotations[shape_id].z = position(12);
            g_buffers->shapePrevRotations[shape_id].w = position(13);
        }
        else
        {
            g_buffers->shapePrevPositions[shape_id] = g_buffers->shapePositions[shape_id];
            g_buffers->shapePrevRotations[shape_id] = g_buffers->shapeRotations[shape_id];
        }
        g_buffers->shapePositions[shape_id].x = position(0);
        g_buffers->shapePositions[shape_id].y = position(1);
        g_buffers->shapePositions[shape_id].z = position(2);
        g_buffers->shapeRotations[shape_id].x = position(3);
        g_buffers->shapeRotations[shape_id].y = position(4);
        g_buffers->shapeRotations[shape_id].z = position(5);
        g_buffers->shapeRotations[shape_id].w = position(6);
    }

    Eigen::MatrixXf get_positions()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        auto ans = Eigen::VectorXf(7);

        ans(0) = g_buffers->shapePositions[shape_id].x;
        ans(1) = g_buffers->shapePositions[shape_id].y;
        ans(2) = g_buffers->shapePositions[shape_id].z;
        ans(3) = g_buffers->shapeRotations[shape_id].x;
        ans(4) = g_buffers->shapeRotations[shape_id].y;
        ans(5) = g_buffers->shapeRotations[shape_id].z;
        ans(6) = g_buffers->shapeRotations[shape_id].w;
        return ans;
    }

    void set_velocities(const Eigen::MatrixXf &velocity)
    {
        throw std::runtime_error("You can't set velocity for kinematics objects");
    }

    Eigen::MatrixXf get_velocities()
    {
        throw std::runtime_error("You can't get velocity for kinematics objects");
    }
};

class KinematicBox : public KinematicObject
{
public:
    KinematicBox(string _name, XVec3 center, XVec3 scale, XVec4 rotation, XVec4 color) : KinematicObject(_name, color), center(center.data()), halfEdge(scale(0) / 2, scale(1) / 2, scale(2) / 2), rotation(rotation.data()) {}

    void Initialize(int group)
    {
        g_shapeColors.push_back(Vec3(color[0], color[1], color[2]));
        shape_id = g_buffers->shapePositions.size();
        AddBox(halfEdge, center, rotation);
    }
    Vec3 halfEdge;
    Vec3 center;
    Quat rotation;
};

class ParticleObject : public Object
{
public:
    ParticleObject(const string name, XVec4 color) : Object(name, color) {}

    void set_positions(const Eigen::MatrixXf &position)
    {
        if (position.rows() != r - l)
        {
            throw std::runtime_error("size missmatch: input " + string() + " while the position of " + mName + " require " + std::to_string(r - l));
        }
        g_set_rigid = true;
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        for (int i = l, j = 0; i < r; ++i, ++j)
        {
            auto row = position.row(j);
            g_buffers->positions[i].x = row(0);
            g_buffers->positions[i].y = row(1);
            g_buffers->positions[i].z = row(2);
            g_buffers->positions[i].w = row(3);
        }
    }

    Eigen::MatrixXf get_positions()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        auto positions = Eigen::MatrixXf(r - l, 4);
        for (int i = l, j = 0; i < r; ++i, ++j)
        {
            positions(j, 0) = g_buffers->positions[i].x;
            positions(j, 1) = g_buffers->positions[i].y;
            positions(j, 2) = g_buffers->positions[i].z;
            positions(j, 3) = g_buffers->positions[i].w;
        }
        return positions;
    }

    void set_velocities(const Eigen::MatrixXf &velocity)
    {
        if (velocity.rows() != r - l)
        {
            throw std::runtime_error("size missmatch: input " + string() + " while the velocities of " + mName + " require " + std::to_string(r - l));
        }
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        for (int i = l, j = 0; i < r; ++i, ++j)
        {
            auto row = velocity.row(j);
            g_buffers->velocities[i].x = row(0);
            g_buffers->velocities[i].y = row(1);
            g_buffers->velocities[i].z = row(2);
        }
    }

    Eigen::MatrixXf get_velocities()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        auto velocity = Eigen::MatrixXf(r - l, 3);
        for (int i = l, j = 0; i < r; ++i, ++j)
        {
            velocity(j, 0) = g_buffers->velocities[i].x;
            velocity(j, 1) = g_buffers->velocities[i].y;
            velocity(j, 2) = g_buffers->velocities[i].z;
        }
        return velocity;
    }
    int l;
    int r;
};

class ParticleShape : public ParticleObject
{
public:
    ParticleShape(string _name, string _filename, XVec3 lower, XVec3 scale, float rotation, XVec4 color, float invMass, float spacing = 0.05f, XVec3 axis=XVec3({0.0f, 1.0f, 0.0f}), XVec3 velocity=XVec3({0.0f, 0.0f, 0.0f})) : ParticleObject(_name, color), filename(_filename), lower(lower.data()), scale(scale.data()), rotation(rotation), spacing(spacing), velocity(velocity.data()), invMass(invMass), rigid(true), rigidStiffness(1.), skin(true), jitter(0.0f), skinOffset(0.0f), skinExpand(0.0f), springStiffness(0.0f), axis(axis.data())
    {
    }

    //void set_positions(const Eigen::MatrixXf& position){
    //    ParticleObject::set_positions(position);
    //	NvFlexGetRigids(g_solver, NULL, NULL, NULL, NULL, NULL, NULL, NULL, g_buffers->rigidRotations.buffer, g_buffers->rigidTranslations.buffer);
    //}

    void rotate(const Eigen::MatrixXf &rotation)
    {
        g_set_rigid = true;
        auto position = get_positions();
        position.block(0, 0, position.rows(), 3) = position.block(0, 0, position.rows(), 3) * rotation;
        set_positions(position);
        if (rigid)
        {
            auto xxx = Matrix33(Vec3(rotation(0, 0), rotation(1, 0), rotation(2, 0)), Vec3(rotation(0, 1), rotation(1, 1), rotation(2, 1)), Vec3(rotation(0, 2), rotation(1, 2), rotation(2, 2)));
            Quat ans = Quat(xxx) * g_buffers->rigidRotations[rigid_index];

            g_buffers->rigidRotations[rigid_index].x = ans.x;
            g_buffers->rigidRotations[rigid_index].y = ans.y;
            g_buffers->rigidRotations[rigid_index].z = ans.z;
            g_buffers->rigidRotations[rigid_index].w = ans.w;
        }
    }

    void update_rigid_rotations(){
        auto tmp = get_positions();
        //cout<<"UPDATE"<<endl;
        Eigen::MatrixXf position = tmp.block(0, 0, tmp.rows(), 3);
        Eigen::Vector3f c = position.colwise().mean(); 
        //double cc[] = {0, 0, 0};
        for(size_t i = 0;i<position.rows();++i){
            for(size_t j=0; j<3; ++j){
                position(i, j) = position(i, j) - c(j);
                //cc[j] += position(i, j);
            }
        }
        //cout<<cc[0] <<" "<<cc[1]<<" "<<cc[2]<<endl;
        //cout<<position.rows()<<" "<<position.cols()<<endl;

        //cout<<position.size()<<endl;
        //cout<<"get center pose"<<" "<<position.rows()<<endl;

        auto local_pose = Eigen::MatrixXf(position.rows(), 3);

		const int startIndex = g_buffers->rigidOffsets[rigid_index];
		const int endIndex = g_buffers->rigidOffsets[rigid_index+1];
		for (int j=startIndex; j < endIndex; ++j)
		{
			//const int r = indices[j];
			//localPositions[count++] = Vec3(restPositions[r]) - translations[i];
            auto tmp = local_pose.row(j-startIndex);
            tmp(0) = g_buffers->rigidLocalPositions[j].x;
            tmp(1) = g_buffers->rigidLocalPositions[j].y;
            tmp(2) = g_buffers->rigidLocalPositions[j].z;
        }
        //cout<<"get local pose"<<endl;
        //cout<<local_pose.size()<<endl;
        //cout<<position.size()<<endl;
        auto rotation = Eigen::umeyama(local_pose.transpose(), position.transpose(), false);
        //cout<<"get rotation"<<" "<<rotation.rows()<<" "<<rotation.cols()<<endl;
        //cout<<rotation<<endl;
        auto xxx = Matrix33(Vec3(rotation(0, 0), rotation(1, 0), rotation(2, 0)), Vec3(rotation(0, 1), rotation(1, 1), rotation(2, 1)), Vec3(rotation(0, 2), rotation(1, 2), rotation(2, 2)));
        Quat ans = Quat(xxx);
        //cout<<" get quat"<<endl;
        g_buffers->rigidRotations[rigid_index].x = ans.x;
        g_buffers->rigidRotations[rigid_index].y = ans.y;
        g_buffers->rigidRotations[rigid_index].z = ans.z;
        g_buffers->rigidRotations[rigid_index].w = ans.w;
    }

    Eigen::VectorXf get_pose()
    {
        MapBuffers(g_buffers);
        Quat rot = g_buffers->rigidRotations[rigid_index];
        Vec3 trans = g_buffers->rigidTranslations[rigid_index];
        auto out = Eigen::VectorXf(7);
        out(0) = rot.w; out(1) = rot.x; out(2) = rot.y; out(3) = rot.z;
        out(4) = trans.x; out(5) = trans.y; out(6) = trans.z;
        return out;
    }

    void Initialize(int group)
    {
        l = g_buffers->positions.size();
        CreateParticleShape(GetFilePathByPlatform(filename.c_str()).c_str(), lower, scale, rotation, spacing, velocity, invMass, rigid, rigidStiffness, NvFlexMakePhase(group, 0), skin, jitter, skinOffset, skinExpand, color, springStiffness, axis);
        r = g_buffers->positions.size();

        if (rigid)
        {
            rigid_index = g_buffers->rigidOffsets.size() - 2;
        }
        else
        {
            rigid_index = -1;
        }
    }

    const string filename;
    Vec3 lower;
    Vec3 scale;
    float rotation;
    Vec3 axis;
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

    int rigid_index;
};

class FluidGrid : public ParticleObject
{
public:
    FluidGrid(string _name, XVec3 lower, int dimx, int dimy, int dimz, float radius, XVec4 color, float invMass, float jitter = 0.005f, XVec3 velocity=XVec3({0.0f, 0.0f, 0.0f})) : ParticleObject(_name, color), lower(lower.data()), dimx(dimx), dimy(dimy), dimz(dimz), radius(radius), velocity(velocity.data()), invMass(invMass), rigid(false), rigidStiffness(0), jitter(jitter)
    {
    }

    void Initialize(int group)
    {
        l = g_buffers->positions.size();
        CreateParticleGrid(lower, dimx, dimy, dimz, radius, velocity, invMass, rigid, rigidStiffness, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), jitter);
        r = g_buffers->positions.size();

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

    int _numIterations = 3;
    // gravity is ommitted, always points to -y axis.

    float _radius = 0.15f;
    float _solidRestDistance = 0.0f;
    float _fluidRestDistance = 0.0f;

    //common
    float _dynamicFriction = 0.0f;
    float _staticFriction = 0.0f;
    float _particleFriction = 0.0f;
    float _restitution = 0.0f;
    float _adhesion = 0.0f;
    float _sleepThreshold = 0.0f;
    float _maxSpeed = FLT_MAX;
    float _maxAcceleration = 100.0f;
    float _shockPropagation = 0.0f;
    float _dissipation = 0.0f;
    float _damping = 0.0f;

    //cloth
    float _drag = 0.0f;
    float _lift = 0.0f;

    //fluid
    float _cohesion = 0.025f;
    float _surfaceTension = 0.0f;
    float _viscosity = 0.0f;
    float _vorticityConfinement = 0.0f;
    float _anisotropyScale = 1.0f;
    float _anisotropyMin = 0.1f;
    float _anisotropyMax = 2.0f;
    float _smoothing = 1.0f;
    float _solidPressure = 1.0f;
    float _freeSurfaceDrag = 0.0f;
    float _buoyancy = 1.0f;

    //diffuse
    float _diffuseThreshold = 100.0f;
    float _diffuseBuoyancy = 1.0f;
    float _diffuseDrag = 0.8f;
    float _diffuseBallistic = 16;
    float _diffuseLifetime = 2.0f;

    //particles
    float _collisionDistance = 0.0f;
    float _particleCollisionMargin = 0.0f;
    float _shapeCollisionMargin = 0.0f;

    int _numPlanes = 1;
    int _numSubsteps = 2;

    NvFlexRelaxationMode _relaxationMode = eNvFlexRelaxationLocal;
    float _relaxationFactor = 1.0f;

    XVec3 camPos = {6.0f, 8.0f, 18.0f};
    XVec3 camAngle = {0.0f, -DegToRad(20.0f), 0.0f};

    bool _drawMesh = true;
    bool _drawPoints = false;
    bool _drawFluids = true;
    bool _drawDiffuse = true;
    bool _wireframe = false;
    bool _showHelp = true;
    bool _warmup = true;

    float _maxDiffuseParticles = 64 * 1024;
    float _numExtraParticles = 0;
    float _diffuseScale = 0.5f;
    bool _fixplanes = false;
    XVec3 _sceneLower = XVec3({0.0f, 0.0f, 0.0f});
    XVec3 _sceneUpper = XVec3({-FLT_MAX, -FLT_MAX, -FLT_MAX});
    //g_maxDiffuseParticles = 0;
    //g_diffuseScale = 0.5f;

    void set_params()
    {
        g_params.numIterations = _numIterations;

        g_params.radius = _radius;
        g_params.solidRestDistance = _solidRestDistance;
        g_params.fluidRestDistance = _fluidRestDistance;

        //common
        g_params.dynamicFriction = _dynamicFriction;
        g_params.staticFriction = _staticFriction;
        g_params.particleFriction = _particleFriction;
        g_params.restitution = _restitution;
        g_params.adhesion = _adhesion;
        g_params.sleepThreshold = _sleepThreshold;
        g_params.maxSpeed = _maxSpeed;
        g_params.maxAcceleration = _maxAcceleration;
        g_params.shockPropagation = _shockPropagation;
        g_params.dissipation = _dissipation;
        g_params.damping = _damping;

        //cloth
        g_params.drag = _drag;
        g_params.lift = _lift;

        //fluid
        g_params.cohesion = _cohesion;
        g_params.surfaceTension = _surfaceTension;
        g_params.viscosity = _viscosity;
        g_params.vorticityConfinement = _vorticityConfinement;
        g_params.anisotropyScale = _anisotropyScale;
        g_params.anisotropyMin = _anisotropyMin;
        g_params.anisotropyMax = _anisotropyMax;
        g_params.smoothing = _smoothing;
        g_params.solidPressure = _solidPressure;
        g_params.freeSurfaceDrag = _freeSurfaceDrag;
        g_params.buoyancy = _buoyancy;

        //diffuse
        g_params.diffuseThreshold = _diffuseThreshold;
        g_params.diffuseBuoyancy = _diffuseBuoyancy;
        g_params.diffuseDrag = _diffuseDrag;
        g_params.diffuseBallistic = _diffuseBallistic;
        g_params.diffuseLifetime = _diffuseLifetime;

        //particles
        g_params.collisionDistance = _collisionDistance;
        g_params.particleCollisionMargin = _particleCollisionMargin;
        g_params.shapeCollisionMargin = _shapeCollisionMargin;
        g_params.numPlanes = _numPlanes;
    }

    void Initialize()
    {
        int group = 1;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            objects[i]->Initialize(group);
            group++;
        }
        set_params();

        if (!g_centerCamera)
        {
            g_camPos = Vec3(camPos.data());
            g_camAngle = Vec3(camAngle.data());
        }

        g_warmup = _warmup;
        g_sceneLower = Vec3(_sceneLower.data());
        g_sceneUpper = Vec3(_sceneUpper.data());
        g_numSubsteps = _numSubsteps;

        g_maxDiffuseParticles = _maxDiffuseParticles;
        g_numExtraParticles = _numExtraParticles;
        g_diffuseScale = _diffuseScale;

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

        g_waveFloorTilt = 0.0f;
        g_waveFrequency = 1.5f;
        g_waveAmplitude = 2.0f;

        // draw options
        g_drawMesh = _drawMesh;
        g_drawPoints = _drawPoints;
        g_drawEllipsoids = _drawFluids;
        g_drawDiffuse = _drawDiffuse;
        g_wireframe = _wireframe;
        g_showHelp = _showHelp;

        g_fixplanes = _fixplanes;
    }

    void add_objects(ObjectPtr obj)
    {
        objects.push_back(obj);
    }

    void remove_objects(ObjectPtr obj)
    {
        //remove by name
        for (size_t i = 0; i < objects.size(); ++i)
        {
            if (objects[i]->mName == obj->mName)
            {
                objects.erase(objects.begin() + i);
                break;
            }
        }
    }

    void clear()
    {
        //not sure if we need to erase it..
        objects.clear();
    }

    int n_objects()
    {
        return objects.size();
    }

    ObjectPtr get_objects(int i)
    {
        return objects[i];
    }

    void Update()
    {
        for (size_t i = 0; i < objects.size(); ++i)
            objects[i]->update();
    }

    void update_rigid_rotations(){
        for(size_t i=0;i<objects.size();++i){
            objects[i]->update_rigid_rotations();
        }
    }

    vector<ObjectPtr> objects;
};

int num_sim = 0;

class Simulator
{
public:
    ScenePtr myscene;
    int scene_id;
    AgentPtr agent;
    bool start;

    int newScene;
    bool new_frame;
    double lastTime;
    double frameBeginTime;
    float newSimLatency;
    float newGfxLatency;
    double renderBeginTime;
    double renderEndTime;
    double updateBeginTime;
    double updateEndTime;

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

        g_agent = new Agent();
        agent = AgentPtr(g_agent);

        start = false;
    }

    void pre_scene_update()
    {
        if (!new_frame)
            return;
        new_frame = false;

        static double lastTime;
        frameBeginTime = GetSeconds();
        g_realdt = float(frameBeginTime - lastTime);
        lastTime = frameBeginTime;

        //-------------------------------------------------------------------
        // Scene Update
        MapBuffers(g_buffers);

        // Getting timers causes CPU/GPU sync, so we do it after a map
        newSimLatency = NvFlexGetDeviceLatency(g_solver, &g_GpuTimers.computeBegin, &g_GpuTimers.computeEnd, &g_GpuTimers.computeFreq);
        newGfxLatency = 0;
        if (g_render)
        {
            newGfxLatency = RendererGetDeviceTimestamps(&g_GpuTimers.renderBegin, &g_GpuTimers.renderEnd, &g_GpuTimers.renderFreq);
        }
        (void)newGfxLatency;

        if (g_render)
            UpdateCamera();

        if (!g_pause || g_step)
        {
            UpdateEmitters();
            if (g_render)
                UpdateMouse();
            UpdateWind();
            UpdateScene();
            //if(g_agent!=nullptr)
            //    g_agent->update();
        }

        //TODO
        if(g_set_rigid){
            myscene->update_rigid_rotations();
        }
    }

    bool step()
    {
        if (!start)
            throw runtime_error("You must reset the scene to step");

        pre_scene_update();

        updateBeginTime = GetSeconds();
        FlexStep();
        updateEndTime = GetSeconds();

        //-------------------------------------------------------
        // Update the on-screen timers

        float newUpdateTime = float(updateEndTime - updateBeginTime);
        float newRenderTime = float(renderEndTime - renderBeginTime);

        // Exponential filter to make the display easier to read
        const float timerSmoothing = 0.05f;

        g_updateTime = (g_updateTime == 0.0f) ? newUpdateTime : Lerp(g_updateTime, newUpdateTime, timerSmoothing);
        g_renderTime = (g_renderTime == 0.0f) ? newRenderTime : Lerp(g_renderTime, newRenderTime, timerSmoothing);
        g_simLatency = (g_simLatency == 0.0f) ? newSimLatency : Lerp(g_simLatency, newSimLatency, timerSmoothing);

        // flush out the last frame before freeing up resources in the event of a scene change
        // this is necessary for d3d12
        if (g_render)
        {
            PresentFrame(g_vsync);
        }

        // if gui or benchmark requested a scene change process it now
        if (newScene != -1)
        {
            g_scene = newScene;
            Init(g_scene);
        }

        new_frame = true;

        if (g_render)
        {
            return SDLMain();
        }
        else
        {
            return false;
        }
    }

    bool step2()
    {
        //step with Nvidia pyflex
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

    py::array_t<uint8_t> render(string mode = "human")
    {
        pre_scene_update();

        renderBeginTime = GetSeconds();
        newScene = RenderStep();
        renderEndTime = GetSeconds();

        if (mode == "rgb_array")
        {
            auto data = new uint32_t[g_screenWidth * g_screenHeight];
            ReadFrame((int *)data, g_screenWidth, g_screenHeight);

            auto ans = py::array_t<uint8_t>({g_screenHeight, g_screenWidth, 4}, (unsigned char *)data);
            delete[] data;
            return ans;
        }
        else
        {
            return py::array_t<uint8_t>({});
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
        start = true;

        newScene = -1;
        new_frame = true;
        // reset the scene ...
        g_scene = scene_id;
        Reset(centerCamera);
        agent->reset();
    }

    void sim_srand(unsigned int seed)
    {
        srand(seed);
    }

    ~Simulator()
    {
        destroy_scene();
    }

    AgentPtr get_agent()
    {
        return agent;
    }

    Eigen::MatrixXf get_positions()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        int size = NvFlexGetActiveCount(g_solver);
        auto positions = Eigen::MatrixXf(size, 4);
        for (int i = 0; i < size; ++i)
        {
            positions(i, 0) = g_buffers->positions[i].x;
            positions(i, 1) = g_buffers->positions[i].y;
            positions(i, 2) = g_buffers->positions[i].z;
            positions(i, 3) = g_buffers->positions[i].w;
        }
        return positions;
    }

    void set_positions(Eigen::MatrixXf positions)
    {
        g_set_rigid = true;
        // only for rendering ...
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        int size = NvFlexGetActiveCount(g_solver);
        for (int i = 0; i < size; ++i)
        {
            g_buffers->positions[i].x = positions(i, 0);
            g_buffers->positions[i].y = positions(i, 1);
            g_buffers->positions[i].z = positions(i, 2);
        }

        // update the attributes of the rigids ...
		//const int numRigids = g_buffers->rigidOffsets.size() - 1;
        //CalculateRigidCentersOfMass(&g_buffers->positions[0], g_buffers->positions.size(), &g_buffers->rigidOffsets[0], &g_buffers->rigidTranslations[0], &g_buffers->rigidIndices[0], numRigids);
    }

    Eigen::MatrixXf get_planes()
    {
        MapBuffers(g_buffers);
        auto planes = Eigen::MatrixXf(g_params.numPlanes, 4);
        for (int i = 0; i < (int)g_params.numPlanes; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                planes(i, j) = g_params.planes[i][j];
            }
        }
        return planes;
    }

    Eigen::MatrixXf get_velocities()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        int size = NvFlexGetActiveCount(g_solver);
        auto velocities = Eigen::MatrixXf(size, 3);
        for (int i = 0; i < size; ++i)
        {
            velocities(i, 0) = g_buffers->velocities[i].x;
            velocities(i, 1) = g_buffers->velocities[i].y;
            velocities(i, 2) = g_buffers->velocities[i].z;
        }
        return velocities;
    }


    void set_velocities(Eigen::MatrixXf velocities)
    {
        // only for rendering ...
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        int size = NvFlexGetActiveCount(g_solver);
        for (int i = 0; i < size; ++i)
        {
            g_buffers->velocities[i].x = velocities(i, 0);
            g_buffers->velocities[i].y = velocities(i, 1);
            g_buffers->velocities[i].z = velocities(i, 2);
        }
    }


    Eigen::MatrixXi get_rigid_indices()
    {
        MapBuffers(g_buffers); // map the buffer if it's unmapped
        auto edges = Eigen::MatrixXi(g_buffers->rigidIndices.size(), 2);
        for (int i = 0; i < g_buffers->rigidOffsets.size()-1; ++i)
        {
            
            int l = g_buffers->rigidOffsets[i];
            int r = g_buffers->rigidOffsets[i+1];
            for(int j=l;j<r;++j){
                edges(j, 0) = i;
                edges(j, 1) = g_buffers->rigidIndices[j];
            }
        }
        return edges;
    }

    XVec3 get_sceneLower(){
        return XVec3({g_sceneLower.x, g_sceneLower.y, g_sceneLower.z});
    }

    XVec3 get_sceneUpper(){
        return XVec3({g_sceneUpper.x, g_sceneUpper.y, g_sceneUpper.z});
    }

    float get_dt(){
        return g_dt;
    }
};

void Agent::update()
{
    //clear
    if (w || a || s || d)
    {
        float cc = speed * g_dt;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            Eigen::MatrixXf vel = objects[i]->get_velocities();
            auto x = Eigen::VectorXf::Ones(vel.rows());
            if (w)
                vel.col(2) -= cc * x;
            if (s)
                vel.col(2) += cc * x;
            if (a)
                vel.col(0) -= cc * x;
            if (d)
                vel.col(0) += cc * x;
            objects[i]->set_velocities(vel);
        }
        w = a = s = d = false;
    }
}

void Agent::add_object(ObjectPtr obj)
{
    objects.push_back(obj);
}

void Agent::reset()
{
    //mTime = 0;
    w = a = s = d = j = k = false;
}