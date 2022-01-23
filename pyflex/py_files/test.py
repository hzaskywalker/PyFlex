#Written by Henry M. Clever. November 15, 2018.
import torch
import cv2
from time import time
import tqdm
from random import random
import numpy as np

class Agent:
    def __init__(self, sim):
        self.keyboard = sim.get_keyboard()
        self.obj = []
        self.speed = 1

    def add(self, obj):
        self.obj.append(obj)
    
    def update(self):
        cc = 1./60 * self.speed
        w,a,s,d,j,k=self.keyboard.w, self.keyboard.a,self.keyboard.s,self.keyboard.d,self.keyboard.j,self.keyboard.k
        if w or a or s or d or j or k:
            for i in self.obj:
                vel = i.velocity
                if w:
                    vel[:, 2] -= cc
                if s:
                    vel[:, 2] += cc
                if a:
                    vel[:, 0] -= cc
                if d:
                    vel[:, 0] += cc
                if j:
                    vel[:, 1] += 5.
                if k:
                    vel[:, 1] -= 5.
                i.velocity = vel
        self.keyboard.reset()

def test1():
    import flex as flexbind
    main_loop_quit = False


    rendering = True
    flexbind.initialize(rendering)

    if not rendering:
        for i in tqdm.trange(1000):
            flexbind.update_frame()

    else:
        while main_loop_quit == False:
            flexbind.update_frame()
            """
            orig = time()

            pos_array = np.zeros((1000, 3))

            for i in range(0, 1000):
                pos_array[i, 0] = flexbind.grab_x_pos_particle(i)
                pos_array[i, 0] = flexbind.grab_y_pos_particle(i)
                pos_array[i, 2] = flexbind.grab_z_pos_particle(i)

            """

            main_loop_quit = flexbind.sdl_main()
            
        print(main_loop_quit)

    flexbind.destroy_scene()
    print("got here in python!!!")


def test2():
    import flex
    sim = flex.Simulator(rendering=True)
    scene = sim.get_scene()
    s = 0.1 * 0.5
    bunny = flex.Shape(
        "bunny",
        "/home/hza/fluid/PyFlex/data/box.ply", [0.2, 0.2, 0.4], [1., 1., 1.], 0, [0, 0, 0, 0], 0.5, spacing=s
    )
    sphere = flex.Shape(
        "sphere",
        "/home/hza/fluid/PyFlex/data/sphere.ply", [0.2, 0.8, 0.4], [1., 1., 1.], 0, [0, 0, 0, 0], 10, spacing=s
    )
    box = flex.Shape(
        "sphere",
        "/home/hza/fluid/PyFlex/data/bunny.ply", [0.2, 0.8, 0.4], [0.3, 0.3, 0.3], 0, [0, 0, 0, 0], 1, spacing=s
    )
    scene.add(bunny)
    scene.add(sphere)
    scene.add(box)

    fluid = flex.Fluid(
        "fluid", [0.1, 2, 0.1], 40, 10, 40, 0.055, [1, 0.425, 0.55, 1.], 1.
    )

    fluid2 = flex.Fluid(
        "fluid2", [0.1, 4, 0.1], 40, 10, 40, 0.055, [1, 0.425, 0.55, 1.], 1.
    )

    radius = 0.1
    restDistance = radius * 0.55
    scene.radius = radius
    scene.dynamicFriction = 0.01
    scene.viscosity = 2.0
    scene.numIterations = 4
    scene.vorticityConfinement = 40.0
    scene.fluidRestDistance = restDistance
    scene.solidPressure = 0.
    scene.relaxationFactor = 0.0
    scene.cohesion = 0.02
    scene.collisionDistance = 0.01
    scene.numPlanes = 5

    scene.camPos = [1, 2, 10]
    # the second is the rotation about x ...
    # the first is the rotation about y ...
    #scene.camAngle = [0, -np.pi/2 * 0.8, 0]
    scene.camAngle = [0, 0, 0]

    scene.add(fluid)
    scene.add(fluid2)

    box33 = flex.KBox("box33", center=[0.3, 3, 0.3], scale=[0.5, 0.5, 0.5], color=[1, 0, 0, 1])
    scene.add(box33)
    box44 = flex.KBox("box44", center=[1, 3, 1], scale=[1., 1., 1.], color=[0.3, 0.3, 0, 1])
    scene.add(box44)

    sim.reset(center=False)
    for i in tqdm.trange(60):
        q = sim.step()
        #img = sim.render()[::-1,:,[2,1,0,3]]
        #print(img.shape, img.dtype)
        #import cv2
        #cv2.imshow('x', img)
        #cv2.waitKey(0)

    x = bunny.position
    print(x)
    x[:, 1] += 10
    bunny.position = x
    for i in tqdm.trange(10000):
        q = sim.step()


def test3():
    import flex
    sim = flex.Simulator(rendering=True)
    scene = sim.get_scene()

    radius = 0.1
    restDistance = radius * 0.55
    scene.radius = radius
    scene.dynamicFriction = 0.01
    scene.viscosity = 2.0
    scene.numIterations = 4
    scene.numSubsteps = 6
    scene.vorticityConfinement = 40.0
    scene.fluidRestDistance = restDistance
    scene.solidPressure = 0.
    scene.relaxationFactor = 0.0
    scene.cohesion = 0.02
    scene.collisionDistance = 0.01
    scene.numPlanes = 5

    scene.camPos = [0, 3, 2]
    scene.camAngle = [0, -np.pi/4, 0]

    ground = flex.KBox("ground", center=[0, 0, 0], scale=[2, 0.1, 2], color=[0.9, 0.9, 0.9, 1])
    scene.add(ground)

    left_ground = flex.KBox("left", center = [-0.75, 0.3, 0.5], scale=[0.5, 0.6, 1], color=[0.9, 0, 0, 1])
    scene.add(left_ground)

    right_ground = flex.KBox("left", center = [0.75, 0.3, 0.5], scale=[0.5, 0.6, 1], color=[0.9, 0, 0, 1])
    scene.add(right_ground)

    door = flex.KBox("door", center = [0, 0.75, 0], scale=[2, 1.5, 0.1], color=[0.0, 0.8, 0.7, 1])
    scene.add(door)

    rr = 0.06
    fluid = flex.Fluid(
        "fluid", [-0.99, 0.1, -0.99], int(2/rr), int(1.5/rr), int(0.9/rr), 0.055, [1, 0.425, 0.55, 1.], 1.
    )
    scene.add(fluid)

    """
    bunny = pyflex.Shape(
        "buny",
        "/home/hza/fluid/PyFlex/data/bunny.ply", [0.2, 0.8, 0.4], [0.3, 0.3, 0.3], 0, [0, 0, 0, 0], 1, spacing=spacing
    )
    scene.add(bunny)
    """
    spacing = 0.05
    sphere = flex.Shape("sphere", "/home/hza/fluid/PyFlex/data/sphere.ply", [0.7, 0.8, 0.5], [0.3, 0.3, 0.3], 0, [0, 0, 0, 0], 0.4, spacing=spacing)
    scene.add(sphere)

    boat = flex.Shape("boat", "/home/hza/fluid/PyFlex/data/box.ply", [-0.5, 0.12, 0.3], [1., 0.2, 0.6], 0, [1, 1, 0, 0], 10, spacing=spacing)
    scene.add(boat)

    sim.reset(center=False)
    agent = Agent(sim)
    agent.speed = 10
    agent.add(sphere)
    #exit(0)

    trigger = 0
    while True:
        sim.render()
        agent.update()
        sim.step()
        if not trigger:
            if sphere.position.mean(axis=0)[2] < 0.2:
                door.moveTime = 2
                door.velocity = [0, 0.1, 0]
                trigger = 1


def test4():
    # show the bugs of flex
    import flex
    sim = flex.Simulator(rendering=True)
    scene = sim.get_scene()

    radius = 0.1
    restDistance = radius * 0.55
    scene.radius = radius
    scene.dynamicFriction = 0.01
    scene.viscosity = 2.0
    scene.numIterations = 4
    scene.vorticityConfinement = 40.0
    scene.fluidRestDistance = restDistance
    scene.solidPressure = 0.
    scene.relaxationFactor = 0.0
    scene.cohesion = 0.02
    scene.collisionDistance = 0.01
    scene.numPlanes = 5

    scene.camPos = [0, 3, 2]
    scene.camAngle = [0, -np.pi/4, 0]

    ground = flex.KBox("ground", center=[0, 0, 0], scale=[2, 0.1, 2], color=[0.9, 0.9, 0.9, 1])
    scene.add(ground)

    spacing = 0.05
    cup = flex.Shape("sphere", "/home/hza/fluid/PyFlex/data/bunny.ply", [0, 0, 0.], [1, 1, 1], 0, [0, 0, 0, 0], 0.4, spacing=spacing, rigid=True, rigidStiffness=0.1)
    scene.add(cup)

    #fluid = pyflex.Fluid("water", [0.3, .3, 0.3], 4, 10, 4, 0.05, invMass=1., jitter=0)

    #scene.add(fluid)

    #agent = sim.get_agent()
    #agent.add(cup)

    scene.drawMesh=False
    scene.drawPoints=True
    scene.drawFluids = False

    sim.reset(center=False)

    #img = sim.render()[::-1, :,[2, 1, 0]]
    #import cv2
    #cv2.imshow('x', img)
    #cv2.waitKey(0)

    pos = cup.position
    #print(np.dot(pos[:,:3], [[0, 1, 0], [1, 0, 0], [0, 0, -1]])[:10])
    #pos = np.dot(pos[:,:3], [[0, 1, 0], [1, 0, 0], [0, 0, -1]])[:10]
    #cup.position = pos
    #ang = np.pi * 0.99 # mysterious behavior if I set this to np.pi
    ang = np.pi
    #cup.rotate([[np.cos(ang), np.sin(ang), 0], [-np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
    #print(pos[:10])
    pos[:, 0] = 1 - pos[:, 0]
    pos[:, 1] = 2 - pos[:, 1]
    #pos[:, 1] = - pos[:, 1]
    #pos = cup.position
    #pos[:, 1] += 2
    sim.positions = pos

    while True:
        #print(cup.position[0, 1])
        #img = sim.render('rgb_array')[::-1, :,[2, 1, 0]]
        img = sim.render('human')
        # import cv2
        #cv2.imshow('x', img)
        #cv2.waitKey(1)
        #input()
        sim.step()


def test5():
    import flex
    sim = flex.Simulator(rendering=True)
    scene = sim.get_scene()

    radius = 0.1
    restDistance = radius * 0.55
    scene.radius = radius
    scene.dynamicFriction = 0.01
    scene.viscosity = 2.0
    scene.numIterations = 4
    scene.vorticityConfinement = 40.0
    scene.fluidRestDistance = restDistance
    scene.solidPressure = 0.
    scene.relaxationFactor = 0.0
    scene.cohesion = 0.02
    scene.collisionDistance = 0.01
    scene.numPlanes = 5

    scene.camPos = [0, 3, 2]
    scene.camAngle = [0, -np.pi/4, 0]
    #scene.camPos = [0, -3, 0]
    #scene.camAngle = [0, np.pi/2, 0]

    ground = flex.KBox("ground", center=[0, 0, 0], scale=[2, 0.1, 2], color=[0.9, 0.9, 0.9, 1])
    scene.add(ground)

    spacing = 0.025
    cup = flex.Shape("sphere", "/home/hza/fluid/PyFlex/data/cup.ply", [0, 0, 0.], [1, 1, 1], 0, [0, 0, 0, 0], 0.4, spacing=spacing)
    scene.add(cup)


    fluid = flex.Fluid("water", [0.2, .3, 0.2], 6, 30, 6, radius*0.55, invMass=1., jitter=0)
    #fluid = pyflex.Fluid("water", [0.2, .3, 0.2], 20, 100, 20, radius*0.55, invMass=1., jitter=0)
    scene.add(fluid)

    agent = Agent(sim)
    agent.add(cup)
    agent.speed = 10

    #scene.drawMesh=False
    #scene.drawPoints=True
    #scene.drawFluids = False
    #scene.wireframe = True
    scene.numSubsteps = 4
    scene.numIterations = 6

    sim.reset(center=False)

    #while True:
    while True:
        img = sim.render(mode='human')
        agent.update()
        sim.step()
    print(fluid.position.mean(axis=0))


if __name__ == '__main__':
    #test2()
    #test1()
    test4()
    #test5()
    #test3()
