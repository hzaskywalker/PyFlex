#Written by Henry M. Clever. November 15, 2018.
from time import time
import tqdm
from random import random
import numpy as np



def test1():
    import pyflex as flexbind
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
    import pyflex
    sim = pyflex.Simulator(rendering=True)
    scene = sim.get_scene()
    s = 0.1 * 0.5
    bunny = pyflex.Shape(
        "bunny",
        "/home/hza/fluid/PyFlex/data/box.ply", [0.2, 0.2, 0.4], [1., 1., 1.], 0, [0, 0, 0, 0], 0.5, spacing=s
    )
    sphere = pyflex.Shape(
        "sphere",
        "/home/hza/fluid/PyFlex/data/sphere.ply", [0.2, 0.8, 0.4], [1., 1., 1.], 0, [0, 0, 0, 0], 10, spacing=s
    )
    box = pyflex.Shape(
        "sphere",
        "/home/hza/fluid/PyFlex/data/bunny.ply", [0.2, 0.8, 0.4], [0.3, 0.3, 0.3], 0, [0, 0, 0, 0], 1, spacing=s
    )
    scene.add(bunny)
    scene.add(sphere)
    scene.add(box)

    fluid = pyflex.Fluid(
        "fluid", [0.1, 2, 0.1], 40, 10, 40, 0.055, [1, 0.425, 0.55, 1.], 1.
    )

    fluid2 = pyflex.Fluid(
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

    box33 = pyflex.KBox("box33", center=[0.3, 3, 0.3], scale=[0.5, 0.5, 0.5], color=[1, 0, 0, 1])
    scene.add(box33)
    box44 = pyflex.KBox("box44", center=[1, 3, 1], scale=[1., 1., 1.], color=[0.3, 0.3, 0, 1])
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


if __name__ == '__main__':
    test2()
    #test1()