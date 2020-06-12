#Written by Henry M. Clever. November 15, 2018.
import tqdm

import pyflex as flexbind
from time import time
from random import random
import numpy as np



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
