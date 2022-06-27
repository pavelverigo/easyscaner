# Import module
import pyscaner
import math

# Create API object
api = pyscaner.API("EASYSCANER", "DEFAULT_1.9", 10)

# Define IDM+ model as a function
def idm_plus(vhl, a, v_0, s_0, T, b):
    in_front = api.get_vehicle_in_front(vhl)

    if(in_front and not vhl.in_intersection()):
        v = vhl.get_speed()
        dv = in_front.get_speed() - v
        s = api.get_headway(vhl, in_front)
        s_star = s_0 + (v * T) + (v * dv) / (2 * math.sqrt(a * b))

        accel = a * min(1 - (v / v_0) ** 4, 1 - (s_star / s) ** 2)

        vhl.set_accel(accel)
        api.log(str(vhl.get_id()) + " is following " + str(in_front.get_id()) + " with acceleration " + str(accel))

    else:
        vhl.reset_accel()

# Define the loop function
def running():
    for v in api.get_vehicles():
        idm_plus(v, 4, 80 / 3.6, 5, 4, 7)

# Set the loop function
api.while_running(running)

api.run()
