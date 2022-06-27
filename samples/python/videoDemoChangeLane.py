# Import module
import pyscaner
import time

# Create API object
api = pyscaner.API("EASYSCANER", "CONFIG", 10)

# Define the loop function
def running():
    v = api.get_vehicle(0)
    api.log("Lane: " + str(v.get_lane()))
    api.log("Number of Lanes detected: " + str(v.get_num_of_lanes()))

# Set the loop function
api.while_running(running)

api.run()
