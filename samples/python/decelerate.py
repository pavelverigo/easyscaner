# Import module
import pyscaner

# Create API object
api = pyscaner.API("EASYSCANER", "DEFAULT_1.9", 10)

# Define the initialisation function
def init():
    for v in api.get_vehicles():
        v.set_accel(5.0)
    print("All vehicles were asked to accelerate")

# Set the initialisation function
api.on_init(init)

# Define the loop function
def running():
    for v in api.get_vehicles():
        speed = v.get_speed()
        accel_command = v.get_accel_command()

        if(accel_command.active):
            if(speed > 20 and accel_command.value > 0):
                v.set_accel(-2.0)
                print("Started decelerating", v.get_id())

            elif(speed == 0 and accel_command.value < 0):
                v.reset_accel()
                print(v.get_id(), "has come to a stop")

# Set the loop function
api.while_running(running)

api.run()
