# Import module
import pyscaner

# Create API object
api = pyscaner.API("EASYSCANER", "DEFAULT_1.9", 10)
tick = 0

# Define the initialisation function
def init():
    for v in api.get_vehicles():
        v.try_pull_out()
    print("All vehicles were asked to pull out")

# Set the initialisation function
api.on_init(init)

# Define the loop function
def running():
    global tick
    tick += 1
    print(tick)

    if tick == 5:
        for v in api.get_vehicles():
            v.try_filter_in()
        print("All vehicles were asked to filter in")
        api.while_running(None)

# Set the loop function
api.while_running(running)

api.run()
