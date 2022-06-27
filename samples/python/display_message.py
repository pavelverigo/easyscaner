# Import module
import pyscaner

# Create API object (process_name, config_name, frequency in hz)
api = pyscaner.API("EASYSCANER", "DEFAULT_1.9", 1)

def init():
  pass

# Define the loop function
def running():
  v0 = api.get_vehicle(0) # Get vehicle with ID 0
  time = api.get_time()

  for i in range(1, 8):
    api.display_message(f"Hello Right {i}", f"RIGHT{i}")
    api.display_message(f"Hello Left {i}", f"LEFT{i}")

  time = api.get_time()
  api.log(f"Time: {time}")

# Set callbacks
api.while_running(running)
api.on_init(init)

api.run()
