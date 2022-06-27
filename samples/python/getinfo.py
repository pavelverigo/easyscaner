# Import module
import pyscaner

# Create API object (process_name, config_name, frequency in hz)
api = pyscaner.API("EASYSCANER", "CONFIG", 1)

def init():
  api.log(f"Hello world!")

# Define the loop function
def running():
  v0 = api.get_vehicle(0) # Get vehicle with ID 0

  api.log(f"Vehicle with id 0 speed (km/h): {3.6 * v0.get_speed()}")

  api.log(f"Is road vehicle id 0: {api.is_road_vehicle(v0)}")

  api.log(f"Vehicle count: {len(api.get_road_vehicles())}")

  for vhl in api.get_road_vehicles():

    api.log(f"Vehicle {vhl.get_id()} (type {vhl.get_type()}):")

    api.log(f"Position: {vhl.get_pos()}")
    api.log(f"Speed: {vhl.get_speed()}m/s")
    api.log(f"Acceleration: {vhl.get_accel()}m/s/s")
    api.log(f"Direction: {vhl.get_direction()} (Wheel angle: {vhl.get_wheel_angle()})")

    api.log(f"This vehicle is in road {vhl.get_road()}, lane {vhl.get_lane()}")
    if vhl.in_intersection():
        api.log(f"At intersection {vhl.get_intersection()}")
    in_front = api.get_vehicle_in_front(vhl)
    if in_front:
        api.log(f"{api.get_headway(vhl, in_front)}m behind vehicle {in_front.get_id()}")

    sign = vhl.get_nearest_road_sign_in_lane()
    if sign:
        api.log(f"The road sign {sign.name} (type: {sign.type}, value: {sign.value}) has been detected {sign.distance}m from this vehicle")
    
    tflight = vhl.get_nearest_traffic_light_in_lane()
    if tflight:
        api.log(f"The traffic light {tflight.name} (color: {tflight.color}, flashing: {tflight.flashing}) has been detected {tflight.distance}m from this vehicle")

    api.log("======================================================================")

# Set callbacks
api.while_running(running)
api.on_init(init)

api.run()
