import pyscaner

api = pyscaner.API("EASYSCANER", "CONFIG", 10)

def running():
    vhl = api.get_vehicle(0)

    sign = vhl.get_nearest_road_sign_in_lane()
    tflight = vhl.get_nearest_traffic_light_in_lane()
    
    if sign:
        api.display_message(f"Nearest road sign: {sign.name} - type: {sign.type}, distance: {sign.distance}", "LEFT1")
        print(repr(sign.type))
    elif tflight:
        api.display_message(f"Nearest traffic light: {tflight.name} - color: {tflight.color}, distance: {tflight.distance}", "LEFT1")
    else:
        api.display_message("", "LEFT1")

api.while_running(running)
api.run()