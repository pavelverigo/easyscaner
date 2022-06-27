// Allow printing to console
#include <iostream>

// Include EasyScaner files
#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// Custom function for printing info
void print_vehicle_info(Vehicle* vhl, Scaner* sc) {
  // If the vehicle's engine is running
  if (vhl->get_engine_status()) {
    // Print its position
    std::array<double, 3> pos = vhl->get_pos();
    std::cout << "Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

    // Print its speed, acceleration, and wheel angle
    std::cout << "Speed: " << vhl->get_speed() << "m/s" << std::endl;
    std::cout << "Acceleration: " << vhl->get_accel() << "m/s/s" << std::endl;
    std::cout << "Wheel angle: " << vhl->get_wheel_angle() << " radians" << std::endl;

    // Print in what lane the vehicle is in, and the distance to the next vehicle in this lane
    std::cout << "This vehicle is in road " << vhl->get_road() << ", lane " << vhl->get_lane() << std::endl;
    if (vhl->in_intersection()) std::cout << "At intersection " << vhl->get_intersection() << std::endl;
    Vehicle* in_front = sc->get_vehicle_in_front(vhl);
    if (in_front) std::cout << sc->get_headway(vhl, in_front) << "m behind vehicle " << in_front->get_id() << std::endl;

    // Print information about the nearest road sign to the vehicle, if one was detected
    if (road_sign* sign = vhl->get_nearest_road_sign_in_lane())
      std::cout << "The road sign " << sign->name << " (type: " << sign->type << ") has been detected "
                << sign->distance << "m from this vehicle" << std::endl;

    // Print information about the nearest traffic light to the vehicle, if one was detected
    if (traffic_light* tflight = vhl->get_nearest_traffic_light_in_lane())
      std::cout << "The traffic light " << tflight->name << " (color: " << tflight->color << ") has been detected "
                << tflight->distance << "m from this vehicle" << std::endl;

    // Print indicator status if applicable
    indicator ind = vhl->get_indicator();
    if (ind == LEFT)
      std::cout << "Signalling left" << std::endl;
    else if (ind == RIGHT)
      std::cout << "Signalling right" << std::endl;
    else if (ind != INDICATOR_OFF)
      std::cout << "Undefined indicator position!" << std::endl;

    // Print if the horn is being sound
    if (vhl->get_horn()) std::cout << "Horn is on" << std::endl;

    // Print cruise control mode if applicable
    cruise_control cc = vhl->get_cruise_control();
    if (cc.mode == CRUISE_CONTROL)
      std::cout << "Cruise control is on with target speed: " << cc.target << "m/s" << std::endl;
    else if (cc.mode == SPEED_LIMITER)
      std::cout << "The speed limiter is on with top speed: " << cc.target << "m/s" << std::endl;
    else if (cc.mode != CRUISE_CONTROL_OFF)
      std::cout << "Undefined cruise control mode!" << std::endl;

    // Print lights status if applicable (note that the indicator lights are NOT the same as the indicator status above)
    lights l = vhl->get_lights();
    if (l.left_indicator) std::cout << "The left indicator light is on" << std::endl;
    if (l.right_indicator) std::cout << "The right indicator light is on" << std::endl;
    if (l.stop_lights) std::cout << "The stop lights are on" << std::endl;
  }

  // If the vehicle's engine is not running
  else {
    std::cout << "This vehicle's engine is not running" << std::endl;
  }

  // Print horizontal divider
  std::cout << "======================================================================" << std::endl;
}

// Main function; setup goes here
int main(int argc, char* argv[]) {
  // Create Scaner object
  Scaner sc = Scaner("EASYSCANER", "CONFIG", 1);

  // Set the initialisation function
  sc.on_init([&]() { sc.log("Hello World! Vehicle information will be printed to the console."); });

  // Set the loop function
  sc.while_running([&]() {
    // go though all vehicles
    for (auto vhl : sc.get_road_vehicles()) {
      std::cout << "Hello World! We are looking at the vehicle with ID: " << vhl->get_id() << " (type "
                << vhl->get_type() << ")" << std::endl;
      print_vehicle_info(vhl, &sc);
    }
  });

  // Set the vehicle_added listener function
  sc.on_vehicle_added([&](Vehicle* vhl) {
    std::cout << "A new vehicle has been added with ID: " << vhl->get_id() << std::endl;
    print_vehicle_info(vhl, &sc);
  });

  // Run the tool
  sc.run();
}
