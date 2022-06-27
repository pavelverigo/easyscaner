// Include EasyScaner files
#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// Custom function for applying IDM+ model
void idm_plus(Vehicle* vhl, Scaner* sc, double a, double v_0, double s_0, double T, double b) {
  // Get the vehicle in front
  Vehicle* in_front = sc->get_vehicle_in_front(vhl);

  // If there's a vehicle in front, apply IDM+
  if (in_front && !vhl->in_intersection()) {
    double s = sc->get_headway(vhl, in_front);

    // Ignore vehicles which are too far
    if (s > 50) {
      vhl->reset_accel();
      return;
    }

    // Check if there is a road sign that's closer than the car in front, and if so, relinquish control of acceleration.
    road_sign* nearest_sign = vhl->get_nearest_road_sign_in_lane();
    if (nearest_sign && nearest_sign->distance < s && (nearest_sign->type == YIELD || nearest_sign->type == STOP)) {
      sc->log("Vehicle " + std::to_string(vhl->get_id()) + " is approaching the road sign " + nearest_sign->name);
      vhl->reset_accel();
      return;
    }

    // Check if there is a non-green traffic light that's closer than the car in front, and if so, relinquish control of
    // acceleration.
    traffic_light* nearest_tflight = vhl->get_nearest_traffic_light_in_lane();
    if (nearest_tflight && nearest_tflight->distance < s && nearest_tflight->color != OFF &&
        nearest_tflight->color != GREEN) {
      sc->log("Vehicle " + std::to_string(vhl->get_id()) + " is approaching the traffic light " +
              nearest_tflight->name);
      vhl->reset_accel();
      return;
    }

    double v = vhl->get_speed();
    double dv = in_front->get_speed() - v;
    double s_star = s_0 + v * T + v * dv / (2 * std::sqrt(a * b));

    float accel = a * std::min(1 - std::pow(v / v_0, 4), 1 - std::pow(s_star / s, 2));

    vhl->set_accel(accel);
    sc->log("Vehicle " + std::to_string(vhl->get_id()) + " is following vehicle " + std::to_string(in_front->get_id()) +
            " with acceleration " + std::to_string(accel));
  }
  // Else, set acceleration to 0
  else {
    vhl->reset_accel();
  }
}

// Main function; setup goes here
int main(int argc, char* argv[]) {
  // Create Scaner object
  Scaner sc = Scaner("EASYSCANER", "CONFIG", 10);

  // Set the loop function
  sc.while_running([&]() {
    // Apply the IDM+ model to each vehicle
    for (auto vhl : sc.get_road_vehicles()) idm_plus(vhl, &sc, 4, 80 / 3.6, 5, 4, 7);
  });

  // Run the tool
  sc.set_sensor_wait_mode(
      FREEZE_VEHICLES);  // All vehicles are stopped until the sensors start sending data. For this reason, the program
                         // will not work on scenarios without any sensors (the vehicles will remain frozen).
  sc.run();
}
