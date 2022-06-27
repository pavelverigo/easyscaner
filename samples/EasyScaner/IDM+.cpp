// Include EasyScaner files
#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// Custom function for applying IDM+ model
void idm_plus(Vehicle* vhl, Scaner* sc, double a, double v_0, double s_0, double T, double b) {
  // Get the vehicle in front
  Vehicle* in_front = sc->get_vehicle_in_front(vhl);

  // If there's a vehicle in front, apply IDM+
  if (in_front && !vhl->in_intersection()) {
    double v = vhl->get_speed();
    double dv = in_front->get_speed() - v;
    double s = sc->get_headway(vhl, in_front);

    // Ignore vehicles which are too far
    if (s > 50) {
      vhl->reset_accel();
      return;
    }

    double s_star = s_0 + v * T + v * dv / (2 * std::sqrt(a * b));
    float accel = a * std::min(1 - std::pow(v / v_0, 4), 1 - std::pow(s_star / s, 2));

    vhl->set_accel(accel);
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
  sc.run();
}
