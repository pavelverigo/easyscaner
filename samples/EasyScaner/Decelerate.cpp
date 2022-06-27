// Allow printing to console
#include <iostream>

// Include EasyScaner files
#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// Main function; setup goes here
int main(int argc, char* argv[]) {
  // Create Scaner object
  Scaner sc = Scaner("EASYSCANER", "CONFIG", 10);

  // Set the initialisation function
  sc.on_init([&]() {
    for (auto vhl : sc.get_road_vehicles()) {
      vhl->set_accel(5.0f);  // Accelerate at 5 m/s^2
    }
    std::cout << "Accelerating all vehicles" << std::endl;
  });

  // Set the loop function
  sc.while_running([&]() {
    for (auto vhl : sc.get_road_vehicles()) {
      // Get speed and acceleration command
      float speed = vhl->get_speed();
      numerical_command accel_command = vhl->get_accel_command();

      // If an acceleration command is active
      if (accel_command.active) {
        // If speed is above 20 m/s and acceleration is above 0 m/s
        if (speed > 20.0f && accel_command.value > 0.0f) {
          // Start decelerating at a rate of 2 m/s^2
          vhl->set_accel(-2.0f);
          std::cout << "Decelerating vehicle " << vhl->get_id() << std::endl;

        }

        // Else if speed is 0 m/s and acceleration is below 0 m/s (a.k.a. decelerating)
        else if (speed == 0.0f && accel_command.value < 0.0f) {
          // Set acceleration to 0 m/s^2
          vhl->reset_accel();
          std::cout << "Vehicle " << vhl->get_id() << " has come to a halt, resetting acceleration" << std::endl;
        }
      }
    }
  });

  // Run the tool
  sc.run();
}
