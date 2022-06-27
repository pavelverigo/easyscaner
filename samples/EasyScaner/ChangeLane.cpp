// Include EasyScaner files
#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// Main function; setup goes here
int main(int argc, char* argv[]) {
  // Create Scaner object
  Scaner sc = Scaner("EASYSCANER", "CONFIG", 1);
  int tick;

  // Set the initialisation function
  sc.on_init([&]() {
    tick = 0;
    for (auto vhl : sc.get_road_vehicles()) vhl->try_pull_out();  // Ask each vehicle to move a lane to the left
    sc.log("All vehicles have been ordered to move to the lane on their left.");
  });

  // Set the loop function
  sc.while_running([&]() {
    if (tick++ == 5) {                                               // If 5 SCANeR loops have passed
      for (auto vhl : sc.get_road_vehicles()) vhl->try_filter_in();  // Ask each vehicle to move a lane to the right
      sc.log("All vehicles have been ordered to move to the lane on their right.");
      sc.while_running([&]() {});  // Stop the loop from running by setting an empty function as callback
    }
  });

  // Run the tool
  sc.run();
}
