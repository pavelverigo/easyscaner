#define _USE_MATH_DEFINES

// Include EasyScaner files
#include <math.h>

#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

// modified idm plus function
float idm_plus(Vehicle* vhl, Scaner* sc, std::vector<Vehicle*> lanes, PhantomVehicle& phantom, bool usePhantom) {
  // Get the vehicle in front
  Vehicle* in_front = sc->get_vehicle_in_front(vhl, lanes, phantom, usePhantom);
  auto a = 4;
  auto v_0 = 80 / 3.6;
  auto s_0 = 5;
  auto T = 4;
  auto b = 7;
  // If there's a vehicle in front, apply IDM+
  if (in_front && !vhl->in_intersection()) {
    double v = vhl->get_speed();
    double dv = in_front->get_speed() - v;
    double s = sc->get_headway(vhl, in_front);
    double s_star = s_0 + v * T + v * dv / (2 * std::sqrt(a * b));

    float accel = a * std::min(1 - std::pow(v / v_0, 4), 1 - std::pow(s_star / s, 2));

    return accel;
  }
  // Else, set acceleration to 0
  else {
    return 0;
  }
}

// Custom function for applying MOBIL model
void mobil(Vehicle* c, Scaner* sc) {
  // Determine which lanes can be switched to.
  std::vector<int> lanesToCheck;
  const short ownLane = c->get_lane();
  auto numOfLanes = c->get_num_of_lanes();
  if (numOfLanes <= 0) {
    return;
  }
  if (ownLane + 1 <= numOfLanes) {  // left
    lanesToCheck.push_back(ownLane + 1);
  }
  if (ownLane - 1 >= 1) {  // right
    lanesToCheck.push_back(ownLane - 1);
  }

  // Setup best acceleration.
  float bestAcc = -std::numeric_limits<float>::infinity();
  int bestLane = ownLane;
  for (int lane : lanesToCheck) {
    PhantomVehicle nullPhantom;
    // Get vehicles behind the car, and behind the car in the lane it's checking to move to, o and n respectively.
    Vehicle* o = sc->get_vehicle_in_front(c, sc->lanes[ownLane], nullPhantom, false);
    Vehicle* n = sc->get_vehicle_in_front(c, sc->lanes[lane], nullPhantom, false);

    // Get current accelerations.
    float c_acc_before = idm_plus(c, sc, sc->lanes[ownLane], nullPhantom, false);
    float o_acc_before = idm_plus(o, sc, sc->lanes[ownLane], nullPhantom, false);
    float n_acc_before = idm_plus(n, sc, sc->lanes[lane], nullPhantom, false);

    // update best acceleration.
    if (c_acc_before + o_acc_before + n_acc_before > bestAcc) {
      bestLane = ownLane;
      bestAcc = c_acc_before + o_acc_before + n_acc_before;
    }

    // Set the distance to mock to the other lane.
    auto distToMove = c->get_laneWidth() - c->get_lane_gap();
    auto dy = distToMove * std::sin(M_PI_2 - c->get_direction());
    auto dx = distToMove * std::cos(M_PI_2 - c->get_direction());
    std::array<double, 3> pos;
    auto origPos = c->get_pos();

    // Set mock position.
    if (lane = ownLane + 1) {  // right
      pos = std::array<double, 3>{origPos[0] - dx, origPos[1] - dy, origPos[2]};
    } else {  // left
      pos = std::array<double, 3>{origPos[0] + dx, origPos[1] + dy, origPos[2]};
    }

    // Set Mock vehicle.
    PhantomVehicle cPhantom(c);
    cPhantom.set_pos(pos[0], pos[1], pos[2]);

    // Set mock lanes.
    auto cnLane = sc->lanes[lane];
    cnLane.push_back(c);
    auto oLane = sc->lanes[ownLane];
    std::remove(oLane.begin(), oLane.end(), c);

    // Calculate the acceleration after the mock.
    float c_acc_after = idm_plus(c, sc, cnLane, cPhantom, true);
    float o_acc_after = idm_plus(o, sc, oLane, cPhantom, true);
    float n_acc_after = idm_plus(n, sc, cnLane, cPhantom, true);

    // Update best standings.
    if (c_acc_after + o_acc_after + n_acc_after > bestAcc) {
      bestLane = lane;
      bestAcc = c_acc_after + o_acc_after + n_acc_after;
    }
  }

  // Change lane accordingly.
  if (bestLane == ownLane + 1) {
    // bestLane = left

    c->try_pull_out();
  } else if (bestLane == ownLane - 1) {
    // bestLane = right
    c->try_filter_in();
  }
}

// Main function; setup goes here
int main(int argc, char* argv[]) {
  // Create Scaner object
  Scaner sc = Scaner("EASYSCANER", "CONFIG", 1);

  // Set the loop function
  sc.while_running([&]() {
    // Apply the IDM+ model to each vehicle
    for (auto vhl : sc.get_road_vehicles()) mobil(vhl, &sc);
  });

  // Run the tool
  sc.run();
}
