#include "Scaner.hpp"

#include <cmath>
#include <iostream>

#include "ScanerAPI.hpp"

#define target(i, atr) ("targetsArray[" + std::to_string(i) + "]/" + atr).c_str()
#define in_same_lane(i)                                                                                               \
  Com_getShortData(interface, target(i, "laneId1")) == 0 || Com_getShortData(interface, target(i, "laneId2")) == 0 || \
      Com_getShortData(interface, target(i, "laneId3")) == 0 ||                                                       \
      Com_getShortData(interface, target(i, "laneId4")) == 0 || Com_getShortData(interface, target(i, "laneId5")) == 0

/**
 * `Scaner` constructor.
 *
 * @param process The name of the SCANeR process to connect to. This should be one of the modules under `SIMULATOR
 * STATUS`.
 * @param config The name of the SCANeR configuration. It's in the SCANeR title window. `SCANeR studio - [config name] -
 * (...)
 * @param frequency How often per second the simulation should run
 */
Scaner::Scaner(std::string process, std::string config, float frequency) {
  Process_InitParams(process.c_str(), config.c_str(), frequency);
  wait_mode = NO_WAIT;
  benchmark_counter = 0;
  benchmark_total = 0;
}

/**
 * Logs a message to the SCANeR console.
 *
 * @param message The message to print
 */
void Scaner::log(std::string message) { Process_Output(message.c_str()); }

/**
 * Get process time.
 */
double Scaner::get_time() { return Process_GetTime(); }

/**
 * Logs a message to the SCANeR console.
 *
 * @param message The message to display
 * @param position String representing posistion on screen, should be in form 'RIGHTn' or 'LEFTn', where 'n' number from
 * 1 to 8
 */
void Scaner::display_message(std::string message, std::string position) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IINFORMATION_SENDVISUALCONSTANTTEXTMESSAGE);
  Com_setStringData(interface, "constantString", message.c_str());
  Com_setStringData(interface, "displayAreaName", position.c_str());
  Com_updateOutputDataInterface(interface);
  Com_deleteOutputDataInterface(interface);
}

void Scaner::set_road_vehicle_types(std::vector<VType> types) {
  road_vehicle_types = types;

  road_vehicles.clear();
  road_vehicles_vec.clear();
  road_vehicle_ids_vec.clear();

  for (auto id_vhl : vehicles) {
    if (is_road_vehicle(id_vhl.second)) {
      road_vehicles.insert(id_vhl);
      road_vehicles_vec.push_back(id_vhl.second);
      road_vehicle_ids_vec.push_back(id_vhl.first);
    }
  }
}

bool Scaner::is_road_vehicle(Vehicle* vhl) { return is_road_vehicle(vhl->get_type()); }

/**
 * Gets a vehicle by ID.
 *
 * @param id The ID of the vehicle to retrieve.
 * @return The vehicle
 */
Vehicle* Scaner::get_vehicle(int id) { return vehicles[id]; }

/**
 * Gets all vehicles.
 *
 * @return All vehicles
 */
const std::vector<Vehicle*> Scaner::get_vehicles() { return vehicles_vec; }

const std::vector<Vehicle*> Scaner::get_road_vehicles() { return road_vehicles_vec; }

/**
 * Gets all vehicle IDs.
 *
 * @return All vehicle IDs
 */
const std::vector<int> Scaner::get_vehicle_ids() { return ids_vec; }

const std::vector<int> Scaner::get_road_vehicle_ids() { return road_vehicle_ids_vec; }

/**
 * Gets all vehicles in the lane.
 *
 * @param lane_id The ID of the lane to get the vehicles from
 * @return All vehicles in the given lane
 */
const std::vector<Vehicle*> Scaner::get_vehicles_in_lane(short lane_id) { return lanes[lane_id]; }

/**
 * Gets the vehicle in front of this vehicle.
 *
 * @param vhl The vehicle that follows the target
 * @return null if no vehicle was found, the vehicle otherwise
 */
Vehicle* Scaner::get_vehicle_in_front(Vehicle* vhl) {
  if (vhl == NULL) {
    return NULL;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  auto speed = vhl->get_speed_vector();
  double dir = vhl->get_direction();
  std::array<double, 3> speed_v = {std::cos(dir) * speed[0], std::sin(dir) * speed[0], speed[2]};
  auto pos = vhl->get_pos();
  double d = speed_v[0] * pos[0] + speed_v[1] * pos[1] + speed_v[2] * pos[2];

  Vehicle* ret = NULL;
  for (Vehicle* other : lanes[vhl->get_lane()]) {
    if (other == vhl) continue;

    auto other_pos = other->get_pos();
    auto in_front = speed_v[0] * other_pos[0] + speed_v[1] * other_pos[1] + speed_v[2] * other_pos[2] - d;

    double dx = pos[0] - other_pos[0];
    double dy = pos[1] - other_pos[1];
    double dz = pos[2] - other_pos[2];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (in_front > 0 && dist < min_dist) {
      min_dist = dist;
      ret = other;
    }
  }

  return ret;
}

/**
 * Gets the vehicle in front of the vehicle with the given ID.
 *
 * @param id The ID of the vehicle that follows the target
 * @return null if no vehicle was found, the vehicle otherwise
 */
Vehicle* Scaner::get_vehicle_in_front(int id) { return get_vehicle_in_front(get_vehicle(id)); }

Vehicle* Scaner::get_vehicle_in_front(Vehicle* vhl, std::vector<Vehicle*> vehicles, PhantomVehicle& phantom,
                                      bool usePhantom, bool behind) {
  if (vhl == NULL) {
    return NULL;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  std::array<float, 3> speed;
  std::array<double, 3> pos;
  double dir;

  if (usePhantom && vhl->get_id() == phantom.get_id()) {
    speed = phantom.get_speed_vector();
    pos = phantom.get_pos();
    dir = phantom.get_direction();
  } else {
    speed = vhl->get_speed_vector();
    pos = vhl->get_pos();
    dir = vhl->get_direction();
  }

  std::array<double, 3> speed_v = {std::cos(dir) * speed[0], std::sin(dir) * speed[0], speed[2]};
  double d = speed_v[0] * pos[0] + speed_v[1] * pos[1] + speed_v[2] * pos[2];

  Vehicle* ret = NULL;
  for (Vehicle* other : vehicles) {
    if (other == vhl) continue;

    std::array<double, 3> other_pos;
    if (usePhantom && other->get_id() == phantom.get_id()) {
      other_pos = phantom.get_pos();
    } else {
      other_pos = other->get_pos();
    }

    auto in_front = speed_v[0] * other_pos[0] + speed_v[1] * other_pos[1] + speed_v[2] * other_pos[2] - d;

    double dx = pos[0] - other_pos[0];
    double dy = pos[1] - other_pos[1];
    double dz = pos[2] - other_pos[2];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < min_dist) {
      if ((behind && in_front < 0) || (!behind && in_front > 0)) {
        min_dist = dist;
        ret = other;
      }
    }
  }

  return ret;
}

/**
 * Gets the headway between two vehicles.
 * Both vehicles should be in the same lane.
 *
 * @param v1 The first vehicle
 * @param v2 The second vehicle
 * @return The headway
 */
double Scaner::get_headway(Vehicle* v1, Vehicle* v2, bool behind) {
  auto p1 = v1->get_pos();
  auto p2 = v2->get_pos();
  double dx = p1[0] - p2[0];
  double dy = p1[1] - p2[1];
  double dz = p1[2] - p2[2];
  double carLength;
  if (!behind) {
    carLength = v1->get_length() - v1->get_rear_overhang() + v2->get_rear_overhang();
  } else {
    carLength = v1->get_rear_overhang() + v2->get_length() - v2->get_rear_overhang();
  }
  return std::sqrt(dx * dx + dy * dy + dz * dz) - carLength;
}

/**
 * Sets the function that is called each time the SCANeR simulation starts a new loop.
 *
 * @param f The function
 */
void Scaner::while_running(const std::function<void(void)>& f) { running_callback = f; }

/**
 * Sets the function that is called when the simulation starts its first loop.
 *
 * @param f The function
 */
void Scaner::on_init(const std::function<void(void)>& f) { first_callback = f; }

/**
 * Sets the function that is called when a vehicle is added dynamically.
 *
 * @param f The function
 */
void Scaner::on_vehicle_added(const std::function<void(Vehicle*)>& f) { vehicle_added_callback = f; }

void Scaner::set_sensor_wait_mode(sensor_wait_mode wait_mode) { this->wait_mode = wait_mode; }

/**
 * Prints the new SCANeR simulation status if it was updated.
 *
 * @param oldStatus The previous status
 * @param status The current status
 */
void debugStatusChange(APIProcessState oldStatus, APIProcessState status) {
  if (oldStatus != status) {
    switch (status) {
      case PS_DAEMON:
        std::cout << " SCANER API STATUS : DAEMON " << std::endl;
        break;
      case PS_DEAD:
        std::cout << " SCANER API STATUS : DEAD " << std::endl;
        break;
      case PS_LOADED:
        std::cout << " SCANER API STATUS : LOADED " << std::endl;
        break;
      case PS_PAUSED:
        std::cout << " SCANER API STATUS : PAUSED " << std::endl;
        break;
      case PS_READY:
        std::cout << " SCANER API STATUS : READY " << std::endl;
        break;
      case PS_RUNNING:
        std::cout << " SCANER API STATUS : RUNNING " << std::endl;
        break;
      default:
        break;
    }
  }
}

/**
 * Starts the tool. Should be started BEFORE the SCANeR simulation to make it work.
 * Note that this is likely done automatically when SCANeR is configured correctly.
 *
 * @param benchmark Whether this run should be benchmarked
 * @param runs Over how many runs the average time spent on a frame should be calculated
 */
void Scaner::run(bool benchmark, int runs) {
  // Register for events
  status = Process_GetState();
  Com_registerEvent("LOAD");
  Com_registerEvent(NETWORK_IVEHICLE_VEHICLECREATE);
  Com_registerEvent(NETWORK_IVEHICLE_VEHICLEDEL);

  Com_registerEvent(NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETS);
  Com_registerEvent(NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETSTRAFFICSTATES);

  Com_registerEvent(NETWORK_ISENSOR_SENSORMOVABLETARGETS);
  Com_registerEvent(NETWORK_ISENSOR_ROADLANESPOINTS);
  Com_registerEvent(NETWORK_ISENSOR_ROADLANESTRAFFICSTATES);

  bool should_run = wait_mode == NO_WAIT;
  bool just_initialized = true;

  // Start benchmark
  std::chrono::time_point start = std::chrono::high_resolution_clock::now();

  // While SCANeR simulation is running
  while (status != PS_DEAD) {
    // Run the process when available
    Process_Wait();
    Process_Run();

    // Update the status, log if it was changed
    APIProcessState oldStatus = status;
    status = Process_GetState();
    debugStatusChange(oldStatus, status);

    // Handle Event queue
    Event* event;
    while ((event = Com_getNextEvent())) {
      Com_validateStateEvent(event);

      switch (Com_getTypeEvent(event)) {
        case ET_message: {
          std::string msg = Com_getMessageEventDataStringId(event);

          if (msg.find(NETWORK_IVEHICLE_VEHICLECREATE) != std::string::npos) {
            int id_offset = sizeof(NETWORK_IVEHICLE_VEHICLECREATE);
            int vhl_id = stoi(msg.substr(id_offset));

            VehicleInfoStruct vinfostruct;
            // Return all vehicle info, can be called only after LOAD, which suits current case
            // The only way to find VType from docs that I found (for dynamicly created vehicle).
            bool ok = Utils_getVehicleInformations(vhl_id, &vinfostruct);
            if (ok) {
              Vehicle* new_vhl = add_vehicle(vhl_id, vinfostruct);
              if (vehicle_added_callback) vehicle_added_callback(new_vhl);
            }
          }

          else if (msg.find(NETWORK_ISENSOR_ROADLANESPOINTS) == 0) {
            DataInterface* dEventDataInterface = Com_getMessageEventDataInterface(event);
            auto vhlId = Com_getShortData(dEventDataInterface, "egoVhlId");
            short numLanes = Com_getShortData(dEventDataInterface, "roadLanesNb0m");
            vehicles[vhlId]->set_num_of_lanes(numLanes);
          }

          else if (msg.find(NETWORK_IVEHICLE_VEHICLEDEL) != std::string::npos)
            remove_vehicle(stoi(msg.substr(28)));

          else if (msg.find(NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETSTRAFFICSTATES) != std::string::npos) {
            should_run = true;

            DataInterface* interface = Com_getMessageEventDataInterface(event);
            short vhlId = Com_getShortData(interface, "egoVhlId");
            Vehicle* vhl = get_vehicle(vhlId);
            short targetCount = Com_getShortData(interface, "targetsArrayCount");

            vhl->road_signs_in_lane.clear();
            vhl->traffic_lights_in_lane.clear();
            for (int i = 0; i < targetCount; i++) {
              int id = Com_getShortData(interface, target(i, "scanerId"));

              // If the target is a traffic light
              if (Com_getShortData(interface, target(i, "type")) == SC_TRAFFICLIGHT) {
                auto search = vhl->traffic_lights.find(id);
                // If the traffic light is already known to this vehicle
                if (search != vhl->traffic_lights.end()) {
                  traffic_light* light = search->second;
                  traffic_light_color detected_color =
                      static_cast<traffic_light_color>(Com_getShortData(interface, target(i, "trafficLightState")));
                  if (detected_color != UNDEFINED_COLOR) light->color = detected_color;
                  light->flashing = Com_getCharData(interface, target(i, "trafficLightBlinking"));
                  if (light->distance != -1 && in_same_lane(i)) vhl->traffic_lights_in_lane.push_back(light);
                }
                // Otherwise create a new entry for the traffic light and add it to the map
                else {
                  traffic_light* light = new traffic_light;
                  light->id = id;
                  light->color =
                      static_cast<traffic_light_color>(Com_getShortData(interface, target(i, "trafficLightState")));
                  light->flashing = Com_getCharData(interface, target(i, "trafficLightBlinking"));
                  vhl->traffic_lights.emplace(id, light);
                }
              }

              // Otherwise all other target types are treated as road signs
              else {
                auto search = vhl->road_signs.find(id);
                // If the road sign is already known to this vehicle
                if (search != vhl->road_signs.end()) {
                  road_sign* sign = search->second;
                  sign->type = static_cast<road_sign_type>(Com_getShortData(interface, target(i, "type")));
                  sign->value = Com_getFloatData(interface, target(i, "value"));
                  if (sign->distance != -1 && in_same_lane(i)) vhl->road_signs_in_lane.push_back(sign);
                }
                // Otherwise create a new entry for the sign and add it to the map
                else {
                  road_sign* sign = new road_sign;
                  sign->id = id;
                  sign->type = static_cast<road_sign_type>(Com_getShortData(interface, target(i, "type")));
                  sign->value = Com_getFloatData(interface, target(i, "value"));
                  vhl->road_signs.emplace(id, sign);
                }
              }
            }
          }

          else if (msg.find(NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETS) != std::string::npos) {
            should_run = true;

            DataInterface* interface = Com_getMessageEventDataInterface(event);
            short vhlId = Com_getShortData(interface, "egoVhlId");
            Vehicle* vhl = get_vehicle(vhlId);
            short targetCount = Com_getShortData(interface, "targetsArrayCount");

            for (int i = 0; i < targetCount; i++) {
              int id = Com_getShortData(interface, target(i, "scanerId"));

              // If the target is a traffic light
              if (Com_getShortData(interface, target(i, "type")) == SC_TRAFFICLIGHT) {
                auto search = vhl->traffic_lights.find(id);
                // If the traffic light is already known to this vehicle
                if (search != vhl->traffic_lights.end()) {
                  traffic_light* light = search->second;
                  light->name = Com_getStringData(interface, target(i, "name"));
                  light->distance = Com_getFloatData(interface, target(i, "distanceToCollision"));
                }
                // Otherwise create a new entry for the traffic light and add it to the map
                else {
                  traffic_light* light = new traffic_light;
                  light->id = id;
                  light->name = Com_getStringData(interface, target(i, "name"));
                  light->distance = Com_getFloatData(interface, target(i, "distanceToCollision"));
                  vhl->traffic_lights.emplace(id, light);
                }
              }

              // Otherwise all other target types are treated as road signs
              else {
                auto search = vhl->road_signs.find(id);
                // If the road sign is already known to this vehicle
                if (search != vhl->road_signs.end()) {
                  road_sign* sign = search->second;
                  sign->name = Com_getStringData(interface, target(i, "name"));
                  sign->distance = Com_getFloatData(interface, target(i, "distanceToCollision"));
                }
                // Otherwise create a new entry for the sign and add it to the map
                else {
                  road_sign* sign = new road_sign;
                  sign->id = id;
                  sign->name = Com_getStringData(interface, target(i, "name"));
                  sign->distance = Com_getFloatData(interface, target(i, "distanceToCollision"));
                  vhl->road_signs.emplace(id, sign);
                }
              }
            }
          }

          else if (msg.find(NETWORK_ISENSOR_ROADLANESTRAFFICSTATES) == 0) {
            DataInterface* dEventDataInterface = Com_getMessageEventDataInterface(event);
            auto vhlId = Com_getShortData(dEventDataInterface, "vhlId");
            vehicles[vhlId]->set_laneWidth(Com_getFloatData(dEventDataInterface, "LaneTrafficState[0]/width"));
          }
          break;
        }

        case ET_state: {
          if (Com_getStateEventType(event) == ST_Load) {
            Com_validateStateEvent(event);
            const InitialConditions* init = Com_getInitConditions(event);
            vehicles.clear();
            for (int i = 0; i < init->vehiclesCount; i++) {
              const VStruct* vstruct = Com_getInitConditionsVehicle(event, i);
              VehicleInfoStruct vinfostruct;
              auto vhl_id = vstruct->id;
              // Return all vehicle info, can be called only after LOAD, which suits current case
              // The only way to find VType from docs that I found (for dynamicly created vehicle).
              bool ok = Utils_getVehicleInformations(vhl_id, &vinfostruct);
              if (ok) {
                add_vehicle(vhl_id, vinfostruct);
              }
            }
          }
          break;
        }

        default:
          // We are not handling it
          break;
      }
    }

    if (should_run) {
      // If the simulation is currently running
      if (status == PS_RUNNING) {
        Com_updateInputs();

        // Update lanes
        lanes.clear();
        for (Vehicle* vhl : road_vehicles_vec) lanes[vhl->get_lane()].push_back(vhl);

        // Call the correct callback function(s)
        if (just_initialized) {
          if (wait_mode == FREEZE_VEHICLES)
            for (Vehicle* vhl : vehicles_vec) vhl->reset_speed();
          if (first_callback) first_callback();
          just_initialized = false;
        }
        if (running_callback) running_callback();

        // Log error if something went wrong
        if (!Com_updateOutputs(UT_NetworkData)) std::cout << "Failed to update outputs" << std::endl;

        // If run is being benchmarked
        if (benchmark) {
          // Calculate time of frame
          auto end = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

          // Update benchmark variables
          benchmark_total += duration.count();
          benchmark_counter++;

          // If the average should be calculated right now: do so, and log the result inside SCANeR's application log
          if (benchmark_counter == runs) {
            auto avg = benchmark_total / benchmark_counter;
            benchmark_counter = 0;
            benchmark_total = 0;
            std::string benchmarkResult =
                std::to_string(avg) + " microseconds (" + std::to_string(avg / 1000) + " milliseconds)";
            log(benchmarkResult.c_str());
          }
        }
      }
    } else if (wait_mode == FREEZE_VEHICLES)
      for (Vehicle* vhl : vehicles_vec) vhl->set_speed(0.0f);
  }
}

bool Scaner::is_road_vehicle(VType vtype) {
  return std::find(road_vehicle_types.begin(), road_vehicle_types.end(), vtype) != road_vehicle_types.end();
}

/**
 * Adds a vehicle to the vehicles list of this tool.
 *
 * @param id The ID of the vehicle that was added
 * @return The created vehicle
 */
Vehicle* Scaner::add_vehicle(int id, VehicleInfoStruct& vhlInfoStruct) {
  Vehicle* vhl = new Vehicle(id, vhlInfoStruct);
  vehicles.emplace(id, vhl);
  vehicles_vec.push_back(vhl);
  ids_vec.push_back(id);

  if (is_road_vehicle(vhlInfoStruct.type)) {
    road_vehicles.emplace(id, vhl);
    road_vehicles_vec.push_back(vhl);
    road_vehicle_ids_vec.push_back(id);
  }

  return vhl;
}

/**
 * Removes a vehicle from the vehicles list of this tool.
 *
 * @param id The ID of the vehicle to remove
 * @return The removed vehicle
 */
void Scaner::remove_vehicle(int id) {
  Vehicle* vhl = vehicles[id];
  vehicles.erase(id);
  ids_vec.erase(std::remove(ids_vec.begin(), ids_vec.end(), id), ids_vec.end());
  vehicles_vec.erase(std::remove(vehicles_vec.begin(), vehicles_vec.end(), vhl), vehicles_vec.end());
  delete vhl;
}
