#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "ScanerAPI.hpp"
#include "Vehicle.hpp"

/**
 * Defines the behaviour of EasyScaner while it waits for the sensors to fully load.
 */
enum sensor_wait_mode {
  NO_WAIT,    ///< Default behaviour, do not wait for the sensors to start sending data.
  DELAY_RUN,  ///< EasyScaner does not come into effect until the sensors start sending data, autonomous vehicles may
              ///< still be controlled by the TRAFFIC process until then.
  FREEZE_VEHICLES  ///< All vehicles are forced to stand still until the sensors start sending data.
};

class Scaner {
public:
  Scaner(std::string process, std::string config, float frequency);

  // @brief the lanes with a list of vehicles with each lane.
  std::map<short, std::vector<Vehicle*>> lanes;

  // @brief Send a message to the application log.
  //
  // @param The message to send.
  void log(std::string message);

  /**
   * @brief Get process time.
   */
  double get_time();

  /**
   * @brief Logs a message to the SCANeR console.
   *
   * @param message The message to display
   * @param position String representing posistion on screen, should be in form 'RIGHTn' or 'LEFTn', where 'n' number
   * from 1 to 8
   */
  void display_message(std::string message, std::string position);

  // @brief Define the types of vehicles which are considered "road vehicles": get_road_vehicles(),
  // get_road_vehicle_ids(), get_vehicles_in_lane() and get_vehicle_in_front() all ignore vehicle types which have not
  // been set as road vehicles. The default road vehicles are: RIGID, TRACTOR, CAR, BUS, MOTORBIKE, BICYCLE.
  //
  // @param types a list of VTypes which should be considered road vehicles. The available vehicle types are:
  // UNKNOWN_TYPE, RIGID, TRACTOR, SEMI_TRAILER, TRAILER, CAR, BUS, MOTORBIKE, BICYCLE, PEDESTRIAN, STATIC_OBJECT, TRAM,
  // TRAIN, ALIVEBEING, AIRPLANE.
  void set_road_vehicle_types(std::vector<VType> types);

  // @brief Returns true iff the given vehicle is defined as a road vehicle. The types of vehicles which are considered
  // road vehicles can be configured with set_road_vehicle_types() and the default types are: RIGID, TRACTOR, CAR, BUS,
  // MOTORBIKE, BICYCLE.
  bool is_road_vehicle(Vehicle* vhl);

  // @brief Run the traffic flow simulation, calling the function set with while_running() on each tick.
  void run(bool benchmark = false, int runs = 1000);

  // @brief Register a function to be called once with each tick.
  //
  // @param f a void function that interacts with the simulation.
  void while_running(const std::function<void(void)>& f);

  // @brief Register a function to be called once in the first tick.
  //
  // @param f a void function that interacts with the simulation.
  void on_init(const std::function<void(void)>& f);

  // @brief Register a function to be called when a new vehicle is added.
  //
  // @param f a void function that takes a Vehicle (the new Vehicle) as parameter.
  void on_vehicle_added(const std::function<void(Vehicle*)>& f);

  // @brief Set the behaviour of run() when using sensors. Since there is a delay between when the simulation is started
  // and when the sensors start sending data (and this delay can be quite significant if there are a lot of sensors in
  // effect) this method can be used to delay the activation of EasyScaner until data is first received from a sensor.
  //
  // @param wait_mode may be NO_WAIT (the default behaviour: the program does not wait for sensors to start sending
  // data), DELAY_RUN (the program does not come into effect until it starts receiving sensor data) or FREEZE_VEHICLES
  // (all vehicles are forcefully stopped until the sensors start sending data).
  void set_sensor_wait_mode(sensor_wait_mode wait_mode);

  // @brief Returns the vehicle with the given ID.
  //
  // @param id the ID of the vehicle.
  Vehicle* get_vehicle(int id);

  // @brief Returns a list of all vehicles in the simulation, may include non-road vehicles such as pedestrians or
  // airplanes.
  const std::vector<Vehicle*> get_vehicles();

  // @brief Returns a list of all road vehicles in the simulation.
  const std::vector<Vehicle*> get_road_vehicles();

  // @brief Returns the IDs of all vehicles in the simulation, may include non-road vehicles such as pedestrians or
  // airplanes.
  const std::vector<int> get_vehicle_ids();

  // @brief Returns the IDs of all road vehicles in the simulation.
  const std::vector<int> get_road_vehicle_ids();

  // @brief Returns a list of all road vehicles in the given lane.
  //
  // @param lane_id the ID of the lane.
  const std::vector<Vehicle*> get_vehicles_in_lane(short lane_id);

  // @brief Returns the closest road vehicle in front of the given vehicle in the same lane.
  //
  // @param vhl the vehicle for which the car in front should be found.
  Vehicle* get_vehicle_in_front(Vehicle* vhl);

  // @brief Returns the closest road vehicle in front of the given vehicle from the specified vehicles vector.
  // Has the option to use a phantom vehicle, where some of the methods are static and not called at runtime. This can
  // be used to simulate a vehicle being in another position than it actually is. There is also the option to look for a
  // vehicle behind the given vehicle instead of in front.
  //
  // @param vhl - the vehicle for which the car in front should be found.
  // @param vehicles -  the list of vehicles to check the nearest car from.
  // @param phantom - the phantom vehicle to use instead of a vhl or a vehicle from vehicles
  // @param usePhantom - a boolean on whether to use the phantom vehicle
  // @param behind - get the vehicle behind instead of in front of vehicle vhl
  Vehicle* get_vehicle_in_front(Vehicle* vhl, std::vector<Vehicle*> vehicles, PhantomVehicle& phantom,
                                bool usePhantom = false, bool behind = false);

  // @brief Returns the closest road vehicle in front of the given vehicle in the same lane.
  //
  // @param id the ID of the vehicle for which the car in front should be found.
  Vehicle* get_vehicle_in_front(int id);

  // @brief Returns the headway between two vehicles in the same lane, in metres.
  //
  // @param v1 the first vehicle.
  // @param v2 the second vehicle, which should be in the same lane as the first.
  double get_headway(Vehicle* v1, Vehicle* v2, bool behind = false);

private:
  std::vector<VType> road_vehicle_types = {RIGID, TRACTOR, CAR, BUS, MOTORBIKE, BICYCLE};

  std::map<int, Vehicle*> vehicles;
  std::vector<Vehicle*> vehicles_vec;
  std::vector<int> ids_vec;
  std::map<int, Vehicle*> road_vehicles;
  std::vector<Vehicle*> road_vehicles_vec;
  std::vector<int> road_vehicle_ids_vec;

  APIProcessState status;
  sensor_wait_mode wait_mode;
  int benchmark_counter;
  long long benchmark_total;

  bool is_road_vehicle(VType vtype);

  Vehicle* add_vehicle(int id, VehicleInfoStruct& vhlInfoStruct);
  void remove_vehicle(int id);

  // Callbacks
  std::function<void(void)> running_callback;
  std::function<void(void)> first_callback;
  std::function<void(Vehicle*)> vehicle_added_callback;
};
