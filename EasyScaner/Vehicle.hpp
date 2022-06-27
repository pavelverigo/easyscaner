#pragma once

#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ScanerAPI.hpp"

/**
 * Describes the currently active mode of cruise control.
 */
enum cruise_control_mode {
  CRUISE_CONTROL_OFF,            ///< Both the cruise control and the speed limiter are off
  CRUISE_CONTROL,                ///< The cruise control is on
  SPEED_LIMITER,                 ///< The speed limiter is on
  UNDEFINED_CRUISE_CONTROL_MODE  ///< Mode is undefined
};

/**
 * Describes what indicators are on.
 */
enum indicator {
  INDICATOR_OFF,       ///< No indicators are turned on
  LEFT,                ///< Left indicator is turned on
  RIGHT,               ///< Right indicator is turned on
  UNDEFINED_INDICATOR  ///< Indicators are undefined
};

enum road_sign_type {
  DANGER = 1,
  DANGER_RIGHT_TURN = 2,
  DANGER_LEFT_TURN = 3,
  DANGER_SPEED_BUMP = 4,
  DANGER_RIGHT_PRIORITY = 5,
  DANGER_TRAFFIC_LIGHT_AHEAD = 6,
  DANGER_ROUNDABOUT_AHEAD = 7,
  DANGER_CROSSWIND = 8,
  DANGER_RIGHT_DOUBLE_CURVE = 9,
  DANGER_LEFT_DOUBLE_CURVE = 10,
  DANGER_CHILDREN = 11,
  DANGER_FALLING_ROCKS = 12,
  DANGER_DOMESTIC_ANIMALS = 13,
  DANGER_ROADWORKS_AHEAD = 14,
  YIELD = 15,
  STOP = 16,
  TRAFFIC_LIGHT = 17,
  WRONG_WAY = 18,
  NO_LEFT_TURN = 19,
  NO_RIGHT_TURN = 20,
  NO_U_TURN = 21,
  NO_OVERTAKING = 22,
  BARRIER = 23,
  NO_PARKING = 24,
  NO_STOP_OR_PARKING = 25,
  STRAIGHT_OBLIGATORY = 26,
  LEFT_TURN_OBLIGATORY = 27,
  RIGHT_TURN_OBLIGATORY = 28,
  LEFT_OR_RIGHT_TURN_OBLIGATORY = 29,
  PROCEED_STRAIGHT_OR_TURN_LEFT = 30,
  PROCEED_STRAIGHT_OR_TURN_RIGHT = 31,
  LEFT_DIRECTION_OBLIGATORY = 32,
  RIGHT_DIRECTION_OBLIGATORY = 33,
  PASS_ON_RIGHT_SIDE = 34,
  PASS_ON_LEFT_SIDE = 35,
  MESSAGE_BOARD = 36,
  DISTANCE_BOARD = 37,
  PARKING = 38,
  DEAD_END = 39,
  TOLL_GATE = 40,
  START_HIGHWAY = 41,
  END_HIGHWAY = 42,
  START_PRIORITY_ROAD = 43,
  END_PRIORITY_ROAD = 44,
  DIRECTION = 45,
  EXIT = 46,
  ONE_WAY = 47,
  PEDESTRIAN_AREA = 48,
  PEDESTRIAN_CROSSING = 49,
  RAILROAD_CROSSING = 50,
  TRAMWAY_CROSSING = 51,
  PEDESTRIAN_CROSSING_AHEAD = 52,
  TRAM_CROSSING_AHEAD = 53,
  SPEED_LIMIT = 54,
  END_SPEED_LIMIT = 55,
  MINIMUM_SPEED = 56,
  SPEED_LIMIT_ZONE = 57,
  END_SPEED_LIMIT_ZONE = 58,
  ENTERING_BUILT_UP_AREA = 59,
  LEAVING_BUILT_UP_AREA = 60,
  CYCLE_LANE = 61,
  END_CYCLE_LANE = 62,
  RESERVED_BICYCLE = 63,
  END_RESERVED_BICYCLE = 64,
  RESERVED_PEDESTRIAN = 65,
  END_RESERVED_PEDESTRIAN = 66,
  RESERVED_PUBLIC_TRANSPORT = 67,
  END_RESERVED_PUBLIC_TRANSPORT = 68,
  RESERVED_TRAM = 69,
  END_RESERVED_TRAM = 70,
  RESERVED_EQUESTRIAN = 71,
  END_RESERVED_EQUESTRIAN = 72,
  DANGER_TWO_WAY_TRAFFIC = 73,
  OVERTAKE_SEGMENT = 74,
  DOUBLE_OVERTAKE_SEGMENT = 75,
  END_NO_OVERTAKING = 76,
  SWITCH_ON_HEADLAMPS = 77,
  SWITCH_OFF_HEADLAMPS = 78,
  TUNNEL = 79,
  BEACON = 80,
  BEACON_TURN_RIGHT = 81,
  BEACON_TURN_LEFT = 82,
  MAXIMUM_HEIGHT = 83,
  BUS_STOP = 84,
  END_OF_OBLIGATION = 85,
  MINIMUM_DISTANCE = 86,
  SNOW_CHAIN_OBLIGATORY = 87,
  UNKNOWN_SIGN = 0
};

enum traffic_light_color { OFF = 1, GREEN = 2, YELLOW = 3, YELLOW_AND_RED = 4, RED = 5, UNDEFINED_COLOR = 0 };

/**
 * Describes a command that has been given to a vehicle, where the command is based on a float.
 *
 * Examples include orders to move or accelerate at a certain speed, using `Vehicle::set_speed()` or
 * `Vehicle::set_accel()`. You can obtain an command that has been given to a vehicle with functions such as
 * `Vehicle::get_accel_command()`.
 */
struct numerical_command {
  bool active = false;  ///< Only true when a command is in effect.
  float value = 0.0f;   ///< Gives the value that's been assigned to this command.
};
/**
 * Describes the status of all the external lights on a vehicle.
 */
struct lights {
  bool left_indicator;   ///< As the indicator blinks, the status changes between on and off; use
                         ///< `Vehicle::get_indicator()` to reliably get the status.
  bool right_indicator;  ///< As the indicator blinks, the status changes between on and off; use
                         ///< `Vehicle::get_indicator()` to reliably get the status.

  bool stop_lights;
  bool dipped_lights;
  bool full_lights;
  bool fog_lights;
  bool reverse_lights;
  bool side_lights;
  bool rear_fog_lights;
  bool rear_lights;
  bool left_dlr_lights;
  bool right_dlr_lights;
};

/**
 * Describes the status of a vehicle's cruise control.
 */
struct cruise_control {
  cruise_control_mode mode;  ///< The mode of the cruise control: CRUISE_CONTROL_OFF, CRUISE_CONTROL or SPEED_LIMITER.
  float target;              ///< The cruise control's target speed in m/s.
};

/**
 * Describes a detected road sign and its distance to the vehicle that detected it.
 */
struct road_sign {
  int id = -1;                         ///< The ID of the road sign in SCANeR.
  std::string name = "";               ///< The name of the road sign set in the scenario.
  float distance = -1;                 ///< The distance of the road sign to the vehicle which detected it.
  road_sign_type type = UNKNOWN_SIGN;  ///< The type of road sign, there are 87 possible `road_sign_type`s.
  float value = -1;                    ///< The value displayed on this road sign, in cases such as speed limit signs.
};

/**
 * Describes a detected traffic light and its distance to the vehicle that detected it.
 */
struct traffic_light {
  int id = -1;                                  ///< The ID of the traffic light in SCANeR.
  std::string name = "";                        ///< The name of the traffic light set in the scenario.
  float distance = -1;                          ///< The distance of the traffic light to the vehicle which detected it.
  traffic_light_color color = UNDEFINED_COLOR;  ///< The colour of the traffic light: OFF, GREEN, YELLOW,
                                                ///< YELLOW_AND_RED, RED or UNDEFINED_COLOR.
  bool flashing = false;                        ///< True iff the traffic light is blinking.
};

// @brief A class for interacting with a vehicle and the commands given to it using the TrafficFlowController API.
// Provides getters for obtaining information about the current state of the vehicle, and setters for giving orders.
class Vehicle {
public:
  // Don't make bindings for these, they're for internal use only
  std::map<int, road_sign*> road_signs;
  std::map<int, traffic_light*> traffic_lights;
  std::vector<traffic_light*> traffic_lights_in_lane;
  std::vector<road_sign*> road_signs_in_lane;

  Vehicle(int vhlId, VehicleInfoStruct vhlInfoStruct);
  ~Vehicle();

  Vehicle() = default;

  // @brief Returns the ID of the vehicle as used in SCANeR.
  int get_id();

  // @brief Returns the SCANeR VType of the vehicle, the possible vehicle types are: UNKNOWN_TYPE, RIGID, TRACTOR,
  // SEMI_TRAILER, TRAILER, CAR, BUS, MOTORBIKE, BICYCLE, PEDESTRIAN, STATIC_OBJECT, TRAM, TRAIN, ALIVEBEING, AIRPLANE.
  VType get_type();

  // @brief Returns an array with the current position of the vehicle in the order x, y, z.
  std::array<double, 3> get_pos();

  // @brief Returns the direction in which the vehicle is heading.
  double get_direction();

  // @brief Moves the vehicle to the given absolute coordinates.
  //
  // @param x the absolute x-coordinate to move the vehicle to.
  // @param y the absolute y-coordinate to move the vehicle to.
  // @param z the absolute z-coordinate to move the vehicle to.
  // @param direction the direction in which the vehicle is to be heading, in radians.
  void set_pos(double x, double y, double z, float direction);

  // @brief Returns the current speed of the vehicle in the forward direction, in m/s.
  float get_speed();

  // @brief Returns the current speed of the vehicle as a vector.
  std::array<float, 3> get_speed_vector();

  // @brief Returns a numerical_command which describes orders given using set_speed().
  //
  // @return numerical_command.active is true iff the vehicle has been ordered to move at a certain speed.
  // numerical_command.value then gives the speed the vehicle has been ordered to move at, in m/s.
  numerical_command get_speed_command();

  // @brief Order a vehicle to move at a certain speed.
  //
  // @param speed the target speed in m/s.
  // @param time the time in which the vehicle should accelerate/decelerate to the target speed. This is an optional
  // parameter and 0 by default (meaning the vehicle changes speed instantly).
  void set_speed(float speed, float time = 0.0f);

  // @brief Set the maximum speed of the vehicle.
  //
  // @param speed the maximum speed, in m/s.
  void set_max_speed(float speed);

  // @brief Disable any orders made using set_speed().
  // The speed of the vehicle may then be controlled by other processes.
  void reset_speed();

  // @brief Returns the current acceleration of the vehicle in the forward direction, in m/s/s.
  float get_accel();

  // @brief Returns a numerical_command which describes orders given using set_accel().
  //
  // @return numerical_command.active is true iff the vehicle has been ordered to accelerate at a certain rate.
  // numerical_command.value then gives the rate the vehicle has been ordered to accelerate at, in m/s/s.
  numerical_command get_accel_command();

  // @brief Order a vehicle to accelerate at a certain rate.
  //
  // @param accel the target acceleration in m/s/s.
  void set_accel(float accel);

  // @brief Set the maximum acceleration of the vehicle.
  //
  // @param accel the maximum acceleration, in m/s/s.
  void set_max_accel(float accel);

  // @brief Disable any orders made using set_accel().
  // The acceleration of the vehicle may then be controlled by other processes.
  void reset_accel();

  // @brief Returns the ID of the road section the vehicle is on.
  short get_road();

  // @brief Returns the ID of the vehicle's current lane.
  short get_lane();

  // @brief Returns the gap from the middle of the lane to the current position.
  float get_lane_gap();

  // @brief Returns the ID of the intersection the vehicle is currently in, or -1 if it is not in any intersection.
  short get_intersection();

  // @brief Returns true iff the vehicle is currently in an intersection (waiting at an intersection doesn't count).
  bool in_intersection();

  // @brief Order the vehicle to change lane to the left.
  // Surrounding vehicles will not be taken into account, although the vehicle will still try to avoid collisions.
  void pull_out();

  // @brief Order the vehicle to try changing lane to the left. The vehicle will change lane as soon as it's safe.
  void try_pull_out();

  // @brief Order the vehicle to change lane to the right.
  // Surrounding vehicles will not be taken into account, although the vehicle will still try to avoid collisions.
  void filter_in();

  // @brief Order the vehicle to try changing lane to the right. The vehicle will change lane as soon as it's safe.
  void try_filter_in();

  // @brief Order the vehicle to stay in its current lane.
  void drive_on();

  // @brief Disable any orders made using pull_out(), filter_in() or drive_on().
  // The lane-selection behaviour of the vehicle may then be controlled by other processes.
  void reset_lane();

  // @brief Returns a list of road signs in the same lane detected by a sensor attached to the vehicle.
  // For this to work, the vehicle must have a sensor which is set to output target anchor points and logical traffic
  // infos, with vertical road signs, horizontal road signs and lanes among its selected targets.
  std::vector<road_sign*> get_road_signs_in_lane();

  // @brief Returns the nearest road sign in the same lane detected by a sensor attached to the vehicle.
  // For this to work, the vehicle must have a sensor which is set to output target anchor points and logical traffic
  // infos, with vertical road signs, horizontal road signs and lanes among its selected targets.
  //
  // @return A road_sign struct describing the nearest road sign to the vehicle, or a null pointer if no road
  // signs were detected in the lane.
  road_sign* get_nearest_road_sign_in_lane();

  // @brief Returns a list of traffic lights in the same lane detected by a sensor attached to the vehicle.
  // For this to work, the vehicle must have a sensor which is set to output target anchor points and logical traffic
  // infos, with traffic lights and lanes among its selected targets.
  std::vector<traffic_light*> get_traffic_lights_in_lane();

  // @brief Returns the nearest traffic light in the same lane detected by a sensor attached to the vehicle.
  // For this to work, the vehicle must have a sensor which is set to output target anchor points and logical traffic
  // infos, with traffic lights and lanes among its selected targets.
  //
  // @return A traffic_light struct describing the nearest traffic light to the vehicle, or a null pointer if no traffic
  // lights were detected in the lane.
  traffic_light* get_nearest_traffic_light_in_lane();

  // @brief Returns the relative heading of the wheels of the vehicle in radians, left being positive.
  float get_wheel_angle();

  // @brief Returns the status of all the vehicle's external lights.
  lights get_lights();

  // @brief Returns true iff the vehicle's engine is on.
  bool get_engine_status();

  // @brief Returns the current mode of the vehicle's cruise control.
  //
  // @return OFF, CRUISE_CONTROL or SPEED_LIMITER.
  cruise_control_mode get_cruise_control_mode();

  // @brief Returns the target speed of the vehicle's cruise control, in m/s.
  float get_cruise_control_target();

  // @brief Returns the current status of the vehicle's cruise control: the mode it is set to and its target speed.
  cruise_control get_cruise_control();

  // @brief Returns the way the vehicle is currently signalling a turn.
  //
  // @return INDICATOR_OFF, LEFT or RIGHT.
  indicator get_indicator();

  // @brief Returns true iff the vehicle's horn is on.
  bool get_horn();

  // @brief Order the vehicle to follow the target vehicle with the given time.
  // Does not offer control over the car-following model used.
  // UNTESTED
  //
  // @param target_id the ID of the vehicle to follow.
  // @param time the time to the vehicle being followed, positive times being behind the target.
  void set_time_to_vehicle(int target_id, float time);

  // @brief Gets the length of a vehicle.
  //
  // @return The length of a vehicle.
  double get_length();

  // @brief Gets the rear overhang of a vehicle.
  //
  // @return The length of a vehicle.
  double get_rear_overhang();

  void set_num_of_lanes(short laneNumber);

  int get_num_of_lanes();

  float get_laneWidth();

  void set_laneWidth(float lanewidth);

  VehicleInfoStruct info;
  /* // @brief Set the type of autonomous control of a vehicle
  // UNTESTED
  void setAutonomousMode(int id, long control, long activation);

  // @brief The state of the autonomous vehicle controls
  // UNTESTED
  void autonomousControlState(int id, bool isLateralAutonomous, bool isLongitudinalAutonomous,
                              bool isSteeringInteraction, bool isGasPedalInteraction, bool isBrakePedalInteraction,
                              bool isAutonomousRequested, bool isManualRequested);

  // @brief Ask a pull out to a vehicle
  // Does not seem to be working
  void pullOut(int id, float risk, float duration);

  // @brief Ask a filter in to a vehicle
  // Does not seem to be working
  void filterIn(int id, float risk, float duration);

  // @brief enable/disable longitudinal acceleration rules
  // UNTESTED
  void setSpeedRuleState(int id, int ruleAccelType, bool state);

  // @brief enable/disable target speed
  // UNTESTED
  void setSpeedTarget(int id, float targetSpeed, bool state);

  // @brief enable/disable stay on lane property
  // UNTESTED
  void setStayOnLane(int id, bool state); */

private:
  DataInterface* vhlUpdate;
  int id;
  VType vtype;
  int num_of_lanes;
  float laneWidth;

  numerical_command speed_command;
  numerical_command accel_command;
};

class PhantomVehicle {
public:
  PhantomVehicle(Vehicle* vhl);
  PhantomVehicle();
  std::array<double, 3> get_pos();
  void set_pos(double x, double y, double z);
  float get_speed();
  std::array<float, 3> get_speed_vector();
  float get_direction();
  int get_id();

private:
  int id;
  std::array<double, 3> mockPos;
  float mockDirection;
  std::array<float, 3> mockSpeedVector;
};
