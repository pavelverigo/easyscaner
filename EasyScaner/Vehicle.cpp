#include "Vehicle.hpp"

#include <cmath>

#include "ScanerAPI.hpp"
#include "Vehicle.hpp"

#define bit(l, i) ((l >> i) & 1) != 0

/**
 * Closes the given interface so that any influences in the simulation will no longer echo in here.
 *
 * @param interface The DataInterface to close
 */
void close_interface(DataInterface* interface) {
  Com_updateOutputDataInterface(interface);
  Com_deleteOutputDataInterface(interface);
}

/**
 * Vehicle constructor.
 *
 * @param vhlId The SCANeR ID of the vehicle
 */
Vehicle::Vehicle(int vhlId, VehicleInfoStruct vhlInfoStruct) {
  id = vhlId;
  vtype = vhlInfoStruct.type;
  vhlUpdate = Com_declareInputData(NETWORK_IVEHICLE_VEHICLEUPDATE, vhlId);
  info = vhlInfoStruct;
}

/**
 * Vehicle deconstructor.
 */
Vehicle::~Vehicle() { Com_releaseInterface(vhlUpdate); }

/**
 * Gets the ID.
 *
 * @return The ID
 */
int Vehicle::get_id() { return id; }

VType Vehicle::get_type() { return vtype; }

/**
 * Gets the position.
 *
 * @return The position
 */
std::array<double, 3> Vehicle::get_pos() {
  return {Com_getDoubleData(vhlUpdate, "pos[0]"), Com_getDoubleData(vhlUpdate, "pos[1]"),
          Com_getDoubleData(vhlUpdate, "pos[2]")};
}

double Vehicle::get_length() { return info.length; }

double Vehicle::get_rear_overhang() { return info.rearOverhang; }

/**
 * Gets the direction.
 *
 * @return The direction
 */
double Vehicle::get_direction() { return Com_getDoubleData(vhlUpdate, "pos[3]"); }

/**
 * Sets the position.
 *
 * @param x The new x position
 * @param y The new y position
 * @param z The new z position
 * @param direction The new direction
 */
void Vehicle::set_pos(double x, double y, double z, float direction) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEMOVE);
  Com_setShortData(interface, "vhlId", id);
  Com_setDoubleData(interface, "pos0", x);
  Com_setDoubleData(interface, "pos1", y);
  Com_setDoubleData(interface, "pos2", z);
  Com_setFloatData(interface, "h", direction);
  close_interface(interface);
}

/**
 * Gets the speed.
 *
 * @return The speed
 */
float Vehicle::get_speed() {
  float x = Com_getFloatData(vhlUpdate, "speed[0]");
  float y = Com_getFloatData(vhlUpdate, "speed[1]");
  float z = Com_getFloatData(vhlUpdate, "speed[2]");
  return std::sqrt(x * x + y * y + z * z);
}

/**
 * Gets the speed per axis.
 *
 * @return The speeds per axis
 */
std::array<float, 3> Vehicle::get_speed_vector() {
  return {Com_getFloatData(vhlUpdate, "speed[0]"), Com_getFloatData(vhlUpdate, "speed[1]"),
          Com_getFloatData(vhlUpdate, "speed[2]")};
}

/**
 * Gets the speed command.
 *
 * @return The speed command
 */
numerical_command Vehicle::get_speed_command() { return speed_command; }

/**
 * Sets the speed.
 *
 * @param speed The target speed
 * @param time How long it should take to reach the target speed in seconds
 */
void Vehicle::set_speed(float speed, float time) {
  speed_command.active = true;
  speed_command.value = speed;

  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "speed", speed);
  Com_setCharData(interface, "state", 1);
  Com_setFloatData(interface, "smoothingTime", time);
  close_interface(interface);
}

/**
 * Sets the maximum speed.
 *
 * @param speed The maximum speed
 */
void Vehicle::set_max_speed(float speed) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETMAXSPEED);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "maxSpeed", speed);
  close_interface(interface);
}

/**
 * Resets the speed.
 */
void Vehicle::reset_speed() {
  speed_command.active = false;

  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  Com_setCharData(interface, "state", 0);
  close_interface(interface);
}

/**
 * Gets the acceleration.
 *
 * @return The acceleration
 */
float Vehicle::get_accel() { return Com_getFloatData(vhlUpdate, "accel[0]"); }

/**
 * Gets the acceleration command.
 *
 * @return The acceleration command
 */
numerical_command Vehicle::get_accel_command() { return accel_command; }

/**
 * Sets the acceleration.
 *
 * @param accel The acceleration
 */
void Vehicle::set_accel(float accel) {
  accel_command.active = true;
  accel_command.value = accel;

  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETACCELERATIONOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "acceleration", accel);
  Com_setCharData(interface, "state", 1);
  close_interface(interface);
}

/**
 * Sets the maximum acceleration.
 *
 * @param accel The maximum acceleration
 */
void Vehicle::set_max_accel(float accel) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEMAXACCELERATION);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "maxAcc", accel);
  close_interface(interface);
}

/**
 * Resets the acceleration.
 */
void Vehicle::reset_accel() {
  accel_command.active = false;

  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETACCELERATIONOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  Com_setCharData(interface, "state", 0);
  close_interface(interface);
}

short Vehicle::get_road() { return Com_getShortData(vhlUpdate, "roadInfo[0]/roadId"); }

/**
 * Gets the lane ID of the lane the vehicle is currently in.
 *
 * @return The lane ID
 */
short Vehicle::get_lane() { return Com_getShortData(vhlUpdate, "roadInfo[0]/laneId"); }

float Vehicle::get_lane_gap() { return Com_getFloatData(vhlUpdate, "roadInfo[0]/laneGap"); }

short Vehicle::get_intersection() { return Com_getShortData(vhlUpdate, "roadInfo[0]/intersectionId"); }

bool Vehicle::in_intersection() { return get_intersection() != -1; }

/**
 * Forces the vehicle to move a lane to the left.
 */
void Vehicle::pull_out() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFORCEPHASEPULLOUTOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

/**
 * Moves a lane to the left when possible.
 */
void Vehicle::try_pull_out() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFORCEPHASEPULLOUT);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

/**
 * Forces the vehicle to move a lane to the right.
 */
void Vehicle::filter_in() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFORCEPHASEFILTERINOBLIGATORY);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

/**
 * Moves a lane to the right when possible.
 */
void Vehicle::try_filter_in() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFORCEPHASEFILTERIN);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

/**
 * Makes the vehicle continue in this lane.
 */
void Vehicle::drive_on() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFORCEPHASEDRIVEON);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

/**
 * Makes the vehicle behave normally.
 */
void Vehicle::reset_lane() {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFREEPHASEDRIVEON);
  Com_setShortData(interface, "vhlId", id);
  close_interface(interface);
}

std::vector<road_sign*> Vehicle::get_road_signs_in_lane() { return road_signs_in_lane; }

road_sign* Vehicle::get_nearest_road_sign_in_lane() {
  if (road_signs_in_lane.empty()) return nullptr;
  road_sign* ret = road_signs_in_lane[0];
  for (road_sign* sign : road_signs_in_lane)
    if (sign->distance < ret->distance) ret = sign;
  return ret;
}

std::vector<traffic_light*> Vehicle::get_traffic_lights_in_lane() { return traffic_lights_in_lane; }

traffic_light* Vehicle::get_nearest_traffic_light_in_lane() {
  if (traffic_lights_in_lane.empty()) return nullptr;
  traffic_light* ret = traffic_lights_in_lane[0];
  for (traffic_light* light : traffic_lights_in_lane)
    if (light->distance < ret->distance) ret = light;
  return ret;
}

/**
 * Gets the angle of the wheels.
 *
 * @return The wheel angle
 */
float Vehicle::get_wheel_angle() { return Com_getFloatData(vhlUpdate, "wheelAngle"); }

/**
 * Gets the lights.
 *
 * @return The lights as a Lights instance
 */
lights Vehicle::get_lights() {
  int l = Com_getLongData(vhlUpdate, "lights");
  return lights{bit(l, 0), bit(l, 1), bit(l, 2), bit(l, 3), bit(l, 4),  bit(l, 5),
                bit(l, 6), bit(l, 7), bit(l, 8), bit(l, 9), bit(l, 10), bit(l, 11)};
}

/**
 * Gets the engine status.
 *
 * @return true if the engine is running, false otherwise
 */
bool Vehicle::get_engine_status() { return Com_getCharData(vhlUpdate, "engineStatus") != 0; }

/**
 * Gets the cruise control mode.
 *
 * @return The cruise control mode
 */
cruise_control_mode Vehicle::get_cruise_control_mode() {
  switch (Com_getCharData(vhlUpdate, "cruiseControlMode")) {
    case 0:
      return CRUISE_CONTROL_OFF;
    case 1:
      return CRUISE_CONTROL;
    case 2:
      return SPEED_LIMITER;
    default:
      return UNDEFINED_CRUISE_CONTROL_MODE;
  }
}

/**
 * Gets the cruise control target speed.
 *
 * @return The target speed
 */
float Vehicle::get_cruise_control_target() { return Com_getFloatData(vhlUpdate, "cruiseControlTarget"); }
cruise_control Vehicle::get_cruise_control() {
  return cruise_control{get_cruise_control_mode(), get_cruise_control_target()};
}

/**
 * Gets the indicator status.
 *
 * @return The indicator status
 */
indicator Vehicle::get_indicator() {
  switch (Com_getCharData(vhlUpdate, "indicators")) {
    case 0:
      return INDICATOR_OFF;
    case 1:
      return LEFT;
    case -1:
      return RIGHT;
    default:
      return UNDEFINED_INDICATOR;
  }
}

/**
 * Gets the horn status.
 *
 * @return true if the horn is being used, false otherwise
 */
bool Vehicle::get_horn() { return Com_getCharData(vhlUpdate, "horn") != 0; }

/**
 * Makes this vehicle follow another vehicle for a set amount of time.
 *
 * @param target_id The ID of the vehicle to follow
 * @param time How long to follow said vehicle
 */
void Vehicle::set_time_to_vehicle(int target_id, float time) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETTIMETOVEHICLE);
  Com_setShortData(interface, "vhlId", id);
  Com_setShortData(interface, "vhlIdTarget", target_id);
  Com_setFloatData(interface, "time", time);
  close_interface(interface);
}

void Vehicle::set_num_of_lanes(short laneNumber) { num_of_lanes = laneNumber; }

int Vehicle::get_num_of_lanes() { return num_of_lanes; }

float Vehicle::get_laneWidth() { return laneWidth; }

void Vehicle::set_laneWidth(float myLaneWidth) { laneWidth = myLaneWidth; }

/* // Set the type of autonomous control of a vehicle
// UNTESTED
void Vehicle::setAutonomousMode(int id, long control, long activation) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETAUTONOMOUSMODE);
  Com_setShortData(interface, "vhlId", id);
  Com_setLongData(interface, "control", control);
  Com_setLongData(interface, "activation", activation);
  close_interface(interface);
}

// The state of the autonomous vehicle controls
// UNTESTED
void Vehicle::autonomousControlState(int id, bool isLateralAutonomous, bool isLongitudinalAutonomous,
                                     bool isSteeringInteraction, bool isGasPedalInteraction,
                                     bool isBrakePedalInteraction, bool isAutonomousRequested, bool isManualRequested) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEAUTONOMOUSCONTROLSTATE);
  Com_setShortData(interface, "vhlId", id);
  Com_setCharData(interface, "isLateralAutonomous", isLateralAutonomous);
  Com_setCharData(interface, "isLongitudinalAutonomous", isLongitudinalAutonomous);
  Com_setCharData(interface, "isSteeringInteraction", isSteeringInteraction);
  Com_setCharData(interface, "isGasPedalInteraction", isGasPedalInteraction);
  Com_setCharData(interface, "isBrakePedalInteraction", isBrakePedalInteraction);
  Com_setCharData(interface, "isAutonomousRequested", isAutonomousRequested);
  Com_setCharData(interface, "isManualRequested", isManualRequested);
  close_interface(interface);
}

// Ask a pull out to a vehicle
// Does not seem to be working
void Vehicle::pullOut(int id, float risk, float duration) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEPULLOUT);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "risk", risk);
  Com_setFloatData(interface, "duration", duration);
  close_interface(interface);
}

// Ask a filter in to a vehicle
// Does not seem to be working
void Vehicle::filterIn(int id, float risk, float duration) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEFILTERIN);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "risk", risk);
  Com_setFloatData(interface, "duration", duration);
  close_interface(interface);
}

// enable/disable longitudinal acceleration rules
// UNTESTED
void Vehicle::setSpeedRuleState(int id, int ruleAccelType, bool state) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDRULESTATE);
  Com_setShortData(interface, "vhlId", id);
  Com_setShortData(interface, "ruleAccelType", ruleAccelType);
  Com_setCharData(interface, "state", state);
  close_interface(interface);
}

// enable/disable target speed
// Tested in SCANeR
// setting
void Vehicle::setSpeedTarget(int id, float targetSpeed, bool state) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDTARGET);
  Com_setShortData(interface, "vhlId", id);
  Com_setFloatData(interface, "targetSpeed", targetSpeed);
  Com_setCharData(interface, "state", state);
  close_interface(interface);
}

// enable/disable stay on lane property
// UNTESTED
void Vehicle::setStayOnLane(int id, bool state) {
  DataInterface* interface = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSTAYONLANE);
  Com_setShortData(interface, "vhlId", id);
  Com_setCharData(interface, "state", state);
  close_interface(interface);
} */

PhantomVehicle::PhantomVehicle(Vehicle* vhl) {
  mockSpeedVector = vhl->get_speed_vector();
  mockDirection = vhl->get_direction();
  mockPos = {0, 0, 0};
  id = vhl->get_id();
}

PhantomVehicle::PhantomVehicle() {}

std::array<double, 3> PhantomVehicle::get_pos() { return mockPos; }

void PhantomVehicle::set_pos(double x, double y, double z) {
  std::array<double, 3> myPos{x, y, z};
  mockPos = myPos;
}

float PhantomVehicle::get_speed() {
  return std::sqrt(mockSpeedVector[0] * mockSpeedVector[0] + mockSpeedVector[1] * mockSpeedVector[1] +
                   mockSpeedVector[2] * mockSpeedVector[2]);
}

std::array<float, 3> PhantomVehicle::get_speed_vector() { return mockSpeedVector; }

float PhantomVehicle::get_direction() { return mockDirection; }

int PhantomVehicle::get_id() { return id; }
