#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "EasyScaner/Scaner.hpp"
#include "EasyScaner/Vehicle.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyscaner, m) {
  m.doc() = R"pbdoc(
    Python API
    -----------------------
    .. currentmodule:: pyscaner
    .. autosummary::
      :toctree: _generate_autosummary

      API
      SensorWaitMode
      Vehicle
      VType
      CruiseControlMode
      Indicator
      LightsInfo
      NumericalCommand
  )pbdoc";

  // Test
  m.def(
      "add", [](int a, int b) { return a + b; }, R"pbdoc(
    Add two numbers together. Created for testing, will be deleted in future.

    :param a: int, Number 1 
    :param b: int, Number 2
  )pbdoc",
      py::arg("a"), py::arg("b"));

  auto vhl_t = py::enum_<VType>(m, "VType", "Vehicle types");
  vhl_t.value("UNKNOWN_TYPE", VType::UNKNOWN_TYPE);
  vhl_t.value("RIGID", VType::RIGID);
  vhl_t.value("TRACTOR", VType::TRACTOR);
  vhl_t.value("SEMI_TRAILER", VType::SEMI_TRAILER);
  vhl_t.value("TRAILER", VType::TRAILER);
  vhl_t.value("CAR", VType::CAR);
  vhl_t.value("BUS", VType::BUS);
  vhl_t.value("MOTORBIKE", VType::MOTORBIKE);
  vhl_t.value("BICYCLE", VType::BICYCLE);
  vhl_t.value("PEDESTRIAN", VType::PEDESTRIAN);
  vhl_t.value("STATIC_OBJECT", VType::STATIC_OBJECT);
  vhl_t.value("TRAM", VType::TRAM);
  vhl_t.value("TRAIN", VType::TRAIN);
  vhl_t.value("ALIVEBEING", VType::ALIVEBEING);
  vhl_t.value("AIRPLANE", VType::AIRPLANE);

  auto api = py::class_<Scaner>(m, "API", "Scaner API class");
  api.def(py::init<std::string, std::string, float>(), "Constructor for API class");

  api.def("log", &Scaner::log, "Send a message to the application log", py::arg("message"));

  api.def("get_time", &Scaner::get_time, "Return process time");

  api.def("display_message", &Scaner::display_message, R"pbdoc(
        Display image on screen, "position" should be in form "RIGHTn" or "LEFTn", 
        where "n" number from 1 to 8)pbdoc",
          py::arg("message"), py::arg("position"));

  api.def("set_road_vehicle_types", &Scaner::set_road_vehicle_types,
          R"pbdoc(Define the types of vehicles which are considered "road vehicles": get_road_vehicles())pbdoc",
          py::arg("types"));

  bool (Scaner::*is_road_vehicle)(Vehicle*) = &Scaner::is_road_vehicle;
  api.def("is_road_vehicle", is_road_vehicle, "Returns true iff the given vehicle is defined as a road vehicle",
          py::arg("vehicle"));

  api.def("run", &Scaner::run,
          "Run the traffic flow simulation, calling the function set with while_running() on each tick",
          py::arg("benchmark") = false, py::arg("runs") = 1000);
  api.def("while_running", &Scaner::while_running, "Register a function to be called once with each tick",
          py::arg("function"));
  api.def("on_init", &Scaner::on_init, "Register a function to be called once in the first tick", py::arg("function"));
  api.def("on_vehicle_added", &Scaner::on_vehicle_added, "Register a function to be called when a new vehicle is added",
          py::arg("function"));
  api.def("get_vehicle", &Scaner::get_vehicle, py::return_value_policy::reference,
          "Returns the vehicle with the given ID", py::arg("id"));
  api.def("get_vehicles", &Scaner::get_vehicles, py::return_value_policy::reference,
          "Returns a list of all vehicles in the simulation, may include non-road vehicles such as pedestrians or "
          "airplanes");
  api.def("get_road_vehicles", &Scaner::get_road_vehicles, py::return_value_policy::reference,
          "Returns a list of all road vehicles in the simulation");
  api.def("get_vehicle_ids", &Scaner::get_vehicle_ids,
          "Returns the IDs of all vehicles in the simulation, may include non-road vehicles such as pedestrians or "
          "airplanes");
  api.def("get_road_vehicle_ids", &Scaner::get_road_vehicle_ids,
          "Returns the IDs of all road vehicles in the simulation");
  api.def("get_vehicles_in_lane", &Scaner::get_vehicles_in_lane, py::return_value_policy::reference,
          "Returns a list of all road vehicles in the given lane", py::arg("lane_id"));

  // Trick to get pointer to overloaded function
  Vehicle* (Scaner::*in_front_by_id)(int) = &Scaner::get_vehicle_in_front;
  api.def("get_vehicle_in_front_by_id", in_front_by_id, py::return_value_policy::reference,
          "Returns the closest road vehicle in front of the given vehicle in the same lane", py::arg("vehicle_id"));

  Vehicle* (Scaner::*in_front_by_vhl)(Vehicle*) = &Scaner::get_vehicle_in_front;
  api.def("get_vehicle_in_front", in_front_by_vhl, py::return_value_policy::reference,
          "Returns the closest road vehicle in front of the given vehicle in the same lane", py::arg("vehicle"));

  api.def("get_headway", &Scaner::get_headway, "Returns the headway between two vehicles in the same lane, in metres",
          py::arg("vehicle1"), py::arg("vehicle2"), py::arg("behind") = false);

  auto sensor_wait_mode_e = py::enum_<sensor_wait_mode>(
      m, "SensorWaitMode", "Defines the behaviour of EasyScaner while it waits for the sensors to fully load");
  sensor_wait_mode_e.value("NO_WAIT", sensor_wait_mode::NO_WAIT,
                           "Default behaviour, do not wait for the sensors to start sending data");
  sensor_wait_mode_e.value("DELAY_RUN", sensor_wait_mode::DELAY_RUN,
                           R"pbdoc(EasyScaner does not come into effect until the sensors start sending data,
    autonomous vehicles may still be controlled by the TRAFFIC process until then)pbdoc");
  sensor_wait_mode_e.value("FREEZE_VEHICLES", sensor_wait_mode::FREEZE_VEHICLES,
                           "All vehicles are forced to stand still until the sensors start sending data");

  // set_sensor_wait_modeset_sensor_wait_modeset_sensor_wait_mode
  api.def(
      "set_sensor_wait_mode", &Scaner::set_sensor_wait_mode,
      R"pbdoc(Set the behaviour of run() when using sensors. Since there is a delay between when the simulation is started
    and when the sensors start sending data (and this delay can be quite significant if there are a lot of sensors in
    effect) this method can be used to delay the activation of EasyScaner until data is first received from a sensor.)pbdoc",
      py::arg("wait_mode"));

  auto vhl = py::class_<Vehicle>(m, "Vehicle", "Vehicle class, describing vehicle instance inside SCANeR simulation");

  vhl.def("get_id", &Vehicle::get_id, "Returns the ID of the vehicle as used in SCANeR");

  vhl.def("get_type", &Vehicle::get_type, "Returns the SCANeR VType of the vehicle");

  vhl.def("get_pos", &Vehicle::get_pos,
          "Returns an array with the current position of the vehicle in the order x, y, z");
  vhl.def("get_direction", &Vehicle::get_direction, "Returns the direction in which the vehicle is heading");
  vhl.def("set_pos", &Vehicle::set_pos, "Moves the vehicle to the given absolute coordinates", py::arg("x"),
          py::arg("y"), py::arg("z"), py::arg("direction"));

  vhl.def("get_speed", &Vehicle::get_speed,
          "Returns the current speed of the vehicle in the forward direction, in m/s");
  vhl.def("get_speed_vector", &Vehicle::get_speed_vector, "Returns the current speed of the vehicle as a vector");
  vhl.def("get_speed_command", &Vehicle::get_speed_command,
          "Returns a numerical_command which describes orders given using set_speed()");
  vhl.def("set_speed", &Vehicle::set_speed, "Order a vehicle to move at a certain speed", py::arg("speed"),
          py::arg("time") = 0.0f);
  vhl.def("set_max_speed", &Vehicle::set_max_speed, "Set the maximum speed of the vehicle", py::arg("speed"));
  vhl.def("reset_speed", &Vehicle::reset_speed, "Disable any orders made using set_speed()");

  vhl.def("get_accel", &Vehicle::get_accel,
          "Returns the current acceleration of the vehicle in the forward direction, in m/s/s");
  vhl.def("get_accel_command", &Vehicle::get_accel_command,
          "Returns a numerical_command which describes orders given using set_accel()");
  vhl.def("set_accel", &Vehicle::set_accel, "Order a vehicle to accelerate at a certain rate", py::arg("accel"));
  vhl.def("set_max_accel", &Vehicle::set_max_accel, " Set the maximum acceleration of the vehicle", py::arg("accel"));
  vhl.def("reset_accel", &Vehicle::reset_accel, "Disable any orders made using set_accel()");

  vhl.def("get_road", &Vehicle::get_road, "Returns the ID of the road section the vehicle is on");
  vhl.def("get_lane", &Vehicle::get_lane, "Returns the ID of the vehicle's current lane");

  vhl.def("get_intersection", &Vehicle::get_intersection,
          "Returns the ID of the intersection the vehicle is currently in, or -1 if it is not in any intersection");
  vhl.def("in_intersection", &Vehicle::in_intersection,
          "Returns true iff the vehicle is currently in an intersection (waiting at an intersection doesn't count)");

  vhl.def("pull_out", &Vehicle::pull_out,
          "Order the vehicle to change lane to the left. Surrounding vehicles will not be taken into account");
  vhl.def("try_pull_out", &Vehicle::try_pull_out,
          "Order the vehicle to try changing lane to the left. The vehicle will change lane as soon as it's safe");

  vhl.def("filter_in", &Vehicle::filter_in,
          "Order the vehicle to change lane to the right. Surrounding vehicles will not be taken into account");
  vhl.def("try_filter_in", &Vehicle::try_filter_in,
          "Order the vehicle to try changing lane to the right. The vehicle will change lane as soon as it's safe");

  vhl.def("drive_on", &Vehicle::drive_on, "Order the vehicle to stay in its current lane");

  vhl.def("reset_lane", &Vehicle::reset_lane, "Disable any orders made using pull_out(), filter_in() or drive_on()");

  vhl.def("get_wheel_angle", &Vehicle::get_wheel_angle,
          "Returns the relative heading of the wheels of the vehicle in radians, left being positive");

  vhl.def("get_engine_status", &Vehicle::get_engine_status, "Returns true iff the vehicle's engine is on");

  vhl.def("get_horn", &Vehicle::get_horn, "Returns true iff the vehicle's horn is on");

  vhl.def("get_laneWidth", &Vehicle::get_laneWidth, "Returns the width of the current lane");

  vhl.def("get_num_of_lanes", &Vehicle::get_num_of_lanes,
          "Returns the number of lanes on the road that the vehicle is driving on");

  auto lights_t = py::class_<lights>(m, "LightsInfo", "Describes the status of all the external lights on a vehicle");

  lights_t.def_readonly("left_indicator", &lights::left_indicator,
                        "As the indicator blinks, the status changes between on and off; use "
                        "`Vehicle::get_indicator()` to reliably get the status.");
  lights_t.def_readonly("right_indicator", &lights::right_indicator,
                        "As the indicator blinks, the status changes between on and off; use "
                        "`Vehicle::get_indicator()` to reliably get the status.");
  lights_t.def_readonly("stop_lights", &lights::stop_lights);
  lights_t.def_readonly("dipped_lights", &lights::dipped_lights);
  lights_t.def_readonly("full_lights", &lights::full_lights);
  lights_t.def_readonly("fog_lights", &lights::fog_lights);
  lights_t.def_readonly("rear_fog_lights", &lights::rear_fog_lights);
  lights_t.def_readonly("rear_lights", &lights::rear_lights);
  lights_t.def_readonly("left_dlr_lights", &lights::left_dlr_lights);
  lights_t.def_readonly("right_dlr_lights", &lights::right_dlr_lights);

  vhl.def("get_lights", &Vehicle::get_lights, "Returns the status of all the vehicle's external lights");

  auto ccm_e =
      py::enum_<cruise_control_mode>(m, "CruiseControlMode", "Describes the currently active mode of cruise control");
  ccm_e.value("CRUISE_CONTROL_OFF", cruise_control_mode::CRUISE_CONTROL_OFF);
  ccm_e.value("CRUISE_CONTROL", cruise_control_mode::CRUISE_CONTROL);
  ccm_e.value("SPEED_LIMITER", cruise_control_mode::SPEED_LIMITER);
  ccm_e.value("UNDEFINED_CRUISE_CONTROL_MODE", cruise_control_mode::UNDEFINED_CRUISE_CONTROL_MODE);

  // vhl.def("get_cruise_control", &Vehicle::get_cruise_control, "Returns the current status of the vehicle's cruise
  // control: the mode it is set to and its target speed");

  vhl.def("get_cruise_control_mode", &Vehicle::get_cruise_control_mode,
          "Returns the current mode of the vehicle's cruise control");

  vhl.def("get_cruise_control_target", &Vehicle::get_cruise_control_target,
          "Returns the target speed of the vehicle's cruise control, in m/s");

  auto indicator_e = py::enum_<indicator>(m, "Indicator", "Describes what indicators are on.");
  indicator_e.value("INDICATOR_OFF", indicator::INDICATOR_OFF);
  indicator_e.value("LEFT", indicator::LEFT);
  indicator_e.value("RIGHT", indicator::RIGHT);
  indicator_e.value("UNDEFINED_INDICATOR", indicator::UNDEFINED_INDICATOR);

  vhl.def("get_indicator", &Vehicle::get_indicator, "Returns the way the vehicle is currently signalling a turn");

  auto road_sign_type_e = py::enum_<road_sign_type>(m, "RoadSignType", "Describes the type of a road sign");
  road_sign_type_e.value("DANGER", road_sign_type::DANGER);
  road_sign_type_e.value("DANGER_RIGHT_TURN", road_sign_type::DANGER_RIGHT_TURN);
  road_sign_type_e.value("DANGER_LEFT_TURN", road_sign_type::DANGER_LEFT_TURN);
  road_sign_type_e.value("DANGER_SPEED_BUMP", road_sign_type::DANGER_SPEED_BUMP);
  road_sign_type_e.value("DANGER_RIGHT_PRIORITY", road_sign_type::DANGER_RIGHT_PRIORITY);
  road_sign_type_e.value("DANGER_TRAFFIC_LIGHT_AHEAD", road_sign_type::DANGER_TRAFFIC_LIGHT_AHEAD);
  road_sign_type_e.value("DANGER_ROUNDABOUT_AHEAD", road_sign_type::DANGER_ROUNDABOUT_AHEAD);
  road_sign_type_e.value("DANGER_CROSSWIND", road_sign_type::DANGER_CROSSWIND);
  road_sign_type_e.value("DANGER_RIGHT_DOUBLE_CURVE", road_sign_type::DANGER_RIGHT_DOUBLE_CURVE);
  road_sign_type_e.value("DANGER_LEFT_DOUBLE_CURVE", road_sign_type::DANGER_LEFT_DOUBLE_CURVE);
  road_sign_type_e.value("DANGER_CHILDREN", road_sign_type::DANGER_CHILDREN);
  road_sign_type_e.value("DANGER_FALLING_ROCKS", road_sign_type::DANGER_FALLING_ROCKS);
  road_sign_type_e.value("DANGER_DOMESTIC_ANIMALS", road_sign_type::DANGER_DOMESTIC_ANIMALS);
  road_sign_type_e.value("DANGER_ROADWORKS_AHEAD", road_sign_type::DANGER_ROADWORKS_AHEAD);
  road_sign_type_e.value("YIELD", road_sign_type::YIELD);
  road_sign_type_e.value("STOP", road_sign_type::STOP);
  road_sign_type_e.value("TRAFFIC_LIGHT", road_sign_type::TRAFFIC_LIGHT);
  road_sign_type_e.value("WRONG_WAY", road_sign_type::WRONG_WAY);
  road_sign_type_e.value("NO_LEFT_TURN", road_sign_type::NO_LEFT_TURN);
  road_sign_type_e.value("NO_RIGHT_TURN", road_sign_type::NO_RIGHT_TURN);
  road_sign_type_e.value("NO_U_TURN", road_sign_type::NO_U_TURN);
  road_sign_type_e.value("NO_OVERTAKING", road_sign_type::NO_OVERTAKING);
  road_sign_type_e.value("BARRIER", road_sign_type::BARRIER);
  road_sign_type_e.value("NO_PARKING", road_sign_type::NO_PARKING);
  road_sign_type_e.value("NO_STOP_OR_PARKING", road_sign_type::NO_STOP_OR_PARKING);
  road_sign_type_e.value("STRAIGHT_OBLIGATORY", road_sign_type::STRAIGHT_OBLIGATORY);
  road_sign_type_e.value("LEFT_TURN_OBLIGATORY", road_sign_type::LEFT_TURN_OBLIGATORY);
  road_sign_type_e.value("RIGHT_TURN_OBLIGATORY", road_sign_type::RIGHT_TURN_OBLIGATORY);
  road_sign_type_e.value("LEFT_OR_RIGHT_TURN_OBLIGATORY", road_sign_type::LEFT_OR_RIGHT_TURN_OBLIGATORY);
  road_sign_type_e.value("PROCEED_STRAIGHT_OR_TURN_LEFT", road_sign_type::PROCEED_STRAIGHT_OR_TURN_LEFT);
  road_sign_type_e.value("PROCEED_STRAIGHT_OR_TURN_RIGHT", road_sign_type::PROCEED_STRAIGHT_OR_TURN_RIGHT);
  road_sign_type_e.value("LEFT_DIRECTION_OBLIGATORY", road_sign_type::LEFT_DIRECTION_OBLIGATORY);
  road_sign_type_e.value("RIGHT_DIRECTION_OBLIGATORY", road_sign_type::RIGHT_DIRECTION_OBLIGATORY);
  road_sign_type_e.value("PASS_ON_RIGHT_SIDE", road_sign_type::PASS_ON_RIGHT_SIDE);
  road_sign_type_e.value("PASS_ON_LEFT_SIDE", road_sign_type::PASS_ON_LEFT_SIDE);
  road_sign_type_e.value("MESSAGE_BOARD", road_sign_type::MESSAGE_BOARD);
  road_sign_type_e.value("DISTANCE_BOARD", road_sign_type::DISTANCE_BOARD);
  road_sign_type_e.value("PARKING", road_sign_type::PARKING);
  road_sign_type_e.value("DEAD_END", road_sign_type::DEAD_END);
  road_sign_type_e.value("TOLL_GATE", road_sign_type::TOLL_GATE);
  road_sign_type_e.value("START_HIGHWAY", road_sign_type::START_HIGHWAY);
  road_sign_type_e.value("END_HIGHWAY", road_sign_type::END_HIGHWAY);
  road_sign_type_e.value("START_PRIORITY_ROAD", road_sign_type::START_PRIORITY_ROAD);
  road_sign_type_e.value("END_PRIORITY_ROAD", road_sign_type::END_PRIORITY_ROAD);
  road_sign_type_e.value("DIRECTION", road_sign_type::DIRECTION);
  road_sign_type_e.value("EXIT", road_sign_type::EXIT);
  road_sign_type_e.value("ONE_WAY", road_sign_type::ONE_WAY);
  road_sign_type_e.value("PEDESTRIAN_AREA", road_sign_type::PEDESTRIAN_AREA);
  road_sign_type_e.value("PEDESTRIAN_CROSSING", road_sign_type::PEDESTRIAN_CROSSING);
  road_sign_type_e.value("RAILROAD_CROSSING", road_sign_type::RAILROAD_CROSSING);
  road_sign_type_e.value("TRAMWAY_CROSSING", road_sign_type::TRAMWAY_CROSSING);
  road_sign_type_e.value("PEDESTRIAN_CROSSING_AHEAD", road_sign_type::PEDESTRIAN_CROSSING_AHEAD);
  road_sign_type_e.value("TRAM_CROSSING_AHEAD", road_sign_type::TRAM_CROSSING_AHEAD);
  road_sign_type_e.value("SPEED_LIMIT", road_sign_type::SPEED_LIMIT);
  road_sign_type_e.value("END_SPEED_LIMIT", road_sign_type::END_SPEED_LIMIT);
  road_sign_type_e.value("MINIMUM_SPEED", road_sign_type::MINIMUM_SPEED);
  road_sign_type_e.value("SPEED_LIMIT_ZONE", road_sign_type::SPEED_LIMIT_ZONE);
  road_sign_type_e.value("END_SPEED_LIMIT_ZONE", road_sign_type::END_SPEED_LIMIT_ZONE);
  road_sign_type_e.value("ENTERING_BUILT_UP_AREA", road_sign_type::ENTERING_BUILT_UP_AREA);
  road_sign_type_e.value("LEAVING_BUILT_UP_AREA", road_sign_type::LEAVING_BUILT_UP_AREA);
  road_sign_type_e.value("CYCLE_LANE", road_sign_type::CYCLE_LANE);
  road_sign_type_e.value("END_CYCLE_LANE", road_sign_type::END_CYCLE_LANE);
  road_sign_type_e.value("RESERVED_BICYCLE", road_sign_type::RESERVED_BICYCLE);
  road_sign_type_e.value("END_RESERVED_BICYCLE", road_sign_type::END_RESERVED_BICYCLE);
  road_sign_type_e.value("RESERVED_PEDESTRIAN", road_sign_type::RESERVED_PEDESTRIAN);
  road_sign_type_e.value("END_RESERVED_PEDESTRIAN", road_sign_type::END_RESERVED_PEDESTRIAN);
  road_sign_type_e.value("RESERVED_PUBLIC_TRANSPORT", road_sign_type::RESERVED_PUBLIC_TRANSPORT);
  road_sign_type_e.value("END_RESERVED_PUBLIC_TRANSPORT", road_sign_type::END_RESERVED_PUBLIC_TRANSPORT);
  road_sign_type_e.value("RESERVED_TRAM", road_sign_type::RESERVED_TRAM);
  road_sign_type_e.value("END_RESERVED_TRAM", road_sign_type::END_RESERVED_TRAM);
  road_sign_type_e.value("RESERVED_EQUESTRIAN", road_sign_type::RESERVED_EQUESTRIAN);
  road_sign_type_e.value("END_RESERVED_EQUESTRIAN", road_sign_type::END_RESERVED_EQUESTRIAN);
  road_sign_type_e.value("DANGER_TWO_WAY_TRAFFIC", road_sign_type::DANGER_TWO_WAY_TRAFFIC);
  road_sign_type_e.value("OVERTAKE_SEGMENT", road_sign_type::OVERTAKE_SEGMENT);
  road_sign_type_e.value("DOUBLE_OVERTAKE_SEGMENT", road_sign_type::DOUBLE_OVERTAKE_SEGMENT);
  road_sign_type_e.value("END_NO_OVERTAKING", road_sign_type::END_NO_OVERTAKING);
  road_sign_type_e.value("SWITCH_ON_HEADLAMPS", road_sign_type::SWITCH_ON_HEADLAMPS);
  road_sign_type_e.value("SWITCH_OFF_HEADLAMPS", road_sign_type::SWITCH_OFF_HEADLAMPS);
  road_sign_type_e.value("TUNNEL", road_sign_type::TUNNEL);
  road_sign_type_e.value("BEACON", road_sign_type::BEACON);
  road_sign_type_e.value("BEACON_TURN_RIGHT", road_sign_type::BEACON_TURN_RIGHT);
  road_sign_type_e.value("BEACON_TURN_LEFT", road_sign_type::BEACON_TURN_LEFT);
  road_sign_type_e.value("MAXIMUM_HEIGHT", road_sign_type::MAXIMUM_HEIGHT);
  road_sign_type_e.value("BUS_STOP", road_sign_type::BUS_STOP);
  road_sign_type_e.value("END_OF_OBLIGATION", road_sign_type::END_OF_OBLIGATION);
  road_sign_type_e.value("MINIMUM_DISTANCE", road_sign_type::MINIMUM_DISTANCE);
  road_sign_type_e.value("SNOW_CHAIN_OBLIGATORY", road_sign_type::SNOW_CHAIN_OBLIGATORY);
  road_sign_type_e.value("UNKNOWN_SIGN", road_sign_type::UNKNOWN_SIGN);

  auto road_sign_c = py::class_<road_sign>(
      m, "RoadSign", "Describes a detected road sign and its distance to the vehicle that detected it");
  road_sign_c.def_readonly("id", &road_sign::id, "The ID of the road sign in SCANeR");
  road_sign_c.def_readonly("name", &road_sign::name, "The name of the road sign set in the scenario");
  road_sign_c.def_readonly("distance", &road_sign::distance,
                           "The distance of the road sign to the vehicle which detected it");
  road_sign_c.def_readonly("type", &road_sign::type, "The type of road sign, there are 87 possible road sign types");
  road_sign_c.def_readonly("value", &road_sign::value,
                           "The value displayed on this road sign, in cases such as speed limit signs");

  vhl.def("get_road_signs_in_lane", &Vehicle::get_road_signs_in_lane, py::return_value_policy::reference,
          "Returns a list of road signs in the same lane detected by a sensor attached to the vehicle. For this to "
          "work, the vehicle must have a sensor which is set to output target anchor points and logical traffic infos, "
          "with vertical road signs, horizontal road signs and lanes among its selected targets.");
  vhl.def("get_nearest_road_sign_in_lane", &Vehicle::get_nearest_road_sign_in_lane, py::return_value_policy::reference,
          "Returns the nearest road sign in the same lane detected by a sensor attached to the vehicle. For this to "
          "work, the vehicle must have a sensor which is set to output target anchor points and logical traffic infos, "
          "with vertical road signs, horizontal road signs and lanes among its selected targets.");

  auto traffic_light_color_e =
      py::enum_<traffic_light_color>(m, "TrafficLightColor", "Describes the colour of a traffic light");
  traffic_light_color_e.value("OFF", traffic_light_color::OFF);
  traffic_light_color_e.value("GREEN", traffic_light_color::GREEN);
  traffic_light_color_e.value("YELLOW", traffic_light_color::YELLOW);
  traffic_light_color_e.value("YELLOW_AND_RED", traffic_light_color::YELLOW_AND_RED);
  traffic_light_color_e.value("RED", traffic_light_color::RED);
  traffic_light_color_e.value("UNDEFINED_COLOR", traffic_light_color::UNDEFINED_COLOR);

  auto traffic_light_c = py::class_<traffic_light>(
      m, "TrafficLight", "Describes a detected traffic light and its distance to the vehicle that detected it");
  traffic_light_c.def_readonly("id", &traffic_light::id, "The ID of the traffic light in SCANeR");
  traffic_light_c.def_readonly("name", &traffic_light::name, "The name of the traffic light set in the scenario");
  traffic_light_c.def_readonly("distance", &traffic_light::distance,
                               "The distance of the traffic light to the vehicle which detected it");
  traffic_light_c.def_readonly(
      "color", &traffic_light::color,
      "The colour of the traffic light: OFF, GREEN, YELLOW, YELLOW_AND_RED, RED or UNDEFINED_COLOR");
  traffic_light_c.def_readonly("flashing", &traffic_light::flashing, "True iff the traffic light is blinking");

  vhl.def("get_traffic_lights_in_lane", &Vehicle::get_traffic_lights_in_lane, py::return_value_policy::reference,
          "Returns a list of traffic lights in the same lane detected by a sensor attached to the vehicle. For this to "
          "work, the vehicle must have a sensor which is set to output target anchor points and logical traffic infos, "
          "with traffic lights and lanes among its selected targets.");
  vhl.def("get_nearest_traffic_light_in_lane", &Vehicle::get_nearest_traffic_light_in_lane,
          py::return_value_policy::reference,
          "Returns the nearest traffic light in the same lane detected by a sensor attached to the vehicle. For this "
          "to work, the vehicle must have a sensor which is set to output target anchor points and logical traffic "
          "infos, with traffic lights and lanes among its selected targets.");

  auto numerical_command_c = py::class_<numerical_command>(
      m, "NumericalCommand",
      "Describes a command that has been given to a vehicle, where the command is based on a float.");
  numerical_command_c.def_readonly("active", &numerical_command::active, "Only true when a command is in effect.");
  numerical_command_c.def_readonly("value", &numerical_command::value,
                                   "Gives the value that's been assigned to this command.");
}
