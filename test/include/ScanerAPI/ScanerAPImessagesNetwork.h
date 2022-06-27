/*
 * File : ScanerAPImessagesNetwork.h
 * Copyright (C) 2001, Renault all rights reserved
 * SCANeR II is distributed by OKTAL S.A.
 */


/**
 * Enum DataTypesId
 * Identifier for each data type to facilitate sorting of vector v2xDataTypes.
 **/
typedef enum 
{
	SC_INT,		/*Integer data type identifier.*/
	SC_FLOAT,		/*Float data type identifier.*/
	SC_DOUBLE,		/*Double data type identifier.*/
	SC_STRING		/*String data type identifier.*/
} DataTypesId;

/**
 * Enum Status
 * The status of a process.
 **/
typedef enum 
{
	SC_DEAD,		/*The process does not exist any more.*/
	SC_DAEMON,		/*The process exists, the hardware configuration data has been loaded.*/
	SC_LOADED,		/*The exercise specific data has been loaded.*/
	SC_PAUSED,		/*The process is ready but in a paused mode.*/
	SC_READY,		/*The process is ready to run, but is frozen. A [GO] message allows it to start with no delay.*/
	SC_RUNNING		/*The process is running.*/
} Status;

/**
 * Enum ErrorLevel
 * Level of an error message sent via 'Error' interface
 **/
typedef enum 
{
	SC_NOTIFY,
	SC_DEBUG,
	SC_WARNING,
	SC_ERROR
} ErrorLevel;

/**
 * Enum TrafficLightState
 * Status for each light of a traffic light.
 **/
typedef enum 
{
	SC_OFF,
	SC_ON,
	SC_BLINKING
} TrafficLightState;

/**
 * Enum FrequencyWatchingMode
 * The mode of frequency watching.
 **/
typedef enum 
{
	SC_NONE,		/*No frequency watching.*/
	SC_FREE,		/*Free frequency.*/
	SC_FIXED,		/*Fixed frequency (compared to desired frequency).*/
	SC_SCHEDULED		/*External scheduled mode.*/
} FrequencyWatchingMode;

/**
 * Enum VehicleState
 * Status for each vehicle.
 **/
typedef enum 
{
	SC_INVISIBLE,		/*This vehicle is not visible, and does not participate into the traffic.*/
	SC_VISIBLE,		/*This vehicle is visible, but does not participate into the traffic.*/
	SC_ALIVE,		/*This vehicle is visible and participates into the traffic.*/
	SC_GHOST		/*This vehicle is visible and ignored by the traffic.*/
} VehicleState;

/**
 * Enum RepositionState
 * Repositioning state for a vehicle.
 **/
typedef enum 
{
	SC_NORMAL,		/*Vehicle is not being repositioned.*/
	SC_FREEZE,		/*The vehicle is frozen.*/
	SC_DISPLACE,		/*This vehicle is being displaced.*/
	SC_DISAPEAR		/*This vehicle has disapeared (GHOST state) instead being displaced. Usually only for autonomous vehicle on collision with an interactive one*/
} RepositionState;

/**
 * Enum RepositionBehaviour
 * Repositioning behaviour for a vehicle.
 **/
typedef enum 
{
	SC_FREEZE_AND_DISPLACE,		/*Vehicle will be frozen and then displace is reposition is needed.*/
	SC_FREEZE_ONLY		/*The vehicle will be frozen.*/
} RepositionBehaviour;

/**
 * Enum RepositionLaneMode
 * Algorithm use to find the lane where to reposition a vehicle.
 **/
typedef enum 
{
	SC_VEHICLE_ORIENTED		/*Used lane is the same orientation than the vehicle.*/
} RepositionLaneMode;

/**
 * Enum SensorType
 * Types of sensors that can be assigned to a vehicle.
 **/
typedef enum 
{
	SC_LASERMETER,		/*Laser Meter.*/
	SC_CAMERA,		/*Additional cameras.*/
	SC_GPS		/*Vehicle GPS.*/
} SensorType;

/**
 * Enum LightState
 * Possible state of a vehicle light
 **/
typedef enum 
{
	SC_LIGHT_OFF,		/*Light off.*/
	SC_LIGHT_ON,		/*Light on.*/
	SC_LIGHT_AUTO		/*Special value when the light state was forced by script, these state mean the light get it state computed automaticaly again.*/
} LightState;

/**
 * Enum TrajType
 * Type of trajectory for lane change manoeuvre
 **/
typedef enum 
{
	SC_SIMPLE,		/*Classic trajectory, lane by lane*/
	SC_DIRECT		/*Direct trajectory through all lanes to go to the destination lane*/
} TrajType;

/**
 * Enum RuleAccelType
 * Type of acceleration for speed computation
 **/
typedef enum 
{
	SC_PRIORITY,		/*Acceleration to respect priority*/
	SC_SIGN,		/*Acceleration to respect road sign*/
	SC_EMERGENCY_PEDESTRIAN,		/*Acceleration in emergency pedestrian*/
	SC_EMERGENCY_VEHICLE,		/*Acceleration in emergency vehicle*/
	SC_REGULATION,		/*Regulation acceleration*/
	SC_CURVE,		/*Acceleration due to road curvature*/
	SC_FOLLOW		/*Acceleration to follow next vehicle*/
} RuleAccelType;

/**
 * Enum GoalEndBehavior
 * Vehicle or pedestrian behavior after reaching all goals
 **/
typedef enum 
{
	SC_CONTINUE,		/*Continue with the same speed and orientation*/
	SC_AUTONOMOUS,		/*Go to autonomous state*/
	SC_STOP,		/*Stay at the final position*/
	SC_REMOVE		/*Disappear from the simulation*/
} GoalEndBehavior;

/**
 * Enum StopReason
 * Simulation's stopping reason
 **/
typedef enum 
{
	SC_USER,		/*Stopped by user*/
	SC_SCENARIO_END,		/*Stopped by scenario script*/
	SC_SCENARIO_STOP,		/*Stopped when scenario's stop criterion reached*/
	SC_VEHICLE_STOP,		/*Stopped when vehicle's stop criterion reached*/
	SC_DYNAMIC_MODEL_FAILURE,		/*Stopped because of dynamic model failure*/
	SC_SENSOR_STOP,		/*Stopped by a sensor*/
	SC_PROCESS_FAILURE,		/*Stopped because of process failure*/
	SC_DRIVER_ERROR,		/*Stopped because of Driver's plugin error*/
	SC_END_OF_REPLAY,		/*Stopped at the end of record's replay */
	SC_UNKNOWN		/*undefined stop reason*/
} StopReason;

/**
 * Enum VehicleStateManeuver
 * Maneuver state of a vehicle
 **/
typedef enum 
{
	SC_DRIVE_ON,		/*Maneuver state drive on*/
	SC_OVERTAKE,		/*Maneuver state overtake*/
	SC_CHANGE_LANE,		/*Maneuver state change lane*/
	SC_RESERVED_1,		/*Reserved*/
	SC_RESERVED_2,		/*Reserved*/
	SC_RESERVED_3,		/*Reserved*/
	SC_PRE_OVERTAKE,		/*Maneuver state pre overtake*/
	SC_PRE_CHANGE_LANE,		/*Maneuver state pre change line*/
	SC_BLOCKED,		/*Maneuver state blocked*/
	SC_NOT_ALLOWED_LANE,		/*Maneuver state not allowed lane*/
	SC_FOLLOW_GOAL,		/*Maneuver state follow goal*/
	SC_STOPPED		/*Maneuver state stop*/
} VehicleStateManeuver;

/**
 * Enum TypeOfDriver 
 * possible types for not associated driver at the start of the simulation
 **/
typedef enum 
{
	SC_NO_DRIVER,		/* no driver*/
	SC_TRAFFIC_DRIVER,		/*driver controled by traffic*/
	SC_VIRTUAL_DRIVER,		/*virtual driver*/
	SC_HUMAN_DRIVER,		/*human driver*/
	SC_MIXED_DRIVER,		/*mixed driver*/
	SC_EXTERNAL_DRIVER		/*driver type not defined in SCANeR*/
} TypeOfDriver ;

/**
 * Enum SensorDetectedTargetType
 * the type of the detected target by a sensor
 **/
typedef enum 
{
	SC_STATIC_OBJECT,
	SC_DYNAMIC_OBJECT,
	SC_CARS,
	SC_PEDESTRIANS,
	SC_BICYCLES,
	SC_MOTORCYCLE,
	SC_TRUCKS,
	SC_ANIMALS,
	SC_TRAFFICLIGHT,
	SC_VERTICAL_ROAD_SIGN,
	SC_HORIZONTAL_ROAD_SIGN,
	SC_TRAILER,
	SC_BARRIER,
	SC_BUS,
	SC_TRAIN
} SensorDetectedTargetType;

/**
 * Enum SensorAnchorPoint
 * Localization of the detected point
 **/
typedef enum 
{
	SC_UNDEFINED,		/*undefined*/
	SC_REAR_RIGHT_EDGE,		/*rear right edge*/
	SC_REAR_LEFT_EDGE,		/*rear left edge*/
	SC_FRONT_RIGHT_EDGE,		/*front right edge*/
	SC_FRONT_LEFT_EDGE,		/*front left edge=*/
	SC_REAR_FACE_CENTER,		/*rear face center*/
	SC_FRONT_FACE_CENTER,		/*front face center*/
	SC_RIGHT_FACE_CENTER,		/*right face center*/
	SC_LEFT_FACE_CENTER,		/*left face center*/
	SC_UNUSED,		/*unused*/
	SC_UNKNOWN_FRONT_FACE_RIGHT,		/*unknown front face right*/
	SC_UNKNOWN_FRONT_FACE_LEFT,		/*unknown front face left*/
	SC_UNKNOWN_REAR_FACE_RIGHT,		/*unknown rear face right*/
	SC_UNKNOWN_REAR_FACE_LEFT,		/*unknown rear face left*/
	SC_UNKNOWN_RIGHT_FACE_FRONT,		/*unknown right face front*/
	SC_UNKNOWN_LEFT_FACE_FRONT,		/*unknown left face front*/
	SC_UNKNOWN_RIGHT_FACE_REAR,		/*unknown right face rear*/
	SC_UNKNOWN_LEFT_FACE_REAR		/*unknown left face rear*/
} SensorAnchorPoint;

/**
 * Enum SensorReferenceFrame
 * Reference frame in which are given the positions.
 **/
typedef enum 
{
	SC_WORLD,
	SC_VEHICLE,
	SC_SENSOR
} SensorReferenceFrame;

/**
 * Method RqAck
 * Ask process to send an acknowledgement.
 **/
#define NETWORK_ISESSION_RQACK "Network/ISession/RqAck"

/**
 * Method Ack
 * Send an acknowledgement.
 **/
#define NETWORK_ISESSION_ACK "Network/ISession/Ack"

/**
 * Method RqState
 * Ask for the process status.
 **/
#define NETWORK_ISESSION_RQSTATE "Network/ISession/RqState"

/**
 * Method State
 * Send the process status.
 **/
#define NETWORK_ISESSION_STATE "Network/ISession/State"

/**
 * Method RqFile
 * Ask a master process the name of the current file.
 **/
#define NETWORK_ISESSION_RQFILE "Network/ISession/RqFile"

/**
 * Method File
 * Send the current filename.
 **/
#define NETWORK_ISESSION_FILE "Network/ISession/File"

/**
 * Method Error
 * Send an  error message to a console.
 **/
#define NETWORK_ISESSION_ERROR "Network/ISession/Error"

/**
 * Method RqVersion
 * Ask for the communication protocol version of the other process.
 **/
#define NETWORK_ISESSION_RQVERSION "Network/ISession/RqVersion"

/**
 * Method Version
 * Send the version of this process.
 **/
#define NETWORK_ISESSION_VERSION "Network/ISession/Version"

/**
 * Method Load
 * Ask a process to load a simulation. Resulting status must be LOADED.
 **/
#define NETWORK_ISESSION_LOAD "Network/ISession/Load"

/**
 * Method Start
 * Ask a process to start the simulation. Resulting status must be PAUSED.
 **/
#define NETWORK_ISESSION_START "Network/ISession/Start"

/**
 * Method Init
 * Ask a process to initialise a simulation. Resulting status must be READY.
 **/
#define NETWORK_ISESSION_INIT "Network/ISession/Init"

/**
 * Method Go
 * Ask a process to go real-time the simulation. Resulting status must be RUNNING.
 **/
#define NETWORK_ISESSION_GO "Network/ISession/Go"

/**
 * Method Pause
 * Ask a process to pause the simulation. Resulting status must be PAUSED.
 **/
#define NETWORK_ISESSION_PAUSE "Network/ISession/Pause"

/**
 * Method Stop
 * Ask a process to stop the simulation. Resulting status must be RUNNING.
 **/
#define NETWORK_ISESSION_STOP "Network/ISession/Stop"

/**
 * Method Unload
 * Ask a process to unload the databases. Resulting status must be DAEMON.
 **/
#define NETWORK_ISESSION_UNLOAD "Network/ISession/Unload"

/**
 * Method Kill
 * Ask a process to exit. There is no resulting status, but other processes can see it as DEAD.
 **/
#define NETWORK_ISESSION_KILL "Network/ISession/Kill"

/**
 * Method DeadProcess
 * Update the process state to DEAD.
 **/
#define NETWORK_ISESSION_DEADPROCESS "Network/ISession/DeadProcess"

/**
 * Method Environment
 * Ask process to reload an environment variable.
 **/
#define NETWORK_ISESSION_ENVIRONMENT "Network/ISession/Environment"

/**
 * Method Frequency
 * Frequency measurement message.
 **/
#define NETWORK_ISESSION_FREQUENCY "Network/ISession/Frequency"

/**
 * Method RecordPath
 * Force record path to specified path. Must be sent before each Start.
 **/
#define NETWORK_ISESSION_RECORDPATH "Network/ISession/RecordPath"

/**
 * Method BeginFrame
 * Mark a start a a simulation frame for a module
 **/
#define NETWORK_ISESSION_BEGINFRAME "Network/ISession/BeginFrame"

/**
 * Method EndFrame
 * Mark a start a a simulation frame for a module
 **/
#define NETWORK_ISESSION_ENDFRAME "Network/ISession/EndFrame"

/**
 * Method OfflineRatio
 * Time ratio applied by the OfflineScheduler on the other modules for scheduling
 **/
#define NETWORK_ISESSION_OFFLINERATIO "Network/ISession/OfflineRatio"

/**
 * Method VehicleUpdate
 * This message will be sent by the Traffic process to update vehicles informations in other processes.
 **/
#define NETWORK_IVEHICLE_VEHICLEUPDATE "Network/IVehicle/VehicleUpdate"

/**
 * Method VehicleCreate
 * Create a vehicle in real time.
 **/
#define NETWORK_IVEHICLE_VEHICLECREATE "Network/IVehicle/VehicleCreate"

/**
 * Method VehicleDel
 * Delete a vehicle in real time (not supported by Visual System today).
 **/
#define NETWORK_IVEHICLE_VEHICLEDEL "Network/IVehicle/VehicleDel"

/**
 * Method VehicleSetState
 * Modify the state of a vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSTATE "Network/IVehicle/VehicleSetState"

/**
 * Method VehicleMove
 * Move a vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLEMOVE "Network/IVehicle/VehicleMove"

/**
 * Method VehicleSetMaxSpeed
 * Set the maximum speed for a vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLESETMAXSPEED "Network/IVehicle/VehicleSetMaxSpeed"

/**
 * Method VehicleSetItinerary
 * Add or replace the itinerary of a vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLESETITINERARY "Network/IVehicle/VehicleSetItinerary"

/**
 * Method VehicleSetPriorityBehaviour
 * Set vehicle priority observing. 0:does not observe priority. 1:observes priority.
 **/
#define NETWORK_IVEHICLE_VEHICLESETPRIORITYBEHAVIOUR "Network/IVehicle/VehicleSetPriorityBehaviour"

/**
 * Method VehicleSetSignBehaviour
 * Set vehicle sign observing. 0: does not observe signs. 1: observes signs
 **/
#define NETWORK_IVEHICLE_VEHICLESETSIGNBEHAVIOUR "Network/IVehicle/VehicleSetSignBehaviour"

/**
 * Method VehicleSetOvertakingRisk
 * Set overtaking risk level:(value between -1 and 2
 **/
#define NETWORK_IVEHICLE_VEHICLESETOVERTAKINGRISK "Network/IVehicle/VehicleSetOvertakingRisk"

/**
 * Method VehicleSetSpeedLimitRisk
 * Set speed limit risk level:(value between 0.1 and 2
 **/
#define NETWORK_IVEHICLE_VEHICLESETSPEEDLIMITRISK "Network/IVehicle/VehicleSetSpeedLimitRisk"

/**
 * Method VehicleSetPriorityLevel
 * Set vehicle priority level.0: low priority (standard),1: medium priority (ambulance...),2: high priority (police...).
 **/
#define NETWORK_IVEHICLE_VEHICLESETPRIORITYLEVEL "Network/IVehicle/VehicleSetPriorityLevel"

/**
 * Method VehicleSetQueenId
 * In swarm traffic, attaches vehicle vhlId to queen vhlQueenId with default parameters of swarm
 **/
#define NETWORK_IVEHICLE_VEHICLESETQUEENID "Network/IVehicle/VehicleSetQueenId"

/**
 * Method VehicleSetAsQueen
 * Creates a new swarm with vehicle vhlId as queen.
 **/
#define NETWORK_IVEHICLE_VEHICLESETASQUEEN "Network/IVehicle/VehicleSetAsQueen"

/**
 * Method VehicleSetSwarmDisappearFrontRadius
 * For swarm/queen vhlId, sets the distance from which vehicles will be recycled in the swarm, in front.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMDISAPPEARFRONTRADIUS "Network/IVehicle/VehicleSetSwarmDisappearFrontRadius"

/**
 * Method VehicleSetSwarmAppearFrontRadius
 * For swarm/queen vhlId, sets the minimum distance from which vehicles will appear after recycling in the swarm, in front.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMAPPEARFRONTRADIUS "Network/IVehicle/VehicleSetSwarmAppearFrontRadius"

/**
 * Method VehicleSetSwarmDisappearBackRadius
 * For swarm/queen vhlId, sets the distance from which vehicles will be recycled in the swarm, in the rear.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMDISAPPEARBACKRADIUS "Network/IVehicle/VehicleSetSwarmDisappearBackRadius"

/**
 * Method VehicleSetSwarmAppearBackRadius
 * For swarm/queen vhlId, sets the minimum distance from which vehicles will appear after recycling in the swarm, in the back.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMAPPEARBACKRADIUS "Network/IVehicle/VehicleSetSwarmAppearBackRadius"

/**
 * Method VehicleSetSwarmFrontAppearFactor
 * For swarm/queen vhlId, sets the percentage of vehicle that will appear in the front half. value is in [0,1]
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMFRONTAPPEARFACTOR "Network/IVehicle/VehicleSetSwarmFrontAppearFactor"

/**
 * Method VehicleSetSafetyTime
 * Set vehicle safety time, in seconds, in front.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSAFETYTIME "Network/IVehicle/VehicleSetSafetyTime"

/**
 * Method VehicleCollision
 * Informs the simulation of a collision. The speed is the vector of the speed relative to the obstacle.collisionObjId is vehicle Id of colliding vehicle or -1 if static collision object.
 **/
#define NETWORK_IVEHICLE_VEHICLECOLLISION "Network/IVehicle/VehicleCollision"

/**
 * Method VehicleForcePhasePullOut
 * Ask a vehicle to change lane to the left
 **/
#define NETWORK_IVEHICLE_VEHICLEFORCEPHASEPULLOUT "Network/IVehicle/VehicleForcePhasePullOut"

/**
 * Method VehicleForcePhaseFilterIn
 * Ask a vehicle to change lane to the right
 **/
#define NETWORK_IVEHICLE_VEHICLEFORCEPHASEFILTERIN "Network/IVehicle/VehicleForcePhaseFilterIn"

/**
 * Method VehicleForcePhaseDriveOn
 * Ask a vehicle to stay on its lane
 **/
#define NETWORK_IVEHICLE_VEHICLEFORCEPHASEDRIVEON "Network/IVehicle/VehicleForcePhaseDriveOn"

/**
 * Method VehicleFreePhaseDriveOn
 * Resume to default lane selection behaviour for a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEFREEPHASEDRIVEON "Network/IVehicle/VehicleFreePhaseDriveOn"

/**
 * Method VehicleMaxAcceleration
 * Set the maximum acceleration for a vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLEMAXACCELERATION "Network/IVehicle/VehicleMaxAcceleration"

/**
 * Method SendRadar
 * Sends information on the vehicles detected by radar #radarId..
 **/
#define NETWORK_IVEHICLE_SENDRADAR "Network/IVehicle/SendRadar"

/**
 * Method VehicleSetSwarmQueenDirectionFactor
 * Swarm traffic: Probability to be repositionned in the same direction as the queen vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMQUEENDIRECTIONFACTOR "Network/IVehicle/VehicleSetSwarmQueenDirectionFactor"

/**
 * Method VehicleOutOfRoadState
 * Set out of road state
 **/
#define NETWORK_IVEHICLE_VEHICLEOUTOFROADSTATE "Network/IVehicle/VehicleOutOfRoadState"

/**
 * Method VehicleOutOfRoadBehaviour
 * Configures the way of the specified interactive vehicle behaves when it leaves the road.
 **/
#define NETWORK_IVEHICLE_VEHICLEOUTOFROADBEHAVIOUR "Network/IVehicle/VehicleOutOfRoadBehaviour"

/**
 * Method VehicleOutOfRoadActivationRequest
 * Activate the out of road fonctionality for the specified interactive vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLEOUTOFROADACTIVATIONREQUEST "Network/IVehicle/VehicleOutOfRoadActivationRequest"

/**
 * Method VehicleOutOfRoadRepositioning
 * Repositioning of the interactive vehicle after an out of road.
 **/
#define NETWORK_IVEHICLE_VEHICLEOUTOFROADREPOSITIONING "Network/IVehicle/VehicleOutOfRoadRepositioning"

/**
 * Method VehicleCollisionState
 * Set collision state
 **/
#define NETWORK_IVEHICLE_VEHICLECOLLISIONSTATE "Network/IVehicle/VehicleCollisionState"

/**
 * Method VehicleCollisionBehaviour
 * Configures the way of the specified interactive vehicle behaves when it collides another vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLECOLLISIONBEHAVIOUR "Network/IVehicle/VehicleCollisionBehaviour"

/**
 * Method VehicleCollisionActivationRequest
 * Activate the collision fonctionality for the specified interactive vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLECOLLISIONACTIVATIONREQUEST "Network/IVehicle/VehicleCollisionActivationRequest"

/**
 * Method VehicleCollisionRepositioning
 * Repositioning of the interactive vehicle after an collision with another vehicle.
 **/
#define NETWORK_IVEHICLE_VEHICLECOLLISIONREPOSITIONING "Network/IVehicle/VehicleCollisionRepositioning"

/**
 * Method VehicleSetAdherence
 * Set an adherence coefficient under one wheel of an specified interactive vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETADHERENCE "Network/IVehicle/VehicleSetAdherence"

/**
 * Method VehicleSetSpeedObligatory
 * Set the obligatory speed for the autonomous vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETSPEEDOBLIGATORY "Network/IVehicle/VehicleSetSpeedObligatory"

/**
 * Method VehicleSetAccelerationObligatory
 * Set the obligatory acceleration for the autonomous vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETACCELERATIONOBLIGATORY "Network/IVehicle/VehicleSetAccelerationObligatory"

/**
 * Method VehicleTrailerDetach
 * Ask a trailer to detach from its tractor with a specified behaviour.
 **/
#define NETWORK_IVEHICLE_VEHICLETRAILERDETACH "Network/IVehicle/VehicleTrailerDetach"

/**
 * Method VehicleForcePhasePullOutObligatory
 * Ask a vehicle to change lane to the left without taking in account the vehicles around
 **/
#define NETWORK_IVEHICLE_VEHICLEFORCEPHASEPULLOUTOBLIGATORY "Network/IVehicle/VehicleForcePhasePullOutObligatory"

/**
 * Method VehicleForcePhaseFilterInObligatory
 * Ask a vehicle to change lane to the right without taking without taking in account the vehicles around
 **/
#define NETWORK_IVEHICLE_VEHICLEFORCEPHASEFILTERINOBLIGATORY "Network/IVehicle/VehicleForcePhaseFilterInObligatory"

/**
 * Method VehicleOverrideLightState
 * Override the lights state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEOVERRIDELIGHTSTATE "Network/IVehicle/VehicleOverrideLightState"

/**
 * Method VehicleLaneLateralShift
 * Add a lateral shift to a vehicle in meters or in lanes
 **/
#define NETWORK_IVEHICLE_VEHICLELANELATERALSHIFT "Network/IVehicle/VehicleLaneLateralShift"

/**
 * Method VehicleSetTech
 * Modify a technical features of a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETTECH "Network/IVehicle/VehicleSetTech"

/**
 * Method VehicleDefect
 * Add a defect to an interactive vehicle. The defects value is modulated by the multiplying coefficient contained in this network message
 **/
#define NETWORK_IVEHICLE_VEHICLEDEFECT "Network/IVehicle/VehicleDefect"

/**
 * Method VehicleWheelDefect
 * Add a defect to an wheel of an interactive vehicle. The defects value is modulated by the multiplying coefficient contained in this network message
 **/
#define NETWORK_IVEHICLE_VEHICLEWHEELDEFECT "Network/IVehicle/VehicleWheelDefect"

/**
 * Method PedestrianSetGoal
 * set a new goal for the pedestrian
 **/
#define NETWORK_IVEHICLE_PEDESTRIANSETGOAL "Network/IVehicle/PedestrianSetGoal"

/**
 * Method PedestrianAskToCross
 * give cross order to pedestrian
 **/
#define NETWORK_IVEHICLE_PEDESTRIANASKTOCROSS "Network/IVehicle/PedestrianAskToCross"

/**
 * Method PedestrianCross
 * Sent when a pedestrian start or stop crossing road
 **/
#define NETWORK_IVEHICLE_PEDESTRIANCROSS "Network/IVehicle/PedestrianCross"

/**
 * Method PedestrianWalk
 * Pedestrian start walking
 **/
#define NETWORK_IVEHICLE_PEDESTRIANWALK "Network/IVehicle/PedestrianWalk"

/**
 * Method PedestrianRun
 * Pedestrian start running
 **/
#define NETWORK_IVEHICLE_PEDESTRIANRUN "Network/IVehicle/PedestrianRun"

/**
 * Method PedestrianStop
 * Pedestrian stop moving
 **/
#define NETWORK_IVEHICLE_PEDESTRIANSTOP "Network/IVehicle/PedestrianStop"

/**
 * Method PedestrianAutonavigation
 * active/deactive autonavigation
 **/
#define NETWORK_IVEHICLE_PEDESTRIANAUTONAVIGATION "Network/IVehicle/PedestrianAutonavigation"

/**
 * Method PedestrianSetWalkSpeed
 * set the walking speed of the pedestrian
 **/
#define NETWORK_IVEHICLE_PEDESTRIANSETWALKSPEED "Network/IVehicle/PedestrianSetWalkSpeed"

/**
 * Method PedestrianSetRunningSpeed
 * set the running speed of the pedestrian
 **/
#define NETWORK_IVEHICLE_PEDESTRIANSETRUNNINGSPEED "Network/IVehicle/PedestrianSetRunningSpeed"

/**
 * Method VehiclePath
 * Current known vehicle path.
 **/
#define NETWORK_IVEHICLE_VEHICLEPATH "Network/IVehicle/VehiclePath"

/**
 * Method VehicleChangeProcess
 * set the process of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLECHANGEPROCESS "Network/IVehicle/VehicleChangeProcess"

/**
 * Method ExtendedVehicleOutput
 * extended vehicle output values.
 **/
#define NETWORK_IVEHICLE_EXTENDEDVEHICLEOUTPUT "Network/IVehicle/ExtendedVehicleOutput"

/**
 * Method OverrideCustomVehicleInput
 * modify the values of the custom vehicle inputs.
 **/
#define NETWORK_IVEHICLE_OVERRIDECUSTOMVEHICLEINPUT "Network/IVehicle/OverrideCustomVehicleInput"

/**
 * Method CustomVehicleInput
 * echo of the current values of the custom vehicle inputs.
 **/
#define NETWORK_IVEHICLE_CUSTOMVEHICLEINPUT "Network/IVehicle/CustomVehicleInput"

/**
 * Method CustomVehicleOutput
 * the current values of the custom vehicle outputs.
 **/
#define NETWORK_IVEHICLE_CUSTOMVEHICLEOUTPUT "Network/IVehicle/CustomVehicleOutput"

/**
 * Method VehicleOverrideRearWiperState
 * Override the rear wiper state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEOVERRIDEREARWIPERSTATE "Network/IVehicle/VehicleOverrideRearWiperState"

/**
 * Method VehicleOverrideFrontWiperState
 * Override the front wiper state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEOVERRIDEFRONTWIPERSTATE "Network/IVehicle/VehicleOverrideFrontWiperState"

/**
 * Method VehicleSetDetonation
 * send a detonation message of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETDETONATION "Network/IVehicle/VehicleSetDetonation"

/**
 * Method VehicleOverrideSirenState
 * Override the siren state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEOVERRIDESIRENSTATE "Network/IVehicle/VehicleOverrideSirenState"

/**
 * Method VehicleRemoveFromSwarm
 * Remove a vheicle from a swarm
 **/
#define NETWORK_IVEHICLE_VEHICLEREMOVEFROMSWARM "Network/IVehicle/VehicleRemoveFromSwarm"

/**
 * Method VehicleDesactivateSwarm
 * Desactivate a swarm. Be carreful it's not possible to reactivate the swarm once done
 **/
#define NETWORK_IVEHICLE_VEHICLEDESACTIVATESWARM "Network/IVehicle/VehicleDesactivateSwarm"

/**
 * Method AirplaneTakeOff
 * Airplane takeOff
 **/
#define NETWORK_IVEHICLE_AIRPLANETAKEOFF "Network/IVehicle/AirplaneTakeOff"

/**
 * Method AirplaneFollowVehicle
 * Airplane follow an another vehicle
 **/
#define NETWORK_IVEHICLE_AIRPLANEFOLLOWVEHICLE "Network/IVehicle/AirplaneFollowVehicle"

/**
 * Method VehicleSetTimeToVehicle
 * A vehicle follow an another vehicle at a given time
 **/
#define NETWORK_IVEHICLE_VEHICLESETTIMETOVEHICLE "Network/IVehicle/VehicleSetTimeToVehicle"

/**
 * Method VehicleTrajectoryLateralShift
 * set a lateral shift for a vehicle following a trajectory(only racetraffic vehicle support it)
 **/
#define NETWORK_IVEHICLE_VEHICLETRAJECTORYLATERALSHIFT "Network/IVehicle/VehicleTrajectoryLateralShift"

/**
 * Method VehicleTrajectory
 * Set the trajectory to a vehicle (only racetraffic vehicle support it)
 **/
#define NETWORK_IVEHICLE_VEHICLETRAJECTORY "Network/IVehicle/VehicleTrajectory"

/**
 * Method VehicleSpeedFactor
 * Set the speed factor of a vehicle (only racetraffic vehicle support it)
 **/
#define NETWORK_IVEHICLE_VEHICLESPEEDFACTOR "Network/IVehicle/VehicleSpeedFactor"

/**
 * Method VehicleSetAutonomousMode
 * Set the type of autonomous control of a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETAUTONOMOUSMODE "Network/IVehicle/VehicleSetAutonomousMode"

/**
 * Method VehicleAutonomousControlState
 * The state of the autonomous vehicle controls
 **/
#define NETWORK_IVEHICLE_VEHICLEAUTONOMOUSCONTROLSTATE "Network/IVehicle/VehicleAutonomousControlState"

/**
 * Method VehicleSetAutonomousSteeringPositionalControl
 * enables/disables the movement of the steering wheel during autonomous driving
 **/
#define NETWORK_IVEHICLE_VEHICLESETAUTONOMOUSSTEERINGPOSITIONALCONTROL "Network/IVehicle/VehicleSetAutonomousSteeringPositionalControl"

/**
 * Method VehicleSetStrategy
 * Set a descision strategy to a vehicle driver
 **/
#define NETWORK_IVEHICLE_VEHICLESETSTRATEGY "Network/IVehicle/VehicleSetStrategy"

/**
 * Method VehiclePullOut
 * Ask a pull out to a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEPULLOUT "Network/IVehicle/VehiclePullOut"

/**
 * Method VehicleFilterIn
 * Ask a filter in to a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEFILTERIN "Network/IVehicle/VehicleFilterIn"

/**
 * Method VehicleGoToLane
 * Ask a vehicle to move through some lanes
 **/
#define NETWORK_IVEHICLE_VEHICLEGOTOLANE "Network/IVehicle/VehicleGoToLane"

/**
 * Method VehicleSetSpeedRuleState
 * enable/disable longitudinal acceleration rules
 **/
#define NETWORK_IVEHICLE_VEHICLESETSPEEDRULESTATE "Network/IVehicle/VehicleSetSpeedRuleState"

/**
 * Method VehicleSetSpeedTarget
 * enable/disable target speed
 **/
#define NETWORK_IVEHICLE_VEHICLESETSPEEDTARGET "Network/IVehicle/VehicleSetSpeedTarget"

/**
 * Method VehicleSetStayOnLAne
 * enable/disable stay on lane property
 **/
#define NETWORK_IVEHICLE_VEHICLESETSTAYONLANE "Network/IVehicle/VehicleSetStayOnLAne"

/**
 * Method VehicleAddGoal
 * add a goal the vehicle must follow
 **/
#define NETWORK_IVEHICLE_VEHICLEADDGOAL "Network/IVehicle/VehicleAddGoal"

/**
 * Method VehicleGoalEndBehavior
 * specify the end behavior of the goal following state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEGOALENDBEHAVIOR "Network/IVehicle/VehicleGoalEndBehavior"

/**
 * Method VehicleClearGoal
 * Clears the remaning goals of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLECLEARGOAL "Network/IVehicle/VehicleClearGoal"

/**
 * Method VehicleSetSwarmDensity
 * For swarm/queen vhlId, sets the density of the swarm.
 **/
#define NETWORK_IVEHICLE_VEHICLESETSWARMDENSITY "Network/IVehicle/VehicleSetSwarmDensity"

/**
 * Method VehicleSetLaneGap
 * set the parameters gap mean and std deviation for gap computation for vhlId.
 **/
#define NETWORK_IVEHICLE_VEHICLESETLANEGAP "Network/IVehicle/VehicleSetLaneGap"

/**
 * Method VehicleSetLaneGapPeriod
 * set the parameters period and std deviation for gap computation for vhlId.
 **/
#define NETWORK_IVEHICLE_VEHICLESETLANEGAPPERIOD "Network/IVehicle/VehicleSetLaneGapPeriod"

/**
 * Method VehicleState
 * The state of a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESTATE "Network/IVehicle/VehicleState"

/**
 * Method VehicleDriver
 * Specify the driver of a vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEDRIVER "Network/IVehicle/VehicleDriver"

/**
 * Method VehicleSetTrajectoryEndBehavior
 * specify the end behavior of the trajectory following state of the vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLESETTRAJECTORYENDBEHAVIOR "Network/IVehicle/VehicleSetTrajectoryEndBehavior"

/**
 * Method VehicleDriverType
 * specify the type of driver for the specified vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEDRIVERTYPE "Network/IVehicle/VehicleDriverType"

/**
 * Method VehicleFollowTrajectory
 * specify the a trajectory to follow to a driver
 **/
#define NETWORK_IVEHICLE_VEHICLEFOLLOWTRAJECTORY "Network/IVehicle/VehicleFollowTrajectory"

/**
 * Method VehicleAttachTrailer
 * attach the trailer to the specified vehicle
 **/
#define NETWORK_IVEHICLE_VEHICLEATTACHTRAILER "Network/IVehicle/VehicleAttachTrailer"

/**
 * Method VehicleSetSteeringWheelVibration
 * Generate steering wheel vibration
 **/
#define NETWORK_IVEHICLE_VEHICLESETSTEERINGWHEELVIBRATION "Network/IVehicle/VehicleSetSteeringWheelVibration"

/**
 * Method VehicleSetAutonomousSteeringPositionalPID
 * Set the position PID of steering wheel
 **/
#define NETWORK_IVEHICLE_VEHICLESETAUTONOMOUSSTEERINGPOSITIONALPID "Network/IVehicle/VehicleSetAutonomousSteeringPositionalPID"

/**
 * Method ObserverSetPos
 * Set the observer position and mode Observer mode setting.
 **/
#define NETWORK_IOBSERVER_OBSERVERSETPOS "Network/IObserver/ObserverSetPos"

/**
 * Method OBSERVER_SET_FOV
 * Change the field of view
 **/
#define NETWORK_IOBSERVER_OBSERVER_SET_FOV "Network/IObserver/OBSERVER_SET_FOV"

/**
 * Method ObserverSetFrustum
 * Change the frustum
 **/
#define NETWORK_IOBSERVER_OBSERVERSETFRUSTUM "Network/IObserver/ObserverSetFrustum"

/**
 * Method TrafficLightUpdate
 * Normal update send by TrafLights System to everybody.
 **/
#define NETWORK_IROADSIGN_TRAFFICLIGHTUPDATE "Network/IRoadSign/TrafficLightUpdate"

/**
 * Method TrafficLightChange
 * Ask to Traffic Lights System to change a traffic light.
 **/
#define NETWORK_IROADSIGN_TRAFFICLIGHTCHANGE "Network/IRoadSign/TrafficLightChange"

/**
 * Method BarrierUpdate
 * Normal update send by Barriers System to everybody.
 **/
#define NETWORK_IROADSIGN_BARRIERUPDATE "Network/IRoadSign/BarrierUpdate"

/**
 * Method BarrierSetSpeed
 * Set barrier speed
 **/
#define NETWORK_IROADSIGN_BARRIERSETSPEED "Network/IRoadSign/BarrierSetSpeed"

/**
 * Method BarrierSet
 * Set the specified barrier to be opened or closed
 **/
#define NETWORK_IROADSIGN_BARRIERSET "Network/IRoadSign/BarrierSet"

/**
 * Method RoadSignText
 * Set text for a roadsign
 **/
#define NETWORK_IROADSIGN_ROADSIGNTEXT "Network/IRoadSign/RoadSignText"

/**
 * Method TrafficLightGroupCycleChange
 * Change the current animation cycle in a traffic light group
 **/
#define NETWORK_IROADSIGN_TRAFFICLIGHTGROUPCYCLECHANGE "Network/IRoadSign/TrafficLightGroupCycleChange"

/**
 * Method TrafficLightGroupTime
 * Update trafficLightGroup time
 **/
#define NETWORK_IROADSIGN_TRAFFICLIGHTGROUPTIME "Network/IRoadSign/TrafficLightGroupTime"

/**
 * Method WeatherSunColor
 * send or receive Sun Color
 **/
#define NETWORK_IWEATHER_WEATHERSUNCOLOR "Network/IWeather/WeatherSunColor"

/**
 * Method WeatherAmbientColor
 * send/receive Ambient Color
 **/
#define NETWORK_IWEATHER_WEATHERAMBIENTCOLOR "Network/IWeather/WeatherAmbientColor"

/**
 * Method WeatherSkyColor
 * send/receive Sky Color
 **/
#define NETWORK_IWEATHER_WEATHERSKYCOLOR "Network/IWeather/WeatherSkyColor"

/**
 * Method WeatherFogColor
 * send/receive Fog Color
 **/
#define NETWORK_IWEATHER_WEATHERFOGCOLOR "Network/IWeather/WeatherFogColor"

/**
 * Method WeatherAmbientFog
 * send/receive Ambient Fog
 **/
#define NETWORK_IWEATHER_WEATHERAMBIENTFOG "Network/IWeather/WeatherAmbientFog"

/**
 * Method WeatherRain
 * send/receive Rain
 **/
#define NETWORK_IWEATHER_WEATHERRAIN "Network/IWeather/WeatherRain"

/**
 * Method WeatherWind
 * send/receive Wind
 **/
#define NETWORK_IWEATHER_WEATHERWIND "Network/IWeather/WeatherWind"

/**
 * Method WeatherDayHour
 * send/receive Day Hour
 **/
#define NETWORK_IWEATHER_WEATHERDAYHOUR "Network/IWeather/WeatherDayHour"

/**
 * Method WeatherSnow
 * send/receive Snow
 **/
#define NETWORK_IWEATHER_WEATHERSNOW "Network/IWeather/WeatherSnow"

/**
 * Method WeatherSkySaturation
 * send/receive Sky Saturation
 **/
#define NETWORK_IWEATHER_WEATHERSKYSATURATION "Network/IWeather/WeatherSkySaturation"

/**
 * Method WeatherCloudDensity
 * send/receive Cloud Density
 **/
#define NETWORK_IWEATHER_WEATHERCLOUDDENSITY "Network/IWeather/WeatherCloudDensity"

/**
 * Method WeatherLightning
 * send/receive Lightning
 **/
#define NETWORK_IWEATHER_WEATHERLIGHTNING "Network/IWeather/WeatherLightning"

/**
 * Method WeatherWaterOnRoad
 * send/receive Water Accumulation
 **/
#define NETWORK_IWEATHER_WEATHERWATERONROAD "Network/IWeather/WeatherWaterOnRoad"

/**
 * Method WeatherPublicLight
 * send/receive Public Light state
 **/
#define NETWORK_IWEATHER_WEATHERPUBLICLIGHT "Network/IWeather/WeatherPublicLight"

/**
 * Method WeatherSnowOnRoad
 * send/receive Snow Accumulation
 **/
#define NETWORK_IWEATHER_WEATHERSNOWONROAD "Network/IWeather/WeatherSnowOnRoad"

/**
 * Method ModelInput
 * Dynamic Model data input (see ModelShm API)
 **/
#define NETWORK_IMODEL_MODELINPUT "Network/IModel/ModelInput"

/**
 * Method ModelFailure
 * This message is an order which can be sent to the vehicle model.
 **/
#define NETWORK_IMODEL_MODELFAILURE "Network/IModel/ModelFailure"

/**
 * Method ModelPollution
 * Get the pollution from the vehicle model.
 **/
#define NETWORK_IMODEL_MODELPOLLUTION "Network/IModel/ModelPollution"

/**
 * Method SupervisorLaunch
 * Asks the daemons to launch a program with the specified procId, name and arguments
 **/
#define NETWORK_ISUPERVISOR_SUPERVISORLAUNCH "Network/ISupervisor/SupervisorLaunch"

/**
 * Method SupervisorKill
 * Asks the daemons to kill the program procId. procId identifies the id of the program (they are ordered in process.cfg file)
 **/
#define NETWORK_ISUPERVISOR_SUPERVISORKILL "Network/ISupervisor/SupervisorKill"

/**
 * Method SupervisorRefresh
 * Asks the daemons the current state of processes. procId identifies the id of the program (they are ordered in process.cfg file)
 **/
#define NETWORK_ISUPERVISOR_SUPERVISORREFRESH "Network/ISupervisor/SupervisorRefresh"

/**
 * Method SupervisorReload
 * Ask to change configuration
 **/
#define NETWORK_ISUPERVISOR_SUPERVISORRELOAD "Network/ISupervisor/SupervisorReload"

/**
 * Method SupervisorShutdown
 * Ask to shutdown the computer
 **/
#define NETWORK_ISUPERVISOR_SUPERVISORSHUTDOWN "Network/ISupervisor/SupervisorShutdown"

/**
 * Method DaemonProcState
 * Tell the daemon-handled state of a process
 **/
#define NETWORK_IDAEMON_DAEMONPROCSTATE "Network/IDaemon/DaemonProcState"

/**
 * Method CartoRadarBeamDrawCoords
 * Indicates the x,y position of the points defining the radar beam cone of the radar which has the index #indexInRadarArray in the array of radarsP2  P1  P3   \    |    /     \  |  /      P0
 **/
#define NETWORK_ICARTO_CARTORADARBEAMDRAWCOORDS "Network/ICarto/CartoRadarBeamDrawCoords"

/**
 * Method CartoTurnsDrawCoords
 * Indicates the x,y,z position of the points on the left and right side of the road which begin and end the current and next turn of a vehicle of id vhlId
 **/
#define NETWORK_ICARTO_CARTOTURNSDRAWCOORDS "Network/ICarto/CartoTurnsDrawCoords"

/**
 * Method CartoDrawPoint
 * Point to draw in CARTO
 **/
#define NETWORK_ICARTO_CARTODRAWPOINT "Network/ICarto/CartoDrawPoint"

/**
 * Method CartoDrawSegment
 * Sgment to draw in CARTO
 **/
#define NETWORK_ICARTO_CARTODRAWSEGMENT "Network/ICarto/CartoDrawSegment"

/**
 * Method CartoDrawArc
 * Circle arc to draw in CARTO
 **/
#define NETWORK_ICARTO_CARTODRAWARC "Network/ICarto/CartoDrawArc"

/**
 * Method CartoDrawText
 * Text to draw in CARTO
 **/
#define NETWORK_ICARTO_CARTODRAWTEXT "Network/ICarto/CartoDrawText"

/**
 * Method CartoErase
 * Erase the drawing with given id
 **/
#define NETWORK_ICARTO_CARTOERASE "Network/ICarto/CartoErase"

/**
 * Method Load
 * Load a photometric data
 **/
#define NETWORK_ILIGHTSIMULATION_LOAD "Network/ILightSimulation/Load"

/**
 * Method HeadlampParameters
 * Set parameters to a headlamp
 **/
#define NETWORK_ILIGHTSIMULATION_HEADLAMPPARAMETERS "Network/ILightSimulation/HeadlampParameters"

/**
 * Method Activation
 * Handle switch on/off for headlamps
 **/
#define NETWORK_ILIGHTSIMULATION_ACTIVATION "Network/ILightSimulation/Activation"

/**
 * Method GroundGrid
 *  **/
#define NETWORK_ILIGHTSIMULATION_GROUNDGRID "Network/ILightSimulation/GroundGrid"

/**
 * Method LuxMeasure
 *  **/
#define NETWORK_ILIGHTSIMULATION_LUXMEASURE "Network/ILightSimulation/LuxMeasure"

/**
 * Method DisplayString
 *  **/
#define NETWORK_ILIGHTSIMULATION_DISPLAYSTRING "Network/ILightSimulation/DisplayString"

/**
 * Method HeadLampInUse
 *  **/
#define NETWORK_ILIGHTSIMULATION_HEADLAMPINUSE "Network/ILightSimulation/HeadLampInUse"

/**
 * Method EyeLevel
 *  **/
#define NETWORK_ILIGHTSIMULATION_EYELEVEL "Network/ILightSimulation/EyeLevel"

/**
 * Method AmbientLight
 *  **/
#define NETWORK_ILIGHTSIMULATION_AMBIENTLIGHT "Network/ILightSimulation/AmbientLight"

/**
 * Method Calibrate
 *  **/
#define NETWORK_ILIGHTSIMULATION_CALIBRATE "Network/ILightSimulation/Calibrate"

/**
 * Method WallGrid
 *  **/
#define NETWORK_ILIGHTSIMULATION_WALLGRID "Network/ILightSimulation/WallGrid"

/**
 * Method IsoLuxValue
 *  **/
#define NETWORK_ILIGHTSIMULATION_ISOLUXVALUE "Network/ILightSimulation/IsoLuxValue"

/**
 * Method Color
 *  **/
#define NETWORK_ILIGHTSIMULATION_COLOR "Network/ILightSimulation/Color"

/**
 * Method CartoCoord
 *  **/
#define NETWORK_ILIGHTSIMULATION_CARTOCOORD "Network/ILightSimulation/CartoCoord"

/**
 * Method GroundGridV2
 *  **/
#define NETWORK_ILIGHTSIMULATION_GROUNDGRIDV2 "Network/ILightSimulation/GroundGridV2"

/**
 * Method WallGridV2
 *  **/
#define NETWORK_ILIGHTSIMULATION_WALLGRIDV2 "Network/ILightSimulation/WallGridV2"

/**
 * Method DisplayMessage
 *  **/
#define NETWORK_ILIGHTSIMULATION_DISPLAYMESSAGE "Network/ILightSimulation/DisplayMessage"

/**
 * Method BuildWall
 *  **/
#define NETWORK_ILIGHTSIMULATION_BUILDWALL "Network/ILightSimulation/BuildWall"

/**
 * Method MoveWall
 *  **/
#define NETWORK_ILIGHTSIMULATION_MOVEWALL "Network/ILightSimulation/MoveWall"

/**
 * Method ActiveWall
 *  **/
#define NETWORK_ILIGHTSIMULATION_ACTIVEWALL "Network/ILightSimulation/ActiveWall"

/**
 * Method WallZone
 *  **/
#define NETWORK_ILIGHTSIMULATION_WALLZONE "Network/ILightSimulation/WallZone"

/**
 * Method DeleteWall
 *  **/
#define NETWORK_ILIGHTSIMULATION_DELETEWALL "Network/ILightSimulation/DeleteWall"

/**
 * Method SensorList
 *  **/
#define NETWORK_ILIGHTSIMULATION_SENSORLIST "Network/ILightSimulation/SensorList"

/**
 * Method UpdateSensor
 *  **/
#define NETWORK_ILIGHTSIMULATION_UPDATESENSOR "Network/ILightSimulation/UpdateSensor"

/**
 * Method SensorValues
 *  **/
#define NETWORK_ILIGHTSIMULATION_SENSORVALUES "Network/ILightSimulation/SensorValues"

/**
 * Method PickingData
 *  **/
#define NETWORK_ILIGHTSIMULATION_PICKINGDATA "Network/ILightSimulation/PickingData"

/**
 * Method LIGHTSIMULATION_FOG
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_FOG "Network/ILightSimulation/LIGHTSIMULATION_FOG"

/**
 * Method LIGHTSIMULATION_PRODUCT_INFO
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_PRODUCT_INFO "Network/ILightSimulation/LIGHTSIMULATION_PRODUCT_INFO"

/**
 * Method LIGHTSIMULATION_OPTICALFUNCTION_INFO
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_OPTICALFUNCTION_INFO "Network/ILightSimulation/LIGHTSIMULATION_OPTICALFUNCTION_INFO"

/**
 * Method LIGHTSIMULATION_PROJECTOR_INFO
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_PROJECTOR_INFO "Network/ILightSimulation/LIGHTSIMULATION_PROJECTOR_INFO"

/**
 * Method LIGHTSIMULATION_LIGHTSOURCE_INFO
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_LIGHTSOURCE_INFO "Network/ILightSimulation/LIGHTSIMULATION_LIGHTSOURCE_INFO"

/**
 * Method LIGHTSIMULATION_PRODUCT_RUN
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_PRODUCT_RUN "Network/ILightSimulation/LIGHTSIMULATION_PRODUCT_RUN"

/**
 * Method LIGHTSIMULATION_PRODUCT_STOP
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_PRODUCT_STOP "Network/ILightSimulation/LIGHTSIMULATION_PRODUCT_STOP"

/**
 * Method LIGHTSIMULATION_PRODUCT_UPDATE
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_PRODUCT_UPDATE "Network/ILightSimulation/LIGHTSIMULATION_PRODUCT_UPDATE"

/**
 * Method LIGHTSIMULATION_AFS_ERROR
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_AFS_ERROR "Network/ILightSimulation/LIGHTSIMULATION_AFS_ERROR"

/**
 * Method LIGHTSIMULATION_OPTICALFUNCTION_STRATEGY_NAMES
 *  **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_OPTICALFUNCTION_STRATEGY_NAMES "Network/ILightSimulation/LIGHTSIMULATION_OPTICALFUNCTION_STRATEGY_NAMES"

/**
 * Method CockpitLoad
 * Load a cockpit
 **/
#define NETWORK_ILIGHTSIMULATION_COCKPITLOAD "Network/ILightSimulation/CockpitLoad"

/**
 * Method CockpitTransform
 * Apply a transform to the cockpit
 **/
#define NETWORK_ILIGHTSIMULATION_COCKPITTRANSFORM "Network/ILightSimulation/CockpitTransform"

/**
 * Method CockpitShow
 * Show/Hide a cockpit
 **/
#define NETWORK_ILIGHTSIMULATION_COCKPITSHOW "Network/ILightSimulation/CockpitShow"

/**
 * Method LIGHTSIMULATION_ADVANCED_RENDERING
 * Enable/Disable the BRDF rendering and send wet materials parameters
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_ADVANCED_RENDERING "Network/ILightSimulation/LIGHTSIMULATION_ADVANCED_RENDERING"

/**
 * Method VisibilityLine
 *  **/
#define NETWORK_ILIGHTSIMULATION_VISIBILITYLINE "Network/ILightSimulation/VisibilityLine"

/**
 * Method LIGHTSIMULATION_AFS_DOUBLE_PARAMETER
 * Send the attributes of an AFS double parameter.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_AFS_DOUBLE_PARAMETER "Network/ILightSimulation/LIGHTSIMULATION_AFS_DOUBLE_PARAMETER"

/**
 * Method LIGHTSIMULATION_AFS_STRING_PARAMETER
 * Send the attributes of an AFS string parameter.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_AFS_STRING_PARAMETER "Network/ILightSimulation/LIGHTSIMULATION_AFS_STRING_PARAMETER"

/**
 * Method LIGHTSIMULATION_DYNAMIC_LIGHT_STATE
 * Activate/Deactivate a dynamic light.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DYNAMIC_LIGHT_STATE "Network/ILightSimulation/LIGHTSIMULATION_DYNAMIC_LIGHT_STATE"

/**
 * Method LIGHTSIMULATION_DYNAMIC_LIGHT_COLOR
 * Change the color of a dynamic light.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DYNAMIC_LIGHT_COLOR "Network/ILightSimulation/LIGHTSIMULATION_DYNAMIC_LIGHT_COLOR"

/**
 * Method LIGHTSIMULATION_DYNAMIC_LIGHT_POSITION
 * Change the position of a dynamic light.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DYNAMIC_LIGHT_POSITION "Network/ILightSimulation/LIGHTSIMULATION_DYNAMIC_LIGHT_POSITION"

/**
 * Method LIGHTSIMULATION_DYNAMIC_LIGHT_ORIENTATION
 * Change the orientation of a dynamic light.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DYNAMIC_LIGHT_ORIENTATION "Network/ILightSimulation/LIGHTSIMULATION_DYNAMIC_LIGHT_ORIENTATION"

/**
 * Method LIGHTSIMULATION_DYNAMIC_LIGHT_INTENSITY
 * Change the intensity of a dynamic light.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DYNAMIC_LIGHT_INTENSITY "Network/ILightSimulation/LIGHTSIMULATION_DYNAMIC_LIGHT_INTENSITY"

/**
 * Method LIGHTSIMULATION_HEADLIGHT_UPDATED_BY_MODEL
 * Allow/Disallow the control of headlights by MODEL process
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_HEADLIGHT_UPDATED_BY_MODEL "Network/ILightSimulation/LIGHTSIMULATION_HEADLIGHT_UPDATED_BY_MODEL"

/**
 * Method LIGHTSIMULATION_LIGHTLIST_REBUILDED
 * Sent by the visual when the lightlist is rebuilded properly
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_LIGHTLIST_REBUILDED "Network/ILightSimulation/LIGHTSIMULATION_LIGHTLIST_REBUILDED"

/**
 * Method LIGHTSIMULATION_AFS_STRATEGY_INPUT
 * Sent by the afsmanager in debug mode only
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_AFS_STRATEGY_INPUT "Network/ILightSimulation/LIGHTSIMULATION_AFS_STRATEGY_INPUT"

/**
 * Method LIGHTSIMULATION_SET_SCENE
 * Change the scene shape file
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_SCENE "Network/ILightSimulation/LIGHTSIMULATION_SET_SCENE"

/**
 * Method LIGHTSIMULATION_CUSTOM_GROUND_GRID
 * Change the ground grid file
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_CUSTOM_GROUND_GRID "Network/ILightSimulation/LIGHTSIMULATION_CUSTOM_GROUND_GRID"

/**
 * Method LIGHTSIMULATION_LIGHT_UNIT
 * Change the iso curves unit
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_LIGHT_UNIT "Network/ILightSimulation/LIGHTSIMULATION_LIGHT_UNIT"

/**
 * Method LIGHTSIMULATION_SET_VEHICLE
 * Set current test vehicle
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_VEHICLE "Network/ILightSimulation/LIGHTSIMULATION_SET_VEHICLE"

/**
 * Method LIGHTSIMULATION_SET_STREETLIGHTS_INTENSITY
 * Set street lights intensity
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_STREETLIGHTS_INTENSITY "Network/ILightSimulation/LIGHTSIMULATION_SET_STREETLIGHTS_INTENSITY"

/**
 * Method LIGHTSIMULATION_SET_FUNCTION_TYPE_LUMINANCE
 * Set the luminance to a headlamp corresponding to a given optical function type.
 *               Optical funciton types:
 *               0: Low beams
 *               1: High beams
 *               2: Fog beams
 *               3: Additional lights
 *               4: Tail lights
 *               5: Stop
 *               6: Reverse
 *               7: Rear fog
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_FUNCTION_TYPE_LUMINANCE "Network/ILightSimulation/LIGHTSIMULATION_SET_FUNCTION_TYPE_LUMINANCE"

/**
 * Method LIGHTSIMULATION_MATERIAL_INFO
 * Sends the name of a geometry in the scene and its material properties
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_MATERIAL_INFO "Network/ILightSimulation/LIGHTSIMULATION_MATERIAL_INFO"

/**
 * Method LIGHTSIMULATION_BACKGROUND_LUMINANCE
 * Background luminance
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_BACKGROUND_LUMINANCE "Network/ILightSimulation/LIGHTSIMULATION_BACKGROUND_LUMINANCE"

/**
 * Method LIGHTSIMULATION_SET_WALL_FROM_FILE
 * Sets the 3D file of an aiming wall. If the wall with the given id does not exist, it will be created.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_WALL_FROM_FILE "Network/ILightSimulation/LIGHTSIMULATION_SET_WALL_FROM_FILE"

/**
 * Method LIGHTSIMULATION_SET_WALL_FROM_MEASURE
 * Sets the width and the height of an aiming wall. If the wall with the given id does not exist, it will be created
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_WALL_FROM_MEASURE "Network/ILightSimulation/LIGHTSIMULATION_SET_WALL_FROM_MEASURE"

/**
 * Method LIGHTSIMULATION_DELETE_WALL
 * Delete an aiming wall.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DELETE_WALL "Network/ILightSimulation/LIGHTSIMULATION_DELETE_WALL"

/**
 * Method LIGHTSIMULATION_DISPLAY_TERRAIN
 * Display or hide the visual terrain
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DISPLAY_TERRAIN "Network/ILightSimulation/LIGHTSIMULATION_DISPLAY_TERRAIN"

/**
 * Method LIGHTSIMULATION_SET_DISTANCE_WALL
 * Sets the aiming wall X distance from the headlight reference frame.
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_SET_DISTANCE_WALL "Network/ILightSimulation/LIGHTSIMULATION_SET_DISTANCE_WALL"

/**
 * Method LIGHTSIMULATION_DISPLAY_WALL
 * Display or hide the specified aiming wall
 **/
#define NETWORK_ILIGHTSIMULATION_LIGHTSIMULATION_DISPLAY_WALL "Network/ILightSimulation/LIGHTSIMULATION_DISPLAY_WALL"

/**
 * Method VocalSend
 * play a pre recorded message
 **/
#define NETWORK_IINFORMATION_VOCALSEND "Network/IInformation/VocalSend"

/**
 * Method sendVisualConstantTextMessage
 * Send a string message to the visual. The message is displayed on the defined AreaId.
 **/
#define NETWORK_IINFORMATION_SENDVISUALCONSTANTTEXTMESSAGE "Network/IInformation/sendVisualConstantTextMessage"

/**
 * Method sendVisualInteractiveTextMessage
 * Send a built message to the visual. message = beginString+[channelN1]+endString+[channelN2]. The message is displayed on the defined AreaId.
 **/
#define NETWORK_IINFORMATION_SENDVISUALINTERACTIVETEXTMESSAGE "Network/IInformation/sendVisualInteractiveTextMessage"

/**
 * Method sendVisualDisplayImage
 * Tells the visual module to display an image. It needs to be loaded before through sendVisualImageLoad function.
 **/
#define NETWORK_IINFORMATION_SENDVISUALDISPLAYIMAGE "Network/IInformation/sendVisualDisplayImage"

/**
 * Method sendSoundLoad
 * Ask the sound module to load a sound file. This sould be called during the SCANeR load state.
 **/
#define NETWORK_IINFORMATION_SENDSOUNDLOAD "Network/IInformation/sendSoundLoad"

/**
 * Method sendSoundPlay
 * Ask the sound module to play a sound. The sound should be fisrst loaded with sendSoundLoad
 **/
#define NETWORK_IINFORMATION_SENDSOUNDPLAY "Network/IInformation/sendSoundPlay"

/**
 * Method sendSoundStop
 * Ask the sound module to stop playing a sound.
 **/
#define NETWORK_IINFORMATION_SENDSOUNDSTOP "Network/IInformation/sendSoundStop"

/**
 * Method sendSoundAttachToVehicle
 * Ask the sound module to attach a sound to a specific vehicle. The sound should be first loaded with sendSoundLoad
 **/
#define NETWORK_IINFORMATION_SENDSOUNDATTACHTOVEHICLE "Network/IInformation/sendSoundAttachToVehicle"

/**
 * Method sendSoundAttachToPoint
 * Ask the sound module to attach a sound to a specific point. The sound should be fisrst loaded with sendSoundLoad
 **/
#define NETWORK_IINFORMATION_SENDSOUNDATTACHTOPOINT "Network/IInformation/sendSoundAttachToPoint"

/**
 * Method sendSoundVolume
 * Ask the sound module to modify the volume for a specific sound.
 **/
#define NETWORK_IINFORMATION_SENDSOUNDVOLUME "Network/IInformation/sendSoundVolume"

/**
 * Method sendSoundFrequency
 * Ask the sound module to modify the frequency for a specific sound.
 **/
#define NETWORK_IINFORMATION_SENDSOUNDFREQUENCY "Network/IInformation/sendSoundFrequency"

/**
 * Method sendSoundPause
 * Ask the sound module to pause a sound.
 **/
#define NETWORK_IINFORMATION_SENDSOUNDPAUSE "Network/IInformation/sendSoundPause"

/**
 * Method sendDisplayUrl
 * Display a HTML page on a Display Module
 **/
#define NETWORK_IINFORMATION_SENDDISPLAYURL "Network/IInformation/sendDisplayUrl"

/**
 * Method sendDisplayText
 * Display a Text page on a Display Module
 **/
#define NETWORK_IINFORMATION_SENDDISPLAYTEXT "Network/IInformation/sendDisplayText"

/**
 * Method sendDisplayImage
 * Display an Image page on a Display Module
 **/
#define NETWORK_IINFORMATION_SENDDISPLAYIMAGE "Network/IInformation/sendDisplayImage"

/**
 * Method sendPrintUrl
 * Print a HTML page on a Display Module
 **/
#define NETWORK_IINFORMATION_SENDPRINTURL "Network/IInformation/sendPrintUrl"

/**
 * Method sendVisualSnapshot
 * take a snapshot. It can be a snapshot of the final image or special snapshots according to the given type
 **/
#define NETWORK_IINFORMATION_SENDVISUALSNAPSHOT "Network/IInformation/sendVisualSnapshot"

/**
 * Method sendVisualImageLoad
 * Ask the visual module to load an image. To display the loaded image, use the sendVisualDisplayImage function.
 **/
#define NETWORK_IINFORMATION_SENDVISUALIMAGELOAD "Network/IInformation/sendVisualImageLoad"

/**
 * Method sendVisualImageUnload
 * Ask the visual module to unload an image.
 **/
#define NETWORK_IINFORMATION_SENDVISUALIMAGEUNLOAD "Network/IInformation/sendVisualImageUnload"

/**
 * Method sendChangeWindowResolution
 * Change the window resolution
 **/
#define NETWORK_IINFORMATION_SENDCHANGEWINDOWRESOLUTION "Network/IInformation/sendChangeWindowResolution"

/**
 * Method sendVideoMessage
 * Start/Stop the recording of the visual
 **/
#define NETWORK_IINFORMATION_SENDVIDEOMESSAGE "Network/IInformation/sendVideoMessage"

/**
 * Method sendVisualEnableDisplayInfos
 * Enable or disable the display of informations on the given visual channel
 **/
#define NETWORK_IINFORMATION_SENDVISUALENABLEDISPLAYINFOS "Network/IInformation/sendVisualEnableDisplayInfos"

/**
 * Method ScenarioResult
 * Sent by Scenario process at the end of a scenario.
 **/
#define NETWORK_ISCENARIO_SCENARIORESULT "Network/IScenario/ScenarioResult"

/**
 * Method ScenarioTask
 * Sent by Scenario process when a task is starting. 
 **/
#define NETWORK_ISCENARIO_SCENARIOTASK "Network/IScenario/ScenarioTask"

/**
 * Method ScenarioRule
 * Sent by Scenario process when a rule changes its evaluation result.
 **/
#define NETWORK_ISCENARIO_SCENARIORULE "Network/IScenario/ScenarioRule"

/**
 * Method EvaluationInitCategory
 * inits criterias category properties
 **/
#define NETWORK_ISCENARIO_EVALUATIONINITCATEGORY "Network/IScenario/EvaluationInitCategory"

/**
 * Method EvaluationInitScoreForSuccess
 * inits scores for each criteria type
 **/
#define NETWORK_ISCENARIO_EVALUATIONINITSCOREFORSUCCESS "Network/IScenario/EvaluationInitScoreForSuccess"

/**
 * Method EvaluationInitCriteria
 * inits criteria properties
 **/
#define NETWORK_ISCENARIO_EVALUATIONINITCRITERIA "Network/IScenario/EvaluationInitCriteria"

/**
 * Method EvaluationInitCriticity
 * inits criteria criticity. Optionnal
 **/
#define NETWORK_ISCENARIO_EVALUATIONINITCRITICITY "Network/IScenario/EvaluationInitCriticity"

/**
 * Method EvaluationInitColumn
 * inits an exercise result tab column
 **/
#define NETWORK_ISCENARIO_EVALUATIONINITCOLUMN "Network/IScenario/EvaluationInitColumn"

/**
 * Method EvaluationSetValue
 * sets the value of an evaluation criteria
 **/
#define NETWORK_ISCENARIO_EVALUATIONSETVALUE "Network/IScenario/EvaluationSetValue"

/**
 * Method EvaluationRequestCriteriaParameters
 * to be use when receiving an unknown cat/cri/column
 **/
#define NETWORK_ISCENARIO_EVALUATIONREQUESTCRITERIAPARAMETERS "Network/IScenario/EvaluationRequestCriteriaParameters"

/**
 * Method RecallObjective
 * Sent by the session manager, when RECALL OBJECTIVE button is requested. The exercise manager will ask the visual to display the objective
 **/
#define NETWORK_ISCENARIO_RECALLOBJECTIVE "Network/IScenario/RecallObjective"

/**
 * Method EvaluationFinalColumnScore
 * sets the final evaluation score for a type (between error and objective)
 **/
#define NETWORK_ISCENARIO_EVALUATIONFINALCOLUMNSCORE "Network/IScenario/EvaluationFinalColumnScore"

/**
 * Method EvaluationResult
 * sends true if evaluation succeeded, false otherwise
 **/
#define NETWORK_ISCENARIO_EVALUATIONRESULT "Network/IScenario/EvaluationResult"

/**
 * Method ScriptSetActivation
 * Sends true if the script is activated false otherwise
 **/
#define NETWORK_ISCENARIO_SCRIPTSETACTIVATION "Network/IScenario/ScriptSetActivation"

/**
 * Method ScriptSetVariable
 * Set the variable of the specified script
 **/
#define NETWORK_ISCENARIO_SCRIPTSETVARIABLE "Network/IScenario/ScriptSetVariable"

/**
 * Method TriggerActivationState
 *  **/
#define NETWORK_ISCENARIO_TRIGGERACTIVATIONSTATE "Network/IScenario/TriggerActivationState"

/**
 * Method TriggeredState
 *  **/
#define NETWORK_ISCENARIO_TRIGGEREDSTATE "Network/IScenario/TriggeredState"

/**
 * Method SetSourceActivation
 *  **/
#define NETWORK_ISCENARIO_SETSOURCEACTIVATION "Network/IScenario/SetSourceActivation"

/**
 * Method SetSinkActivation
 *  **/
#define NETWORK_ISCENARIO_SETSINKACTIVATION "Network/IScenario/SetSinkActivation"

/**
 * Method SetSourceFlow
 *  **/
#define NETWORK_ISCENARIO_SETSOURCEFLOW "Network/IScenario/SetSourceFlow"

/**
 * Method ScriptOutput
 * Send the value of the script output
 **/
#define NETWORK_ISCENARIO_SCRIPTOUTPUT "Network/IScenario/ScriptOutput"

/**
 * Method PositionDevice6D_Pos
 * For example of Polhemus type.The deviceId allows to identify the sensor when several are used (head and hands for example).time is time of acquisition.
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONDEVICE6D_POS "Network/IPositionDevice/PositionDevice6D_Pos"

/**
 * Method PositionDevice6D_Speed
 *  **/
#define NETWORK_IPOSITIONDEVICE_POSITIONDEVICE6D_SPEED "Network/IPositionDevice/PositionDevice6D_Speed"

/**
 * Method PositionDevice6D_Acc
 *  **/
#define NETWORK_IPOSITIONDEVICE_POSITIONDEVICE6D_ACC "Network/IPositionDevice/PositionDevice6D_Acc"

/**
 * Method PositionDeviceButton
 * For example of Division or Space mouse type.buttonId has to be understood by receiving process.status is 0 for released, 1 for depressed.time is time of acquisition.
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONDEVICEBUTTON "Network/IPositionDevice/PositionDeviceButton"

/**
 * Method PositionEyeTracker
 * POR position
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONEYETRACKER "Network/IPositionDevice/PositionEyeTracker"

/**
 * Method PositionEyeTrackerFull
 * EyeTracker full structure
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONEYETRACKERFULL "Network/IPositionDevice/PositionEyeTrackerFull"

/**
 * Method PositionEyeTrackerShowTargetHud
 * Show/Hide the target crossbar in visuals
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONEYETRACKERSHOWTARGETHUD "Network/IPositionDevice/PositionEyeTrackerShowTargetHud"

/**
 * Method PositionEyeTrackerRawData
 * EyeTracker raw data
 **/
#define NETWORK_IPOSITIONDEVICE_POSITIONEYETRACKERRAWDATA "Network/IPositionDevice/PositionEyeTrackerRawData"

/**
 * Method VideoSync
 * Synchronisation information for VIDEO module
 **/
#define NETWORK_ISYNCHRONIZE_VIDEOSYNC "Network/ISynchronize/VideoSync"

/**
 * Method ConnectChannel
 * Not used anymore
 **/
#define NETWORK_IUSER_CONNECTCHANNEL "Network/IUser/ConnectChannel"

/**
 * Method ExportChannel
 * Sent by Scenario process.
 **/
#define NETWORK_IUSER_EXPORTCHANNEL "Network/IUser/ExportChannel"

/**
 * Method UserInput
 * Sent by any user defined process to the scenario.
 **/
#define NETWORK_IUSER_USERINPUT "Network/IUser/UserInput"

/**
 * Method StimuliFloat
 *  **/
#define NETWORK_ISTIMULI_STIMULIFLOAT "Network/IStimuli/StimuliFloat"

/**
 * Method StimuliString
 *  **/
#define NETWORK_ISTIMULI_STIMULISTRING "Network/IStimuli/StimuliString"

/**
 * Method StimuliLong
 *  **/
#define NETWORK_ISTIMULI_STIMULILONG "Network/IStimuli/StimuliLong"

/**
 * Method AnimationLoad
 *  **/
#define NETWORK_ISTIMULI_ANIMATIONLOAD "Network/IStimuli/AnimationLoad"

/**
 * Method AnimationValue
 *  **/
#define NETWORK_ISTIMULI_ANIMATIONVALUE "Network/IStimuli/AnimationValue"

/**
 * Method AnimationCtrl
 *  **/
#define NETWORK_ISTIMULI_ANIMATIONCTRL "Network/IStimuli/AnimationCtrl"

/**
 * Method MotionLinearActuatorOffset
 *  **/
#define NETWORK_IPLATFORM_MOTIONLINEARACTUATOROFFSET "Network/IPlatform/MotionLinearActuatorOffset"

/**
 * Method MotionRotationActuatorOffset
 *  **/
#define NETWORK_IPLATFORM_MOTIONROTATIONACTUATOROFFSET "Network/IPlatform/MotionRotationActuatorOffset"

/**
 * Method MotionOffset
 *  **/
#define NETWORK_IPLATFORM_MOTIONOFFSET "Network/IPlatform/MotionOffset"

/**
 * Method MotionEffect
 *  **/
#define NETWORK_IPLATFORM_MOTIONEFFECT "Network/IPlatform/MotionEffect"

/**
 * Method MotionEffectPlayer
 * Effect to apply on platform by the Oktal EffectPlayer.
 **/
#define NETWORK_IPLATFORM_MOTIONEFFECTPLAYER "Network/IPlatform/MotionEffectPlayer"

/**
 * Method DynamicReplayCommand
 * Command sent from replay GUI to vehicle player
 **/
#define NETWORK_IPLATFORM_DYNAMICREPLAYCOMMAND "Network/IPlatform/DynamicReplayCommand"

/**
 * Method DynamicReplayState
 * State sent from vehicle player to replay GUI
 **/
#define NETWORK_IPLATFORM_DYNAMICREPLAYSTATE "Network/IPlatform/DynamicReplayState"

/**
 * Method MotionPlatformCommand
 * Command the platform state
 **/
#define NETWORK_IPLATFORM_MOTIONPLATFORMCOMMAND "Network/IPlatform/MotionPlatformCommand"

/**
 * Method Acknowledge
 *  **/
#define NETWORK_ICONTROL_ACKNOWLEDGE "Network/IControl/Acknowledge"

/**
 * Method Command
 *  **/
#define NETWORK_ICONTROL_COMMAND "Network/IControl/Command"

/**
 * Method State
 *  **/
#define NETWORK_ICONTROL_STATE "Network/IControl/State"

/**
 * Method File
 *  **/
#define NETWORK_ICONTROL_FILE "Network/IControl/File"

/**
 * Method WatchDog
 *  **/
#define NETWORK_ISECURITY_WATCHDOG "Network/ISecurity/WatchDog"

/**
 * Method SecurityInputs
 *  **/
#define NETWORK_ISECURITY_SECURITYINPUTS "Network/ISecurity/SecurityInputs"

/**
 * Method SecurityAuthorizations
 *  **/
#define NETWORK_ISECURITY_SECURITYAUTHORIZATIONS "Network/ISecurity/SecurityAuthorizations"

/**
 * Method SafetyRequirment
 *  **/
#define NETWORK_ISECURITY_SAFETYREQUIRMENT "Network/ISecurity/SafetyRequirment"

/**
 * Method DisplayCockpit
 * Display or not the cockpit set by ID.
 **/
#define NETWORK_IMULTICOCKPIT_DISPLAYCOCKPIT "Network/IMultiCockpit/DisplayCockpit"

/**
 * Method CockpitOpacity
 * Set the opacity of the cockpit set by ID.
 **/
#define NETWORK_IMULTICOCKPIT_COCKPITOPACITY "Network/IMultiCockpit/CockpitOpacity"

/**
 * Method SelectCockpitObserver
 * Set the current oberver matching the cockpit set by ID.
 **/
#define NETWORK_IMULTICOCKPIT_SELECTCOCKPITOBSERVER "Network/IMultiCockpit/SelectCockpitObserver"

/**
 * Method LoadCockpit
 * sets the cockpit file to load in the Visual. The fileName parameter contains only the cockpit's name, without path nor file extension.
 **/
#define NETWORK_IMULTICOCKPIT_LOADCOCKPIT "Network/IMultiCockpit/LoadCockpit"

/**
 * Method SetVariant
 * Enable / Disable a Variant
 **/
#define NETWORK_IMULTICOCKPIT_SETVARIANT "Network/IMultiCockpit/SetVariant"

/**
 * Method SetCockpitEyePoint
 * Change the current eye point of the cockpit
 **/
#define NETWORK_IMULTICOCKPIT_SETCOCKPITEYEPOINT "Network/IMultiCockpit/SetCockpitEyePoint"

/**
 * Method FrustumUpdate
 * Ask update of the camera frustum
 **/
#define NETWORK_ICALIBRATION_FRUSTUMUPDATE "Network/ICalibration/FrustumUpdate"

/**
 * Method SelectionUpdate
 * Set a visual process as selected.
 **/
#define NETWORK_ICALIBRATION_SELECTIONUPDATE "Network/ICalibration/SelectionUpdate"

/**
 * Method LoadImage
 * Preload the calibration picture.
 **/
#define NETWORK_ICALIBRATION_LOADIMAGE "Network/ICalibration/LoadImage"

/**
 * Method SetAliveBeingBehaviour
 *  **/
#define NETWORK_IALIVEBEING_SETALIVEBEINGBEHAVIOUR "Network/IAliveBeing/SetAliveBeingBehaviour"

/**
 * Method SetAliveBeingVisualAnimation
 *  **/
#define NETWORK_IALIVEBEING_SETALIVEBEINGVISUALANIMATION "Network/IAliveBeing/SetAliveBeingVisualAnimation"

/**
 * Method Collision
 *  **/
#define NETWORK_IPHYSICS_COLLISION "Network/IPhysics/Collision"

/**
 * Method InfraPosition
 * Infrastructure object position, pass has 3 value ( x y z) and 3 angles (heading(/z axis) then pitch(y axis) then roll(x axis)).
 **/
#define NETWORK_IPHYSICS_INFRAPOSITION "Network/IPhysics/InfraPosition"

/**
 * Method Join
 * Join 2 physic entities, not vehicle
 **/
#define NETWORK_IPHYSICS_JOIN "Network/IPhysics/Join"

/**
 * Method Disjoin
 * Disjoin 2 physic entities, do nothing if they are not joined
 **/
#define NETWORK_IPHYSICS_DISJOIN "Network/IPhysics/Disjoin"

/**
 * Method JoinVehicle
 * Join a physic entity from a vehicle
 **/
#define NETWORK_IPHYSICS_JOINVEHICLE "Network/IPhysics/JoinVehicle"

/**
 * Method DisjoinVehicle
 * Disjoin a physic entity from a vehicle, do nothing if they are not joined
 **/
#define NETWORK_IPHYSICS_DISJOINVEHICLE "Network/IPhysics/DisjoinVehicle"

/**
 * Method SetPosition
 * Set the position of a physic entity
 **/
#define NETWORK_IPHYSICS_SETPOSITION "Network/IPhysics/SetPosition"

/**
 * Method SetObjectState
 * Set the state of the given object to the given state
 **/
#define NETWORK_IPHYSICS_SETOBJECTSTATE "Network/IPhysics/SetObjectState"

/**
 * Method GroundObjectAltitude
 * modifie a patch of a ground object altitude
 **/
#define NETWORK_IPHYSICS_GROUNDOBJECTALTITUDE "Network/IPhysics/GroundObjectAltitude"

/**
 * Method LoadSNTFile
 * sets the SNT file to load in the Night Test Manager
 **/
#define NETWORK_INIGHTTEST_LOADSNTFILE "Network/INightTest/LoadSNTFile"

/**
 * Method nightTestSwitchProduct
 * Switch to a product in Night Test Manager by specifying the product index value
 **/
#define NETWORK_INIGHTTEST_NIGHTTESTSWITCHPRODUCT "Network/INightTest/nightTestSwitchProduct"

/**
 * Method nightTestProductOffset
 * Switch to the next or previous product in Night Test Manager by specifying an index offset
 **/
#define NETWORK_INIGHTTEST_NIGHTTESTPRODUCTOFFSET "Network/INightTest/nightTestProductOffset"

/**
 * Method nightTestEnableCockpit
 * Activate the display of the cockpit
 **/
#define NETWORK_INIGHTTEST_NIGHTTESTENABLECOCKPIT "Network/INightTest/nightTestEnableCockpit"

/**
 * Method GhostBeginRecord
 * Start record new ghost
 **/
#define NETWORK_IGHOST_GHOSTBEGINRECORD "Network/IGhost/GhostBeginRecord"

/**
 * Method GhostStopRecord
 * Stop and save record
 **/
#define NETWORK_IGHOST_GHOSTSTOPRECORD "Network/IGhost/GhostStopRecord"

/**
 * Method GhostBeginReplay
 * Start playing the specified ghost
 **/
#define NETWORK_IGHOST_GHOSTBEGINREPLAY "Network/IGhost/GhostBeginReplay"

/**
 * Method GhostStopReplay
 * Stop ghost playing
 **/
#define NETWORK_IGHOST_GHOSTSTOPREPLAY "Network/IGhost/GhostStopReplay"

/**
 * Method GhostPauseReplay
 * Pause ghost playing
 **/
#define NETWORK_IGHOST_GHOSTPAUSEREPLAY "Network/IGhost/GhostPauseReplay"

/**
 * Method GhostUnpauseReplay
 * Unpause ghost playing
 **/
#define NETWORK_IGHOST_GHOSTUNPAUSEREPLAY "Network/IGhost/GhostUnpauseReplay"

/**
 * Method GhostCreated
 * Send on ghost creation
 **/
#define NETWORK_IGHOST_GHOSTCREATED "Network/IGhost/GhostCreated"

/**
 * Method GhostReplayFinished
 * Send when a replay is finished
 **/
#define NETWORK_IGHOST_GHOSTREPLAYFINISHED "Network/IGhost/GhostReplayFinished"

/**
 * Method TimeMarker
 * Send a time marker
 **/
#define NETWORK_ITIMEMARKER_TIMEMARKER "Network/ITimeMarker/TimeMarker"

/**
 * Method SequenceStop
 * Send a sequence stop
 **/
#define NETWORK_ITIMEMARKER_SEQUENCESTOP "Network/ITimeMarker/SequenceStop"

/**
 * Method LaserMeter
 * Anounce arriving results for Laser m type sensors
 **/
#define NETWORK_ISENSOR_LASERMETER "Network/ISensor/LaserMeter"

/**
 * Method GPS
 * GPS sensor
 **/
#define NETWORK_ISENSOR_GPS "Network/ISensor/GPS"

/**
 * Method SensorSetPosition
 * Set sensor position.
 **/
#define NETWORK_ISENSOR_SENSORSETPOSITION "Network/ISensor/SensorSetPosition"

/**
 * Method SensorSetVisible
 * Set sensor visibility.
 **/
#define NETWORK_ISENSOR_SENSORSETVISIBLE "Network/ISensor/SensorSetVisible"

/**
 * Method SensorTargets
 * List of targets detected by a sensor.
 **/
#define NETWORK_ISENSOR_SENSORTARGETS "Network/ISensor/SensorTargets"

/**
 * Method SensorDetectedPoint
 * A point detected by a sensor.
 **/
#define NETWORK_ISENSOR_SENSORDETECTEDPOINT "Network/ISensor/SensorDetectedPoint"

/**
 * Method RoadSensorDetectedPoints
 * List of points detected by a road sensor.
 **/
#define NETWORK_ISENSOR_ROADSENSORDETECTEDPOINTS "Network/ISensor/RoadSensorDetectedPoints"

/**
 * Method RoadSensorSetDistanceBetweenScans
 * Set distance between scans for a road sensor.
 **/
#define NETWORK_ISENSOR_ROADSENSORSETDISTANCEBETWEENSCANS "Network/ISensor/RoadSensorSetDistanceBetweenScans"

/**
 * Method SensorTargetsBoundingBoxes
 * List of detected targets bounding boxes.
 **/
#define NETWORK_ISENSOR_SENSORTARGETSBOUNDINGBOXES "Network/ISensor/SensorTargetsBoundingBoxes"

/**
 * Method LaserMeterLite
 * Anounce arriving results for Laser m type sensors
 **/
#define NETWORK_ISENSOR_LASERMETERLITE "Network/ISensor/LaserMeterLite"

/**
 * Method RoadLinesPoints
 * List of detected line road markings.
 **/
#define NETWORK_ISENSOR_ROADLINESPOINTS "Network/ISensor/RoadLinesPoints"

/**
 * Method RoadLanesPoints
 * List of detected road lanes.
 **/
#define NETWORK_ISENSOR_ROADLANESPOINTS "Network/ISensor/RoadLanesPoints"

/**
 * Method SensorMovableTargets
 * List of targets detected by a sensor
 **/
#define NETWORK_ISENSOR_SENSORMOVABLETARGETS "Network/ISensor/SensorMovableTargets"

/**
 * Method SensorInfrastructureTargets
 * List of infrastructure targets detected by a sensor
 **/
#define NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETS "Network/ISensor/SensorInfrastructureTargets"

/**
 * Method SensorMovableTargetsTrafficStates
 * List of targets detected by a sensor
 **/
#define NETWORK_ISENSOR_SENSORMOVABLETARGETSTRAFFICSTATES "Network/ISensor/SensorMovableTargetsTrafficStates"

/**
 * Method SensorInfrastructureTargetsTrafficStates
 * Complementary to list infrastructure target
 **/
#define NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETSTRAFFICSTATES "Network/ISensor/SensorInfrastructureTargetsTrafficStates"

/**
 * Method RoadLinesPolynoms
 * List of detected lines polynoms.
 **/
#define NETWORK_ISENSOR_ROADLINESPOLYNOMS "Network/ISensor/RoadLinesPolynoms"

/**
 * Method RoadLanesTrafficStates
 * Complementary to list Lanes traffic state
 **/
#define NETWORK_ISENSOR_ROADLANESTRAFFICSTATES "Network/ISensor/RoadLanesTrafficStates"

/**
 * Method RoadLinesTrafficStates
 * Complementary to list line traffic state
 **/
#define NETWORK_ISENSOR_ROADLINESTRAFFICSTATES "Network/ISensor/RoadLinesTrafficStates"

/**
 * Method RoadLanesPolynoms
 * List of detected lanes polynoms.
 **/
#define NETWORK_ISENSOR_ROADLANESPOLYNOMS "Network/ISensor/RoadLanesPolynoms"

/**
 * Method SensorMovableTargetsBoundingBoxes
 * List of detected movable targets bounding boxes.
 **/
#define NETWORK_ISENSOR_SENSORMOVABLETARGETSBOUNDINGBOXES "Network/ISensor/SensorMovableTargetsBoundingBoxes"

/**
 * Method SensorInfrastructureTargetsBoundingBoxes
 * List of detected infrastructure targets bounding boxes.
 **/
#define NETWORK_ISENSOR_SENSORINFRASTRUCTURETARGETSBOUNDINGBOXES "Network/ISensor/SensorInfrastructureTargetsBoundingBoxes"

/**
 * Method LidarPoints
 * Informations about the LIDAR Sensor
 **/
#define NETWORK_ISENSOR_LIDARPOINTS "Network/ISensor/LidarPoints"

/**
 * Method Osi
 * OSI messages
 **/
#define NETWORK_ISENSOR_OSI "Network/ISensor/Osi"

/**
 * Method ArbitraryLinesPoints
 * List of detected arbitrary line on the road.
 **/
#define NETWORK_ISENSOR_ARBITRARYLINESPOINTS "Network/ISensor/ArbitraryLinesPoints"

/**
 * Method PhysicalRadarLVL2
 *  **/
#define NETWORK_ISENSOR_PHYSICALRADARLVL2 "Network/ISensor/PhysicalRadarLVL2"

/**
 * Method SetObjectDetectionState
 * Set the detection state of the given object to the given state
 **/
#define NETWORK_ISENSOR_SETOBJECTDETECTIONSTATE "Network/ISensor/SetObjectDetectionState"

/**
 * Method V2XMsg
 * Message emitted by a vehicle
 **/
#define NETWORK_IV2X2V_V2XMSG "Network/IV2X2V/V2XMsg"

/**
 * Method X2VMsg
 * Message for a vehicle
 **/
#define NETWORK_IV2X2V_X2VMSG "Network/IV2X2V/X2VMsg"
